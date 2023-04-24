/**
 * @file ranging.cpp
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief Ranging Related function
 * @version 0.1
 * @date 2022-09-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <zephyr/zephyr.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include <unordered_map>
#include <iostream>
#include <string.h>
#include <math.h>

#include <zephyr/random/rand32.h>

#include "uwb_transport.h"
#include "common_macro.h"
#include "common_variables.h"
#include "ranging.h"

#include "mac.h"
#include "uart.h"

#include "node.h"

LOG_MODULE_REGISTER(ranging, 2);

typedef std::unordered_map<uint32_t, struct node_t *> devices_map_t;

devices_map_t devices_map;

/**
 * @brief Decode ranging buffer
 * 
 * @param ranging_pkt pointer to ranging paket
 * @param buffer_rx pointer to received buffer
 * @return int returns length of message
 */
int decode_ranging_pkt(struct ranging_pkt_t *ranging_pkt, const uint8_t *buffer_rx)
{
    memcpy(&ranging_pkt->packet_number, buffer_rx, sizeof(uint8_t));
    buffer_rx += sizeof(uint8_t);
    memcpy(&ranging_pkt->RoundB, buffer_rx, sizeof(uint32_t));
    buffer_rx += sizeof(uint32_t);
    memcpy(&ranging_pkt->DelayB, buffer_rx, sizeof(uint32_t));
    buffer_rx += sizeof(uint32_t);
    memcpy(&ranging_pkt->DelayA, buffer_rx, sizeof(uint32_t));
    buffer_rx += sizeof(uint32_t);

    return ENCODED_RANGING_PKT_LENGTH;
}

/**
 * @brief 
 * 
 * @param ranging_pkt pointer to ranging paket
 * @param buffer_tx pointer to transmit buffer
 * @return int returns length of message
 */
int encode_ranging_pkt(const struct ranging_pkt_t *ranging_pkt, uint8_t *buffer_tx)
{
    memcpy(buffer_tx, &ranging_pkt->packet_number, sizeof(uint8_t));
    buffer_tx += sizeof(uint8_t);
    memcpy(buffer_tx, &ranging_pkt->RoundA, sizeof(uint32_t));
    buffer_tx += sizeof(uint32_t);
    memcpy(buffer_tx, &ranging_pkt->DelayA, sizeof(uint32_t));
    buffer_tx += sizeof(uint32_t);
    memcpy(buffer_tx, &ranging_pkt->DelayB, sizeof(uint32_t));
    buffer_tx += sizeof(uint32_t);

    return ENCODED_RANGING_PKT_LENGTH;
}

/**
 * @brief Sends ranging request to target_id
 * 
 * @param target_id MAC address of target node
 * @param preprocessing Size of averaging windows
 */
void request_ranging(uint16_t target_id, int preprocessing)
{
    LOG_INF("Requesting ranging for 0x%X and window size %d", target_id, preprocessing);

    if (not devices_map.contains(target_id))
    {
        struct node_t *node = (struct node_t *)k_calloc(1, sizeof(struct node_t));
        __ASSERT(node != NULL, "Allocating device pointer Failed!");

        node_init(node, target_id, preprocessing);

        devices_map[target_id] = node;
    }
    struct node_t *node = devices_map[target_id];

    if(!stats_is_initialized(&node->ranging.stats))
        stats_init(&node->ranging.stats, preprocessing);

    if(preprocessing != stats_get_window_size(&node->ranging.stats))
    {
        stats_destroy(&node->ranging.stats);
        stats_init(&node->ranging.stats, preprocessing);
    }

    node->ranging.role = INITIATOR;
    node->ranging.ranging_state = ENABLED;
    return;
}

/**
 * @brief Disable ranging for target_id
 * 
 * @param target_id MAC address of target node
 */
void disable_ranging(uint16_t target_id)
{
    if (not devices_map.contains(target_id))
        return;

    struct node_t *node = devices_map[target_id];

    node->ranging.ranging_state = DISABLED;
    return;
}

/**
 * ? Since we want to introduce sime delay between received packet and transimted packet
 * ? The best way is to use scheduled work
 */
struct scheduled_ranging
{
    struct k_work_delayable work;
    uint8_t buffer[ENCODED_RANGING_PKT_LENGTH];
    uint16_t destination;
    struct tx_details_t tx_details;
};

void submit_handler(struct k_work *item)
{
    struct scheduled_ranging *data = CONTAINER_OF(item, struct scheduled_ranging, work);
    write_uwb(data->destination, DATA, RANGING_MSG_TYPE, (const uint8_t *)data->buffer, sizeof(data->buffer), &data->tx_details);

    k_free(data);
    return;
}

/**
 * @brief Calculate TOD from Ra, Da, Rb, Db
 * 
 * @param data pointer to ranging packet
 * @return float calculated time of flight
 */
double ToF_DS(const ranging_pkt_t *data)
{
    double Ra, Rb, Da, Db;

    Ra = (double) data->RoundA;
    Da = (double) data->DelayA;
    Rb = (double) data->RoundB;
    Db = (double) data->DelayB;

    double tof = (double)DWT_TIME_UNITS * (Ra * Rb - Da * Db) / (Ra + Da + Rb + Db);

    return tof;
}

/**
 * @brief Function to handle Double sided ranging
 * 
 * @param source_id MAC address of received packet
 * @param msg received buffer
 * @param rx_details Details about received packet
 * @return int status (Not actually used)
 */
int rx_ranging_ds(const uint16_t source_id, void *msg, struct rx_details_t *rx_details)
{
    int status;
    uint8_t *msg_uint8_t = (uint8_t *)msg;

    LOG_HEXDUMP_DBG(msg, ENCODED_RANGING_PKT_LENGTH, "Received message");

    ranging_pkt_t ranging_pkt;
    decode_ranging_pkt(&ranging_pkt, (const uint8_t *)msg);

    //  Create node if not existing yet
    if (not devices_map.contains(source_id))
    {
        struct node_t *node = (struct node_t *)k_calloc(1, sizeof(struct node_t));
        __ASSERT(node != NULL, "Allocating device pointer Failed!");

        node_init(node, source_id, 0);
        node->ranging.ranging_state = ENABLED;
        devices_map[source_id] = node;
    }
    struct node_t *node = devices_map[source_id];

    if(node->ranging.ranging_state == DISABLED)
        return 0;

    // Handle Packet number
    if (ranging_pkt.packet_number == 0)
    {
        node->ranging.role = RESPONDER;
        LOG_INF("New ranging instance!, packet number = %d", ranging_pkt.packet_number);
    }
    else if (ranging_pkt.packet_number == node->ranging.expected_packet_number)
    {
        LOG_DBG("Packet number is same as expected");
    }
    else
    {
        LOG_WRN("Wrong packet number, discarding the measurment! %u", ranging_pkt.packet_number);
        return 0;
    }

    node->ranging.role = ranging_pkt.packet_number % 2 ? INITIATOR : RESPONDER;

    node->ranging.expected_packet_number = (ranging_pkt.packet_number + 2) / 255 + (ranging_pkt.packet_number + 2) % 255;
    LOG_INF("Current packet number: %u, New expected number: %u", ranging_pkt.packet_number, node->ranging.expected_packet_number);

    ranging_pkt.packet_number = (ranging_pkt.packet_number + 1) / 255 + (ranging_pkt.packet_number + 1) % 255;

    // Update ranging struct status
    node->ranging.last_meas_time = k_uptime_get_32();
    node->ranging.error_counter = 0;

    // Get more data
    if (ranging_pkt.RoundB == 0 && ranging_pkt.RoundA == 0 && ranging_pkt.DelayB == 0)
    {
        node->ranging.tx_timestamp = 0;
    }
    else
    {
        uint64_t rx_timestamp = rx_details->rx_timestamp;
        uint64_t tx_timestamp = node->ranging.tx_timestamp;

        if (rx_timestamp > tx_timestamp)
            ranging_pkt.RoundA = rx_timestamp - tx_timestamp;
        else
            ranging_pkt.RoundA = rx_timestamp + (0x000000ffffffffffU - tx_timestamp);
    }

    if(ranging_pkt.packet_number < 4)
    {
        // Setup reply
        struct tx_details_t tx_details = {
            .ranging = 1,
            .tx_mode = DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED,
            .tx_timestamp = &(node->ranging.tx_timestamp),
            .tx_delay = {.rx_timestamp = rx_details->rx_timestamp, .reserved_time = 500, .offset = 5}};

        struct scheduled_ranging *work = (struct scheduled_ranging *)k_malloc(sizeof(struct scheduled_ranging));
        __ASSERT(work != NULL, "Ouch! malloc failed :-(");

        uint8_t *write_buffer = &work->buffer[0];

        encode_ranging_pkt(&ranging_pkt, write_buffer);

        work->destination = source_id;
        work->tx_details = tx_details;

        k_work_init_delayable(&work->work, submit_handler);
        status = k_work_schedule(&work->work, K_MSEC(0 * devices_map.size()));
        __ASSERT(status >= 0, "Error scheduling delayed work, status: %d", status);
    }
    else
    {
        // Setup reply
        struct tx_details_t tx_details = {
            .ranging = 1,
            .tx_mode = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED,
            .tx_timestamp = &(node->ranging.tx_timestamp)
            };

        struct scheduled_ranging *work = (struct scheduled_ranging *)k_malloc(sizeof(struct scheduled_ranging));
        __ASSERT(work != NULL, "Ouch! malloc failed :-(");

        uint32_t empty[] = {0, 0, 0, 0, 0};

        memcpy(&work->buffer[0], empty, ENCODED_RANGING_PKT_LENGTH);

        work->destination = source_id;
        work->tx_details = tx_details;
        node->ranging.expected_packet_number = 1;
        
        float delay = 5e3*sys_rand32_get()/0xffffffff;

        k_work_init_delayable(&work->work, submit_handler);
        status = k_work_schedule(&work->work, K_USEC((int) (10e3 + delay)*devices_map.size()));
        __ASSERT(status >= 0, "Error scheduling delayed work, status: %d", status);
    }

    // Now, calculate the actual if it even makes sense
    if(!stats_is_initialized(&node->ranging.stats))
        return 0;

    if (ranging_pkt.RoundA == 0 || ranging_pkt.DelayA == 0 || ranging_pkt.RoundB == 0 || ranging_pkt.DelayB == 0)
        return 0;

    LOG_DBG("Ra: %d Da: %d Rb: %d Db: %d", ranging_pkt.RoundA, ranging_pkt.DelayA, ranging_pkt.RoundB, ranging_pkt.DelayB);

    double tof = ToF_DS(&ranging_pkt);
    float dist = (float) (SPEED_OF_LIGHT * tof);

    if (abs(dist) > 1000)
    {
        return 0;
    }

    struct statistics_t *stats = &node->ranging.stats;
    stats_update(stats, dist);

    // Send message to queue for ROS_RX thread
    struct uwb_msg_t uwb_msg;

    uwb_msg.msg_type = RANGING_DATA;
    uwb_msg.data.ranging_msg.source_mac = source_id;
    uwb_msg.data.ranging_msg.range = stats_get_mean(stats);
    uwb_msg.data.ranging_msg.variance = stats_get_variance(stats);
    uwb_msg.data.ranging_msg.raw = stats_get_raw(stats);

    LOG_INF("Ranging from ID 0x%X %.2f m | %.4f dev TOF: %.2f ns", source_id, stats_get_mean(stats), stats_get_variance(stats), tof*1e9);

    if(mrs_ranging.control == STANDALONE)
        return 0;

    status = k_msgq_put(&uwb_msgq, &uwb_msg, K_FOREVER);
    __ASSERT(status == 0, "Putting message to queue Failed!");

    return 0;
}

/**
 * @brief Ranging thread periodically checks status of nodes and request ranging if packet lost happend
 */
void uwb_ranging_thread(void)
{
    k_tid_t thread = k_current_get();
    k_thread_suspend(thread);

    // Initialize empty buffer
    uint32_t empty[] = {0, 0, 0, 0, 0};

    LOG_INF("Ranging thread started");

    while (1)
    {
        for (auto node_ptr : devices_map)
        {
            uint32_t device_id = node_ptr.first;
            struct node_t *node = (struct node_t *)node_ptr.second;

            // Check time of last received ranging data
            if (node->ranging.ranging_state == DISABLED || (k_uptime_get_32() - node->ranging.last_meas_time) < 100)
                continue;


            // Handle Error caounter
            node->ranging.error_counter++;

            // After 10 fails switch to initiator role
            if(node->ranging.error_counter > 10)
                node->ranging.role = INITIATOR;

            // After 20 fails delete the node as it is no longer ONLINE
            if (node->ranging.error_counter > 20)
            {
                node_destroy(node);
                devices_map.erase(node_ptr.first);

                LOG_WRN("Deleting node 0x%X", node_ptr.first);

                continue;
            }

            // Onlt INITIATOR has permission to request ranging
            if (node->ranging.role == RESPONDER)
                continue;

            // update status
            node->ranging.last_meas_time = k_uptime_get_32();
            node->ranging.tx_timestamp = 0;
            node->ranging.expected_packet_number = 1;

            // Reset statistics
            stats_reset(&node->ranging.stats);


            LOG_WRN("Requesting ranging to device 0x%X", device_id);
            struct tx_details_t tx_details = {
                .ranging = 1,
                .tx_mode = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED,
                .tx_timestamp = &(node->ranging.tx_timestamp)};

            write_uwb(device_id, DATA, RANGING_MSG_TYPE, (uint8_t *)empty, ENCODED_RANGING_PKT_LENGTH, &tx_details);
        }
        sleep_ms(3);
    }
}
