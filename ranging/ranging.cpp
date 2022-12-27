#include <zephyr/zephyr.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ranging);

#include <unordered_map>
#include <iostream>
#include <string.h>
#include <math.h>

#include <zephyr/random/rand32.h>

#include <pb_encode.h>
#include <pb_decode.h>

#include "uwb_transport.h"
#include "common_macro.h"
#include "common_variables.h"
#include "ranging.h"

#include "mac.h"
#include "uart.h"

#include "node.h"

#define INTEGRATOR_ALPHA 1
#define DISTANCE_ALPHA 1

typedef std::unordered_map<uint32_t, struct node_t *> devices_map_t;
typedef std::unordered_map<uint16_t, uint64_t *> tx_timestamps_map_t;

tx_timestamps_map_t tx_timestamps_map;

devices_map_t devices_map;
K_SEM_DEFINE(print_ranging_semaphore, 0, 1);

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
 * @brief Receiving thread, waits for queue
 *
 */
void ranging_thread(void)
{
    k_tid_t thread = k_current_get();
    k_thread_suspend(thread);

    LOG_INF("Rx thread started");
    struct rx_queue_t data;

    while (1)
    {
        // wait for data
        read_uwb(&data);

        int source_id = data.mac_data.source_id;
        struct rx_details_t *rx_details = &data.rx_details;

        void *offseted_buf = &data.buffer_rx[data.buf_offset];

        int status = SUCCESS;
        switch (data.mac_data.msg_type)
        {
        case BEACON_MSG:
            status = rx_beacon(source_id, offseted_buf);
            break;
        case RANGING_INIT_MSG:
        case RANGING_RESPONSE_MSG:
            // No longer implemented
            break;
        case RANGING_DS_MSG:
            status = rx_ranging_ds(source_id, offseted_buf, rx_details);
            break;
        default:
            LOG_WRN("Unknown message type");
            break;
        }

        k_free(data.buffer_rx);
    }
}

// PROCESS BEACON TYPE MESSAGE
int rx_beacon(const uint16_t source_id, void *msg)
{
    int status;
    beacon_msg beacon;

    pb_istream_t stream = pb_istream_from_buffer((uint8_t *)msg, beacon_msg_size);
    status = pb_decode(&stream, beacon_msg_fields, &beacon);

    if (not devices_map.contains(source_id))
    {
        struct node_t *node = (struct node_t *)k_calloc(1, sizeof(struct node_t));
        __ASSERT(node != NULL, "Allocating device pointer Failed!");

        node_init(node, source_id);

        devices_map[source_id] = node;
    }
    struct node_t *node = devices_map[source_id];

    node->id = source_id;

    LOG_INF("Received beacon frame from 0x%X", node->id);

    return 0;
}

float ToF_DS(ranging_pkt_t *data)
{
    float Ra, Rb, Da, Db;

    Ra = (float) data->RoundA;
    Da = (float) data->DelayA;
    Rb = (float) data->RoundB;
    Db = (float) data->DelayB;

    float tof = (float)DWT_TIME_UNITS * (Ra * Rb - Da * Db) / (Ra + Da + Rb + Db);

    return tof;
}

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
    write_uwb(data->destination, MAC_COMMAND, RANGING_DS_MSG, (const uint8_t *)data->buffer, sizeof(data->buffer), &data->tx_details);

    k_free(data);
    return;
}

int rx_ranging_ds(const uint16_t source_id, void *msg, struct rx_details_t *queue_data)
{
    int status;
    uint8_t *msg_uint8_t = (uint8_t *)msg;

    LOG_HEXDUMP_DBG(msg, ENCODED_RANGING_PKT_LENGTH, "Received message");

    ranging_pkt_t ranging_pkt;
    decode_ranging_pkt(&ranging_pkt, (const uint8_t *)msg);

    if (not devices_map.contains(source_id))
    {
        struct node_t *node = (struct node_t *)k_calloc(1, sizeof(struct node_t));
        __ASSERT(node != NULL, "Allocating device pointer Failed!");

        node_init(node, source_id);

        devices_map[source_id] = node;
    }
    struct node_t *node = devices_map[source_id];

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

    node->ranging.last_meas_time = k_uptime_get_32();
    node->ranging.error_counter = 0;

    if (ranging_pkt.RoundB == 0 && ranging_pkt.RoundA == 0 && ranging_pkt.DelayB == 0)
    {
        node->ranging.tx_timestamp = 0;
    }
    else
    {
        uint64_t rx_timestamp = queue_data->rx_timestamp;
        uint64_t tx_timestamp = node->ranging.tx_timestamp;

        if (rx_timestamp > tx_timestamp)
            ranging_pkt.RoundA = rx_timestamp - tx_timestamp;
        else
            ranging_pkt.RoundA = rx_timestamp + (0x000000ffffffffffU - tx_timestamp);
    }

    struct tx_details_t tx_details = {
        .ranging = 1,
        .tx_mode = DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED,
        .tx_timestamp = &(node->ranging.tx_timestamp),
        .tx_delay = {.rx_timestamp = queue_data->rx_timestamp, .reserved_time = 1000, .offset = 5}};

    struct scheduled_ranging *work = (struct scheduled_ranging *)k_malloc(sizeof(struct scheduled_ranging));
    __ASSERT(work != NULL, "Ouch! malloc failed :-(");

    uint8_t *write_buffer = &work->buffer[0];

    encode_ranging_pkt(&ranging_pkt, write_buffer);

    work->destination = source_id;
    work->tx_details = tx_details;

    k_work_init_delayable(&work->work, submit_handler);
    status = k_work_schedule(&work->work, K_MSEC(10 * devices_map.size()));
    __ASSERT(status >= 0, "Error scheduling delayed work, status: %d", status);

    if (ranging_pkt.RoundA == 0 || ranging_pkt.DelayA == 0 || ranging_pkt.RoundB == 0 || ranging_pkt.DelayB == 0)
        return 0;

    LOG_DBG("Ra: %d Da: %d Rb: %d Db: %d", ranging_pkt.RoundA, ranging_pkt.DelayA, ranging_pkt.RoundB, ranging_pkt.DelayB);

    float tof = ToF_DS(&ranging_pkt);
    float dist = SPEED_OF_LIGHT * tof;

    if (abs(dist) > 1000)
    {
        return 0;
    }

    struct statistics_t *stats = &node->ranging.stats;
    stats_update(stats, dist);

    struct uwb_msg_t uwb_msg;

    uwb_msg.msg_type = RANGING_DATA;
    uwb_msg.data.ranging_msg.source_mac = source_id;
    uwb_msg.data.ranging_msg.range = stats_get_mean(stats);
    uwb_msg.data.ranging_msg.variance = stats_get_variance(stats);

    LOG_INF("Ranging from ID 0x%X %.2f m | %.4f dev TOF: %.2f ns", source_id, stats_get_mean(stats), stats_get_variance(stats), tof*1e9);

    status = k_msgq_put(&uwb_msgq, &uwb_msg, K_FOREVER);
    __ASSERT(status == 0, "Putting message to queue Failed!");

    return 0;
}

// BEACON THREAD
void uwb_beacon_thread(void)
{
    k_tid_t thread = k_current_get();
    k_thread_suspend(thread);

    LOG_INF("Starting beacon thread");

    int status;

    // INIT MESSAGE
    beacon_msg beacon = {.uav_type = UAV_TYPE_DEFAULT,
                         .GPS = {50.4995652542, 13.4499037391}};

    while (1)
    {
        // ALLOCATE MEMORY FOR BUFFER AND QUEUE DATA
        struct tx_details_t tx_details = {
            .ranging = 0,
            .tx_mode = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED,
            .tx_timestamp = NULL};

        uint8_t buffer_tx[beacon_msg_size];

        pb_ostream_t stream = pb_ostream_from_buffer(buffer_tx, beacon_msg_size);
        status = pb_encode(&stream, beacon_msg_fields, &beacon);
        __ASSERT(status == true, "Protocol buffeer encode failed");

        write_uwb(0xffff, DATA, BEACON_MSG, (const uint8_t *)buffer_tx, stream.bytes_written, &tx_details);

        sleep_ms(1000);
    }
}

// RANGING THREAD
void uwb_ranging_thread(void)
{
    k_tid_t thread = k_current_get();
    k_thread_suspend(thread);

    uint32_t empty[] = {0, 0, 0, 0, 0};

    LOG_INF("Ranging thread started");

    while (1)
    {
        for (auto node_ptr : devices_map)
        {
            uint32_t device_id = node_ptr.first;
            struct node_t *node = (struct node_t *)node_ptr.second;

            if ((k_uptime_get_32() - node->ranging.last_meas_time) < 100)
                continue;

            node->ranging.error_counter++;

            if (node->ranging.error_counter > 100)
            {
                node_destroy(node);
                devices_map.erase(node_ptr.first);

                LOG_WRN("Deleting node 0x%X", node_ptr.first);

                continue;
            }

            if (node->ranging.role == RESPONDER)
                continue;

            node->ranging.last_meas_time = k_uptime_get_32();
            node->ranging.tx_timestamp = 0;
            node->ranging.expected_packet_number = 1;

            stats_reset(&node->ranging.stats);
            struct tx_details_t tx_details = {
                .ranging = 1,
                .tx_mode = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED,
                .tx_timestamp = &(node->ranging.tx_timestamp)};

            LOG_INF("Requesting ranging to device 0x%X", device_id);

            write_uwb(device_id, MAC_COMMAND, RANGING_DS_MSG, (uint8_t *)empty, ENCODED_RANGING_PKT_LENGTH, &tx_details);
        }
        sleep_ms(100);
    }
}
