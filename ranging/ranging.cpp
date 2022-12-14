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

#define INTEGRATOR_ALPHA 1
#define DISTANCE_ALPHA 1

typedef std::unordered_map<uint32_t, struct device_t *> devices_map_t;
typedef std::unordered_map<uint16_t, uint64_t *> tx_timestamps_map_t;

tx_timestamps_map_t tx_timestamps_map;

devices_map_t devices_map;
K_SEM_DEFINE(print_ranging_semaphore, 0, 1);

/**
 * @brief Receiving thread, waits for queue
 *
 */
void ranging_thread(void)
{
    LOG_INF("Rx thread started");
    struct rx_queue_t data;

    while (1)
    {
        // wait for data
        read_uwb(&data);

        int source_id = data.mac_data.source_id;
        struct rx_details_t *rx_details = &data.rx_details;
        rx_details->tx_delay = data.mac_data.tx_delay;

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

    // __ASSERT(status == true, "Protocol buffer decode failed");

    if (not devices_map.contains(source_id))
    {
        struct device_t *dev_ptr = (struct device_t *)k_calloc(1, sizeof(struct device_t));
        __ASSERT(dev_ptr != NULL, "Allocating device pointer Failed!");

        devices_map[source_id] = dev_ptr;
        devices_map[source_id]->ranging = {0, 0, 0, 0, 0, 0, 0, 0};
        devices_map[source_id]->ranging.expected_packet_number = 0;

        stats_init(&dev_ptr->ranging.stats, 20);
    }
    struct device_t *device = devices_map[source_id];

    device->id = source_id;
    device->uav_type = beacon.uav_type;
    device->GPS[0] = beacon.GPS[0];
    device->GPS[1] = beacon.GPS[1];

    return 0;
}

float ToF_DS(float Ra, float Da, float Rb, float Db)
{
    float tof = (float)DWT_TIME_UNITS * (Ra * Rb - Da * Db) / (Ra + Da + Rb + Db);

    return tof;
}

struct scheduled_ranging
{
    struct k_work_delayable work;
    uint8_t buffer[1 * sizeof(uint8_t) + 3 * sizeof(uint32_t)];
    uint16_t destination;
    struct tx_details_t tx_details;
};

void submit_handler(struct k_work *item)
{
    struct scheduled_ranging *data = CONTAINER_OF(item, struct scheduled_ranging, work);
    write_uwb(data->destination, MAC_COMMAND, RANGING_DS_MSG, (const uint8_t *)data->buffer,  sizeof(data->buffer), &data->tx_details);

    k_free(data);
    return;
}

int rx_ranging_ds(const uint16_t source_id, void *msg, struct rx_details_t *queue_data)
{
    int status;
    uint8_t *msg_uint8_t = (uint8_t *)msg;

    uint32_t Ra = 0, Da = 0, Rb = 0, Db = queue_data->tx_delay;

    uint8_t packet_number;
    memcpy(&packet_number, msg_uint8_t, sizeof(uint8_t));
    msg_uint8_t += sizeof(uint8_t);
    memcpy(&Rb, msg_uint8_t, sizeof(uint32_t));
    msg_uint8_t += sizeof(uint32_t);
    memcpy(&Ra, msg_uint8_t, sizeof(uint32_t));
    msg_uint8_t += sizeof(uint32_t);
    memcpy(&Da, msg_uint8_t, sizeof(uint32_t));
    msg_uint8_t += sizeof(uint32_t);

    if (not devices_map.count(source_id))
    {
        struct device_t *dev_ptr = (struct device_t *)k_calloc(1, sizeof(struct device_t));
        __ASSERT(dev_ptr != NULL, "Allocating device pointer Failed!");

        stats_init(&dev_ptr->ranging.stats, 20);

        dev_ptr->id = source_id;

        dev_ptr->ranging.last_meas_time = k_uptime_get_32();
        dev_ptr->ranging.expected_packet_number = 0;
        devices_map[source_id] = dev_ptr;
    }

    struct device_t *device = (struct device_t *)devices_map[source_id];

    if (packet_number == 0)
    {
        LOG_INF("New ranging instance!, packet number = %d", packet_number);
    }
    else if (packet_number == device->ranging.expected_packet_number)
    {
        LOG_DBG("Packet number is same as expected");
    }
    else
    {
        LOG_WRN("Wrong packet number, discarding the measurment! %u", packet_number);
        return 0;
    }

    packet_number = (packet_number + 1) / 256 + (packet_number + 1) % 256;
    device->ranging.expected_packet_number = (packet_number + 1) / 256 + (packet_number + 1) % 256;
    LOG_INF("New expected number: %u, current packet number: %u", device->ranging.expected_packet_number, packet_number);

    device->ranging.last_meas_time = k_uptime_get_32();

    if (Rb == 0 && Ra == 0 && Da == 0)
    {
        device->ranging.tx_timestamp = 0;
        device->ranging.rx_timestamp = 0;
    }
    else
    {
        uint64_t rx_timestamp = queue_data->rx_timestamp;
        uint64_t tx_timestamp = device->ranging.tx_timestamp;

        if (rx_timestamp > tx_timestamp)
            Ra = rx_timestamp - tx_timestamp;
        else
            Ra = rx_timestamp + (0x000000ffffffffffU - tx_timestamp);
    }

    struct tx_details_t tx_details = {
        .ranging = 1,
        .tx_mode = DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED,
        .tx_timestamp = &(device->ranging.tx_timestamp),
        .tx_delay = {.rx_timestamp = queue_data->rx_timestamp, .reserved_time = 1000}};

    struct scheduled_ranging *work = (struct scheduled_ranging *)k_malloc(sizeof(struct scheduled_ranging));
    __ASSERT(work != NULL, "Ouch! malloc failed :-(");

    uint8_t *write_buffer = &work->buffer[0];
    memcpy(write_buffer, &packet_number, sizeof(uint8_t));
    write_buffer += sizeof(uint8_t);
    memcpy(write_buffer, &Ra, sizeof(uint32_t));
    write_buffer += sizeof(uint32_t);
    memcpy(write_buffer, &Rb, sizeof(uint32_t));
    write_buffer += sizeof(uint32_t);
    memcpy(write_buffer, &Db, sizeof(uint32_t));
    write_buffer += sizeof(uint32_t);

    work->destination = source_id;
    work->tx_details = tx_details;

    k_work_init_delayable(&work->work, submit_handler);
    status = k_work_schedule(&work->work, K_MSEC(10));
    __ASSERT(status >= 0, "Error scheduling delayed work, status: %d", status);

    if (Ra == 0 || Da == 0 || Rb == 0 || Db == 0)
        return 0;

    float tof = ToF_DS(Ra, Da, Rb, Db);
    float dist = SPEED_OF_LIGHT * tof;

    if (abs(dist) > 1000)
    {
        return 0;
    }

    struct statistics_t *stats = &device->ranging.stats;
    stats_update(stats, dist);

    struct uwb_msg_t uwb_msg;

    uwb_msg.msg_type = RANGING_DATA;
    uwb_msg.data.ranging_msg.source_mac = source_id;
    uwb_msg.data.ranging_msg.range = stats_get_mean(stats);
    uwb_msg.data.ranging_msg.variance = stats_get_variance(stats);

    LOG_INF("Ranging from ID 0x%X %.2f m | %.4f", source_id, stats_get_mean(stats), stats_get_variance(stats));

    status = k_msgq_put(&uwb_msgq, &uwb_msg, K_FOREVER);
    __ASSERT(status == 0, "Putting message to queue Failed!");

    return 0;
}

// BEACON THREAD
void uwb_beacon_thread(void)
{
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
    LOG_INF("Ranging thread started");

    while (1)
    {
        for (auto node : devices_map)
        {
            uint32_t device_id = node.first;
            struct device_t *device = (struct device_t *)node.second;

            uint32_t rand = sys_rand32_get();

            if ((k_uptime_get_32() - device->ranging.last_meas_time) < 100)
            {
                device->ranging.error_counter = 0;
                continue;
            }

            device->ranging.last_meas_time = k_uptime_get_32();
            device->ranging.tx_timestamp = 0;
            device->ranging.error_counter++;
            device->ranging.expected_packet_number = 1;

            __ASSERT(device->ranging.error_counter < 10, "Could not find the device");

            stats_reset(&device->ranging.stats);

            uint32_t empty[4] = {0, 0, 0, 0};

            struct tx_details_t tx_details = {
                .ranging = 1,
                .tx_mode = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED,
                .tx_timestamp = &(device->ranging.tx_timestamp)};

            LOG_INF("Requesting ranging to device 0x%X", device_id);

            write_uwb(device_id, MAC_COMMAND, RANGING_DS_MSG, (uint8_t *)empty, 1 * sizeof(uint8_t) + 3 * sizeof(uint32_t), &tx_details);
        }
        sleep_ms(100 + 20 * (sys_rand32_get() / INT_MAX - 1));
    }
}

// RANGING THREAD
void uwb_ranging_print_thread(void)
{
    LOG_INF("Ranging print thread started");

    return;

    while (devices_map.empty())
        sleep_ms(100);

    while (1)
    {
        printf("\033\143");

        printf("----------RANGING RESULTS----------\n\r");
        printf("-------THIS DEVICE ID 0x%X-------\n\r", DEVICE_ID);

        for (auto node : devices_map)
        {
            struct device_t *device = (struct device_t *)node.second;
            if (device->ranging.new_data)
            {
                printf("\33[32m");
            }
            else
            {
                printf("\33[31m");
            }
            printf(" · Distance to node 0x%X: %.2f m\n\r", device->id, device->ranging.stats.mean);
            printf("\33[39m");

            device->ranging.new_data = false;
        }

        printf("-----------------------------------\n\r");
        k_sem_take(&print_ranging_semaphore, K_MSEC(200));
    }
}
