#include <zephyr/zephyr.h>
#include <sys/__assert.h>
#include <zephyr/random/rand32.h>
#include <string.h>

#include <pb_encode.h>
#include <pb_decode.h>

#include <iostream>
#include <unordered_map>
#include <vector>

#include "ranging.h"
#include "common_macro.h"

#include <math.h>

extern uint16_t DEVICE_ID;
extern uint16_t PAN_ID;

extern uint8_t SEQ_NUM;

extern struct k_fifo tx_fifo;
extern struct k_fifo rx_fifo;

typedef std::unordered_map<uint32_t, struct device_t *> devices_map_t;
devices_map_t devices_map;

int rx_message(struct rx_queue_t *queue_data)
{
    int source_id = queue_data->mac_data.source_id;
    struct rx_details_t *tx_details = &queue_data->rx_details;

    int status = SUCCESS;
    switch (queue_data->mac_data.msg_type)
    {
    case BEACON_MSG:
        status = rx_beacon(source_id, (void *)queue_data->buffer_rx);
        break;
    case RANGING_INIT_MSG:
        status = rx_ranging_init(source_id, (void *)queue_data->buffer_rx, tx_details);
        break;
    case RANGING_RESPONSE_MSG:
        status = rx_ranging_response(source_id, (void *)queue_data->buffer_rx, tx_details);
        break;
    default:
        printf("Unknown message type\n\r");
        break;
    }
    return status;
}

void tx_message(const uint16_t destination_id, const frame_type_t frame_type, const int msg_type, const uint8_t *msg, int len, tx_details_t *tx_details)
{
    // ALLOCATE MEMORY FOR tx BUFFER AND QUEUE DATA
    uint8_t *buffer_tx = (uint8_t *)k_malloc(10 + len + 2);
    struct tx_queue_t *queue_data = (struct tx_queue_t *)k_malloc(sizeof(struct tx_queue_t));

    queue_data->frame_buffer = buffer_tx;

    int frame_length = 0;

    uint16_t frame_ctrl = 0x9840 | frame_type;
    memcpy(buffer_tx, &frame_ctrl, sizeof(uint16_t));
    buffer_tx += sizeof(uint16_t);
    frame_length += sizeof(uint16_t);

    memcpy(buffer_tx, &SEQ_NUM, sizeof(uint8_t));
    buffer_tx += sizeof(uint8_t);
    frame_length += sizeof(uint8_t);
    SEQ_NUM++;
    SEQ_NUM = 0;

    uint16_t pan_id_temp = PAN_ID;
    memcpy(buffer_tx, &pan_id_temp, sizeof(uint16_t));
    buffer_tx += sizeof(uint16_t);
    frame_length += sizeof(uint16_t);

    memcpy(buffer_tx, &destination_id, sizeof(uint16_t));
    buffer_tx += sizeof(uint16_t);
    frame_length += sizeof(uint16_t);

    memcpy(buffer_tx, &DEVICE_ID, sizeof(uint16_t));
    buffer_tx += sizeof(uint16_t);
    frame_length += sizeof(uint16_t);

    memcpy(buffer_tx, &msg_type, sizeof(uint8_t));
    buffer_tx += sizeof(uint8_t);
    frame_length += sizeof(uint8_t);

    memcpy(buffer_tx, msg, len);
    frame_length += len;
    frame_length += 2;

    // INITIALIZE QUEUE DATA
    queue_data->frame_length = frame_length;
    queue_data->tx_details = *tx_details;

    // SEND DATA TO TX QUEUE
    k_fifo_alloc_put(&tx_fifo, queue_data);
}

// PROCESS BEACON TYPE MESSAGE
int rx_beacon(const uint16_t source_id, void *msg)
{
    beacon_msg beacon;

    pb_istream_t stream = pb_istream_from_buffer((uint8_t *)msg, beacon_msg_size);
    pb_decode(&stream, beacon_msg_fields, &beacon);

    if (not devices_map.contains(source_id))
    {
        devices_map[source_id] = (struct device_t *)k_malloc(sizeof(struct device_t));
        devices_map[source_id]->ranging = {0, 0, 0, 0};
    }
    struct device_t *device = devices_map[source_id];

    device->id = source_id;
    device->uav_type = beacon.uav_type;
    device->GPS[0] = beacon.GPS[0];
    device->GPS[1] = beacon.GPS[1];

    return 0;
}

int rx_ranging_init(const uint16_t source_id, void *msg, struct rx_details_t *rx_details)
{
    ranging_response_msg resp;
    uint8_t buffer_tx[ranging_response_msg_size];

    // CALCULATE TX TIMESTAMP
    uint64_t tx_timestamp = rx_details->rx_timestamp + (10000 * UUS_TO_DWT_TIME);
    tx_timestamp &= 0xffffffffffffff00;
    uint64_t delay = tx_timestamp - rx_details->rx_timestamp;

    resp.delay = (uint32_t)delay;

    struct tx_details_t tx_details = {
        .ranging = 1,
        .tx_mode = DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED,
        .tx_delay = (uint32_t)(tx_timestamp >> 8),
        .tx_timestamp = NULL};

    pb_ostream_t stream = pb_ostream_from_buffer(buffer_tx, ranging_response_msg_size);
    pb_encode(&stream, ranging_response_msg_fields, &resp);

    tx_message(source_id, DATA, RANGING_RESPONSE_MSG, (const uint8_t *)buffer_tx, ranging_response_msg_size, &tx_details);

    return 0;
}

int rx_ranging_response(const uint16_t source_id, void *msg, struct rx_details_t *rx_details)
{
    if (not devices_map.count(source_id))
        return -1;

    ranging_response_msg resp;

    pb_istream_t stream = pb_istream_from_buffer((uint8_t *)msg, ranging_response_msg_size);
    pb_decode(&stream, ranging_response_msg_fields, &resp);

    struct device_t *device = (struct device_t *)devices_map[source_id];

    double clockOffsetRatio = rx_details->carrier_integrator * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6);

    // CALCULATE DISTANCE
    uint64_t rx_timestamp = rx_details->rx_timestamp;
    uint64_t tx_timestamp = device->ranging.tx_timestamp;

    long timespan;
    if (rx_timestamp > tx_timestamp)
    {
        timespan = rx_timestamp - tx_timestamp;
    }
    else
    {
        timespan = rx_timestamp + (0x000000ffffffffffU - tx_timestamp);
    }

    double diff = timespan - (1 - clockOffsetRatio) * resp.delay;

    double tof = (diff / 2.) * DWT_TIME_UNITS;
    double dist = tof * SPEED_OF_LIGHT;

    if (abs(dist) > 1000)
        return 0;

    device->ranging.distance = dist * 0.05 + device->ranging.distance * (1 - 0.05);

    return 0;
}

// BEACON THREAD
void uwb_beacon_thread(void)
{
    // SUSPEND THREAD FOR NOW
    k_tid_t thread_id = k_current_get();
    k_thread_suspend(thread_id);

    printf("Starting beacon thread\n\r");

    // INIT MESSAGE

    beacon_msg beacon = {.uav_type = UAV_TYPE_DEFAULT,
                         .GPS = {50.4995652542, 13.4499037391}};

    while (1)
    {
        // ALLOCATE MEMORY FOR BUFFER AND QUEUE DATA
        printf("Sending beacon message!\n\r");

        struct tx_details_t tx_details = {
            .ranging = 0,
            .tx_mode = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED,
            .tx_delay = 0,
            .tx_timestamp = NULL};

        uint8_t buffer_tx[beacon_msg_size];

        pb_ostream_t stream = pb_ostream_from_buffer(buffer_tx, beacon_msg_size);
        pb_encode(&stream, beacon_msg_fields, &beacon);

        tx_message(0xffff, DATA, BEACON_MSG, (const uint8_t *)buffer_tx, stream.bytes_written, &tx_details);

        sleep_ms(10000 + 200 * (1 - sys_rand32_get() / 2147483648));
    }
}

// RANGING THREAD
void uwb_ranging_thread(void)
{
    k_tid_t thread_id = k_current_get();
    k_thread_suspend(thread_id);

    printf("Ranging thread started\n\r");

    while (devices_map.empty())
        sleep_ms(100);

    while (1)
    {
        for (auto node : devices_map)
        {
            struct device_t *device = (struct device_t *)node.second;

            struct tx_details_t tx_details = {
                .ranging = 1,
                .tx_mode = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED,
                .tx_delay = 0,
                .tx_timestamp = &(device->ranging.tx_timestamp)};

            tx_message(device->id, DATA, RANGING_INIT_MSG, NULL, 0, &tx_details);

            sleep_ms(100);
        }
    }
}

// RANGING THREAD
void uwb_ranging_print_thread(void)
{
    k_tid_t thread_id = k_current_get();
    printf("Ranging print thread started\n\r");

    while (devices_map.empty())
        sleep_ms(100);

    while (1)
    {
        // puts( "\033[2J" );
        printf("\033\143");

        printf("----------RANGING RESULTS----------\n\r");
        printf("-------THIS DEVICE ID 0x%X-------\n\r", DEVICE_ID);

        for (auto node : devices_map)
        {
            struct device_t *device = (struct device_t *)node.second;
            printf(" · Distance to node 0x%X: %.2f m\n\r", device->id, device->ranging.distance);
        }

        printf("-----------------------------------\n\r");

        sleep_ms(200);
    }
}
