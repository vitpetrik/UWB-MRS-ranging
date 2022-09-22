#include <zephyr/zephyr.h>
#include <string.h>

#include <iostream>
#include <unordered_map>
#include <vector>

#include "ranging.h"
#include "common_macro.h"

extern uint16_t DEVICE_ID;
extern uint8_t SEQ_NUM;

extern struct k_fifo tx_fifo;
extern struct k_fifo rx_fifo;

typedef std::unordered_map<uint32_t, struct device_t *> devices_map_t;
devices_map_t devices_map;

int rx_message(const uint32_t msg_type, const uint16_t source_id, const void *msg, struct rx_queue_t *queue_data)
{
    int status = SUCCESS;
    switch (msg_type)
    {
    case data_msg_beacon_tag:
        status = rx_beacon(source_id, (beacon_msg *)msg);
        break;
    case data_msg_ranging_init_tag:
        status = rx_ranging_init(source_id, (ranging_init_msg *)msg, queue_data);
        break;
    case data_msg_ranging_response_tag:
        status = rx_ranging_response(source_id, (ranging_response_msg *)msg, queue_data);
        break;
    default:
        printf("Unknown message type\n\r");
        break;
    }
    return status;
}

void tx_message(const uint16_t destination_id, frame_type_t frame_type, const data_msg *msg, tx_details_t *tx_details)
{
    // ALLOCATE MEMORY FOR tx BUFFER AND QUEUE DATA
    uint8_t *buffer_tx = (uint8_t *)k_malloc(9 + data_msg_size);
    struct tx_queue_t *queue_data = (struct tx_queue_t *)k_malloc(sizeof(struct tx_queue_t));

    queue_data->frame_buffer = buffer_tx;

    uint16_t frame_ctrl = 0x8840 | frame_type;
    memcpy(buffer_tx, &frame_ctrl, sizeof(uint16_t));
    buffer_tx += sizeof(uint16_t);

    memcpy(buffer_tx, &SEQ_NUM, sizeof(uint8_t));
    buffer_tx += sizeof(uint8_t);
    SEQ_NUM++;

    uint16_t pan_id = 0xffff;
    memcpy(buffer_tx, &pan_id, sizeof(uint16_t));
    buffer_tx += sizeof(uint16_t);

    memcpy(buffer_tx, &destination_id, sizeof(uint16_t));
    buffer_tx += sizeof(uint16_t);

    memcpy(buffer_tx, &DEVICE_ID, sizeof(uint16_t));
    buffer_tx += sizeof(uint16_t);

    // ENCODE THE MESSAGE
    pb_ostream_t stream = pb_ostream_from_buffer(buffer_tx, data_msg_size);
    pb_encode(&stream, data_msg_fields, msg);
    int msg_length = stream.bytes_written + 9;

    // INITIALIZE QUEUE DATA
    queue_data->frame_length = msg_length;
    queue_data->tx_details = *tx_details;

    // SEND DATA TO TX QUEUE
    k_fifo_put(&tx_fifo, queue_data);
}

// PROCESS BEACON TYPE MESSAGE
int rx_beacon(const uint16_t source_id, beacon_msg *beacon)
{
    if (not devices_map.contains(source_id))
    {
        devices_map[source_id] = (struct device_t *)k_malloc(sizeof(struct device_t));
        devices_map[source_id]->ranging = {0, 0, 0, 0};
    }
    struct device_t *device = devices_map[source_id];

    printf("Received ID: 0x%X at GPS coordinates N%f E%f\n\r",
           source_id,
           beacon->GPS[0],
           beacon->GPS[1]);

    device->id = source_id;
    device->uav_type = beacon->uav_type;
    device->GPS[0] = beacon->GPS[0];
    device->GPS[1] = beacon->GPS[1];

    return RX_ENABLE;
}

int rx_ranging_init(const uint16_t source_id, ranging_init_msg *msg, struct rx_queue_t *rx_metadata)
{
    ranging_response_msg data_tx;

    // CALCULATE TX TIMESTAMP
    uint64_t tx_timestamp = rx_metadata->rx_timestamp + (2000 * UUS_TO_DWT_TIME);
    tx_timestamp &= 0xffffffffffffff00;
    uint64_t delay = tx_timestamp - rx_metadata->rx_timestamp;

    data_tx.delay = (uint32_t)delay;

    data_msg new_msg = {.which_data = data_msg_ranging_response_tag};
    new_msg.data.ranging_response = data_tx;

    struct tx_details_t tx_details = {
        .ranging = 1,
        .tx_mode = DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED,
        .tx_delay = (uint32_t)(tx_timestamp >> 8),
        .tx_timestamp = NULL};

    tx_message(source_id, DATA, &new_msg, &tx_details);

    return SUCCESS;
}

int rx_ranging_response(const uint16_t source_id, ranging_response_msg *msg, struct rx_queue_t *rx_metadata)
{
    if (not devices_map.count(source_id))
        return -1;

    struct device_t *device = (struct device_t *)devices_map[source_id];

    double clockOffsetRatio = rx_metadata->carrier_integrator * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6);

    // CALCULATE DISTANCE
    uint64_t rx_timestamp = rx_metadata->rx_timestamp;
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

    double diff = timespan - (1 - clockOffsetRatio) * msg->delay;

    double tof = (diff / 2.) * DWT_TIME_UNITS;
    double dist = tof * SPEED_OF_LIGHT;

    device->ranging.distance = dist * 0.05 + device->ranging.distance * (1 - 0.05);
    printf("Distance to id 0x%X: %1.2g\n\r", source_id, device->ranging.distance);

    return RX_ENABLE;
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
    data_msg msg = {.which_data = data_msg_beacon_tag}; 
    msg.data.beacon = beacon;

    while (1)
    {
        // ALLOCATE MEMORY FOR BUFFER AND QUEUE DATA
        struct tx_details_t tx_details = {
            .ranging = 0,
            .tx_mode = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED,
            .tx_delay = 0,
            .tx_timestamp = NULL};

        tx_message(0, MRS_BEACON, &msg, &tx_details);

        sleep_ms(10000);
    }
}

// RANGING THREAD
void uwb_ranging_thread(void)
{
    k_tid_t thread_id = k_current_get();
    k_thread_suspend(thread_id);

    printf("Ranging thread started\n\r");

    while (devices_map.empty())
        sleep_ms(1000);

    while (1)
    {
        for (auto node : devices_map)
        {
            struct device_t *device = (struct device_t *)node.second;

            // SET MESSAGE DATA
            data_msg msg = {.which_data = data_msg_ranging_init_tag};

            struct tx_details_t tx_details = {
                .ranging = 1,
                .tx_mode = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED,
                .tx_delay = 0,
                .tx_timestamp = &(device->ranging.tx_timestamp)};

            tx_message(device->id, DATA, &msg, &tx_details);

            sleep_ms(50);
        }
    }
}
