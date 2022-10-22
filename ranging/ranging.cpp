#include <zephyr/zephyr.h>
#include <sys/__assert.h>
#include <string.h>

#include <pb_encode.h>
#include <pb_decode.h>

#include <iostream>
#include <unordered_map>
#include <vector>

#include "ranging.h"
#include "common_macro.h"

#include <math.h>
#include "mac.h"

#define INTEGRATOR_ALPHA 1
#define DISTANCE_ALPHA 1

extern uint16_t DEVICE_ID;
extern uint16_t PAN_ID;

extern uint8_t SEQ_NUM;

extern struct k_fifo tx_fifo;
extern struct k_fifo rx_fifo;

typedef std::unordered_map<uint32_t, struct device_t *> devices_map_t;
typedef std::unordered_map<uint16_t, uint64_t *> tx_timestamps_map_t;
tx_timestamps_map_t tx_timestamps_map;
devices_map_t devices_map;

K_SEM_DEFINE(print_ranging_semaphore, 0, 1);

int rx_message(struct rx_queue_t *queue_data)
{
    int source_id = queue_data->mac_data.source_id;
    struct rx_details_t *rx_details = &queue_data->rx_details;
    rx_details->tx_delay = queue_data->mac_data.tx_delay;

    int status = SUCCESS;
    switch (queue_data->mac_data.msg_type)
    {
    case BEACON_MSG:
        status = rx_beacon(source_id, (void *)queue_data->buffer_rx);
        break;
    case RANGING_INIT_MSG:
        status = rx_ranging_init(source_id, (void *)queue_data->buffer_rx, rx_details);
        break;
    case RANGING_RESPONSE_MSG:
        status = rx_ranging_response(source_id, (void *)queue_data->buffer_rx, rx_details);
        break;
    case RANGING_DS_MSG:
        status = rx_ranging_ds(source_id, (void *)queue_data->buffer_rx, rx_details);
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
    uint8_t *buffer_tx = (uint8_t *)k_malloc(10 + 4 + len + 2);
    struct tx_queue_t *queue_data = (struct tx_queue_t *)k_malloc(sizeof(struct tx_queue_t));

    struct mac_data_t mac_data = {
        .frame_ctrl = 0x9840 | frame_type,
        .seq_num = SEQ_NUM++,
        .pan_id = PAN_ID,
        .destination_id = destination_id,
        .source_id = DEVICE_ID,
        .msg_type = msg_type,
        .tx_delay = 0,
    };

    int frame_length = encode_MAC(&mac_data, buffer_tx);

    memcpy(buffer_tx + frame_length, msg, len);
    frame_length += len;

    //! reserve memory for CRC
    frame_length += 2;

    // INITIALIZE QUEUE DATA
    queue_data->frame_buffer = buffer_tx;
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
        devices_map[source_id]->ranging = {0, 0, 0, 0, 0, 0};
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
    struct tx_details_t tx_details = {
        .ranging = 1,
        .tx_mode = DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED,
        .tx_timestamp = NULL,
        .tx_delay = {.rx_timestamp = rx_details->rx_timestamp, .reserved_time = 500}};

    tx_message(source_id, DATA, RANGING_RESPONSE_MSG, (const uint8_t *)&(rx_details->carrier_integrator), sizeof(int32_t), &tx_details);
    // tx_message(source_id, DATA, RANGING_RESPONSE_MSG, (const uint8_t *) NULL, 0, &tx_details);

    return 0;
}

int rx_ranging_response(const uint16_t source_id, void *msg, struct rx_details_t *rx_details)
{
    if (not devices_map.count(source_id))
        return -1;

    struct device_t *device = (struct device_t *)devices_map[source_id];

    int32_t responder_integrator;
    memcpy(&responder_integrator, msg, sizeof(int32_t));

    int32_t integrator = (rx_details->carrier_integrator - responder_integrator) >> 1;


    // int32_t integrator = rx_details->carrier_integrator;

    if (device->ranging.counter < 1)
        device->ranging.integrator = integrator;
    else
        device->ranging.integrator = INTEGRATOR_ALPHA * integrator + (1 - INTEGRATOR_ALPHA) * device->ranging.integrator;

    double clockOffsetRatio = device->ranging.integrator * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6);

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

    double diff = timespan - (1 - clockOffsetRatio) * rx_details->tx_delay;

    double tof = (diff / 2.) * DWT_TIME_UNITS;
    double dist = tof * SPEED_OF_LIGHT + 0.09;

    if (device->ranging.counter < 1)
        device->ranging.distance = dist;
    else
        device->ranging.distance = DISTANCE_ALPHA * dist + (1 - DISTANCE_ALPHA) * device->ranging.distance;

    device->ranging.rx_power = rx_details->rx_power;

    printf("%g, %f, %i, %lu, %u\n", device->ranging.distance, rx_details->rx_power, integrator, timespan, rx_details->tx_delay);

    device->ranging.new_data = true;
    k_sem_give(&print_ranging_semaphore);

    device->ranging.counter++;

    return 0;
}

float ToF_DS(float Ra, float Da, float Rb, float Db)
{
    float tof = (float) DWT_TIME_UNITS*(Ra * Rb - Da * Db) / (2 * (Ra + Db));

    return tof;
}

int rx_ranging_ds(const uint16_t source_id, void *msg, struct rx_details_t *queue_data)
{
    uint32_t *msg_uint32_t = (uint32_t *)msg;
    uint32_t buffer_tx[3];

    uint32_t Ra = 0, Da = 0, Rb = 0, Db = queue_data->tx_delay;

    memcpy(&Rb, &(msg_uint32_t[0]), sizeof(uint32_t));
    memcpy(&Ra, &(msg_uint32_t[1]), sizeof(uint32_t));
    memcpy(&Da, &(msg_uint32_t[2]), sizeof(uint32_t));

    if (not tx_timestamps_map.contains(source_id))
    {
        uint64_t *timestamp_ptr = (uint64_t *)k_malloc(sizeof(uint64_t));
        *timestamp_ptr = 0;

        tx_timestamps_map[source_id] = timestamp_ptr;
    }
    else
    {
        uint64_t rx_timestamp = queue_data->rx_timestamp;
        uint64_t tx_timestamp = *(tx_timestamps_map[source_id]);

        if (rx_timestamp > tx_timestamp)
            Ra = rx_timestamp - tx_timestamp;
        else
            Ra = rx_timestamp + (0x000000ffffffffffU - tx_timestamp);
    }


    struct tx_details_t tx_details = {
        .ranging = 1,
        .tx_mode = DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED,
        .tx_timestamp = tx_timestamps_map[source_id],
        .tx_delay = {.rx_timestamp = queue_data->rx_timestamp, .reserved_time = 500}};

    memcpy(&(buffer_tx[0]), &Ra, sizeof(uint32_t));
    memcpy(&(buffer_tx[1]), &Rb, sizeof(uint32_t));
    memcpy(&(buffer_tx[2]), &Db, sizeof(uint32_t));

    tx_message(source_id, DATA, RANGING_DS_MSG, (const uint8_t *)buffer_tx, 3 * sizeof(uint32_t), &tx_details);

    if (Ra != 0 && Da != 0 && Rb != 0 && Db != 0)
    {
        float tof = ToF_DS(Ra, Da, Rb, Db);
        float dist = SPEED_OF_LIGHT * tof;
        printf("Time: %u ms ", k_uptime_get_32());
        printf("Dist %.2f\n\r", dist);
    }
    else
    {
        printf("Some numbers are set to 0\n\r");
    }

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
        // printf("Sending beacon message!\n\r");

        struct tx_details_t tx_details = {
            .ranging = 0,
            .tx_mode = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED,
            .tx_timestamp = NULL};

        uint8_t buffer_tx[beacon_msg_size];

        pb_ostream_t stream = pb_ostream_from_buffer(buffer_tx, beacon_msg_size);
        pb_encode(&stream, beacon_msg_fields, &beacon);

        tx_message(0xffff, DATA, BEACON_MSG, (const uint8_t *)buffer_tx, stream.bytes_written, &tx_details);

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
        sleep_ms(100);
    printf("\033\143");
    while (1)
    {
        for (auto node : devices_map)
        {
            struct device_t *device = (struct device_t *)node.second;

            if (not tx_timestamps_map.contains(device->id))
            {
                uint64_t *timestamp_ptr = (uint64_t *)k_malloc(sizeof(uint64_t));
                *timestamp_ptr = 0;

                tx_timestamps_map[device->id] = timestamp_ptr;
            }
            else
            {
                continue;
            }

            uint32_t empty[3] = {0,0,0};

            struct tx_details_t tx_details = {
                .ranging = 1,
                .tx_mode = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED,
                .tx_timestamp = tx_timestamps_map[device->id]};

            tx_message(device->id, DATA, RANGING_DS_MSG, (uint8_t*) empty, 3*sizeof(uint32_t), &tx_details);

        sleep_ms(100);
        }
    }
}

// RANGING THREAD
void uwb_ranging_print_thread(void)
{
    k_tid_t thread_id = k_current_get();
    printf("Ranging print thread started\n\r");

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
            printf(" · Distance to node 0x%X: %.2f m | %.2f dBm\n\r", device->id, device->ranging.distance, device->ranging.rx_power);
            printf("\33[39m");

            device->ranging.new_data = false;
        }

        printf("-----------------------------------\n\r");
        k_sem_take(&print_ranging_semaphore, K_MSEC(200));
    }
}
