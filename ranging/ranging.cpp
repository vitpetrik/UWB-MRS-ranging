#include <zephyr/zephyr.h>
#include <string.h>

#include <iostream>
#include <unordered_map>
#include <vector>

#include "ranging.h"
#include "common_macro.h"

extern uint32_t DEVICE_ID;

extern struct k_fifo tx_fifo;
extern struct k_fifo rx_fifo;

typedef std::unordered_map<uint32_t, struct device_t *> devices_map_t;
devices_map_t devices_map;

// PROCESS BEACON TYPE MESSAGE
int process_beacon(beacon_msg *beacon)
{
    if (not devices_map.contains(beacon->id))
    {
        devices_map[beacon->id] = (struct device_t *)k_malloc(sizeof(struct device_t));
        devices_map[beacon->id]->ranging = {0, 0, 0, 0};
    }
    struct device_t *device = devices_map[beacon->id];

    printf("Received ID: 0x%X at GPS coordinates N%f E%f\n\r",
           beacon->id,
           beacon->GPS[0],
           beacon->GPS[1]);

    device->id = beacon->id;
    device->uav_type = beacon->uav_type;
    device->GPS[0] = beacon->GPS[0];
    device->GPS[1] = beacon->GPS[1];

    return RX_ENABLE;
}

// BEACON THREAD
void uwb_beacon_thread(void)
{
    // SUSPEND THREAD FOR NOW
    k_tid_t thread_id = k_current_get();
    k_thread_suspend(thread_id);

    printk("Starting beacon thread\n\r");

    // INIT MESSAGE
    data_msg data_tx = data_msg_init_default;

    data_tx.which_data = data_msg_beacon_tag;
    data_tx.data.beacon.id = DEVICE_ID;
    data_tx.data.beacon.uav_type = UAV_TYPE_DEFAULT;
    data_tx.data.beacon.GPS[0] = 50.4995652542;
    data_tx.data.beacon.GPS[1] = 13.4499037391;

    while (1)
    {
        printk("Beacon!\n\r");

        // ALLOCATE MEMORY FOR BUFFER AND QUEUE DATA
        uint8_t *buffer_tx = (uint8_t *)k_malloc(data_msg_size);
        struct tx_queue_t *queue_data = (struct tx_queue_t *)k_malloc(sizeof(struct tx_queue_t));

        // ENCODE THE MESSAGE
        pb_ostream_t stream = pb_ostream_from_buffer(buffer_tx, data_msg_size);
        pb_encode(&stream, data_msg_fields, &data_tx);
        int msg_length = stream.bytes_written;

        // SET QUEUE DATA
        queue_data->frame_length = msg_length;
        queue_data->frame_buffer = buffer_tx;

        queue_data->ranging = 0;
        queue_data->tx_delay = 0;
        queue_data->tx_mode = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED;

        queue_data->tx_timestamp = NULL;

        // SEND DATA TO QUEUE
        k_fifo_put(&tx_fifo, queue_data);

        sleep_ms(10000);
    }
}

// PROCESS RANGING TYPE OF MESSAGE
int process_ranging(ranging_msg *ranging, struct rx_queue_t *rx_metadata)
{
    int status;

    switch (ranging->which_data)
    {
    case ranging_msg_ranging_init_tag:
    {
        // RECIEVED INITIALIZER -> TRANSIM RESPONSE
        ranging_init_msg *ranging_init = &(ranging->data.ranging_init);
        if (ranging_init->to_id != DEVICE_ID)
        {
            status = RX_ENABLE;
            break;
        }

        // ALLOCATE MEMORY FOR tx BUFFER AND QUEUE DATA
        uint8_t *buffer_tx = (uint8_t *)k_malloc(data_msg_size);
        struct tx_queue_t *queue_data = (struct tx_queue_t *)k_malloc(sizeof(struct tx_queue_t));

        // CREATE RESPONSE DATA
        data_msg data_tx = data_msg_init_default;
        data_tx.which_data = data_msg_ranging_tag;
        data_tx.data.ranging.which_data = ranging_msg_ranging_response_tag;

        data_tx.data.ranging.data.ranging_response.from_id = DEVICE_ID;
        data_tx.data.ranging.data.ranging_response.to_id = ranging_init->from_id;

        // CALCULATE TX TIMESTAMP
        uint64_t tx_timestamp = rx_metadata->rx_timestamp + (2000 * UUS_TO_DWT_TIME);
        tx_timestamp &= 0xffffffffffffff00;
        uint64_t delay = tx_timestamp - rx_metadata->rx_timestamp;

        data_tx.data.ranging.data.ranging_response.delay = delay;

        // ENCODE THE MESSAGE
        pb_ostream_t stream = pb_ostream_from_buffer(buffer_tx, data_msg_size);
        pb_encode(&stream, data_msg_fields, &data_tx);
        int msg_length = stream.bytes_written;

        // INITIALIZE QUEUE DATA
        queue_data->frame_length = msg_length;
        queue_data->frame_buffer = buffer_tx;

        queue_data->ranging = 1;
        queue_data->tx_delay = (uint32_t)(tx_timestamp >> 8);
        queue_data->tx_mode = DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED;

        queue_data->tx_timestamp = NULL;

        // SEND DATA TO TX QUEUE
        k_fifo_put(&tx_fifo, queue_data);
        status = SUCCESS;
        break;
    }
    case ranging_msg_ranging_response_tag:
    {
        ranging_response_msg *ranging_response = &(ranging->data.ranging_response);
        if (ranging_response->to_id != DEVICE_ID or not devices_map.count(ranging_response->from_id))
        {
            status = RX_ENABLE;
            break;
        }

        struct device_t *device = (struct device_t *)devices_map[ranging_response->from_id];

        double clockOffsetRatio = rx_metadata->carrier_integrator * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6);

        // CALCULATE DISTANCE
        uint64_t rx_timestamp = rx_metadata->rx_timestamp;
        uint64_t tx_timestamp = device->ranging.tx_timestamp;

        uint64_t timespan = rx_timestamp - tx_timestamp;
        double diff = timespan - (1 - clockOffsetRatio) * ranging_response->delay;

        double tof = (diff / 2.) * DWT_TIME_UNITS;
        double dist = tof * SPEED_OF_LIGHT;

        device->ranging.distance = dist * 0.05 + device->ranging.distance * (1 - 0.05);
        printf("Distance to id 0x%X: %1.2g\n\r", ranging_response->from_id, device->ranging.distance);

        status = RX_ENABLE;
        break;
    }
    default:
        printk("Unknown message type\n\r");
    }

    return status;
}

// RANGING THREAD
void uwb_ranging_thread(void)
{
    k_tid_t thread_id = k_current_get();
    k_thread_suspend(thread_id);

    printk("Ranging thread started\n\r");

    while (devices_map.empty())
        sleep_ms(1000);

    while (1)
    {
        for (auto node : devices_map)
        {
            struct device_t *device = (struct device_t *)node.second;

            // ALLOCATE MEMORY FOR BUFFER AND QUEUE DATA
            uint8_t *buffer_tx = (uint8_t *)k_malloc(data_msg_size);
            struct tx_queue_t *queue_data = (struct tx_queue_t *)k_malloc(sizeof(struct tx_queue_t));

            // SET MESSAGE DATA
            data_msg data_tx = data_msg_init_default;

            data_tx.which_data = data_msg_ranging_tag;
            data_tx.data.ranging.which_data = ranging_msg_ranging_init_tag;
            data_tx.data.ranging.data.ranging_init.from_id = DEVICE_ID;
            data_tx.data.ranging.data.ranging_init.to_id = device->id;

            // ENCODE THE DATA TO BUFFER
            pb_ostream_t stream = pb_ostream_from_buffer(buffer_tx, data_msg_size);
            pb_encode(&stream, data_msg_fields, &data_tx);
            int msg_length = stream.bytes_written;

            // SET QUEUE DATA
            queue_data->frame_length = msg_length;
            queue_data->frame_buffer = buffer_tx;

            queue_data->ranging = 1;
            queue_data->tx_delay = 0;
            queue_data->tx_mode = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED;

            queue_data->tx_timestamp = &(device->ranging.tx_timestamp);

            // SEND DATA TO QUEUE
            k_fifo_put(&tx_fifo, queue_data);

            sleep_ms(500);
        }
    }
}
