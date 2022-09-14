/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <string.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "platform.h"

#include <pb_encode.h>
#include <pb_decode.h>
#include "data_msg.pb.h"

#include <search.h>

#define sleep_ms(ms) k_sleep(K_MSEC(ms))

/* Example application name and version to display on LCD screen. */
#define APP_NAME "MRS UWB v0.1"
const char *buildString = "Build was compiled at " __DATE__ ", " __TIME__ ".";

uint32_t device_id;

/* Default communication configuration. We use here EVK1000's mode 4. See NOTE 1 below. */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (128 + 8 + 1)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 �s and 1 �s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 64267

#define SPEED_OF_LIGHT 299702547.0

// GLOBAL VARIABLES

struct k_mutex dwt_mutex;
struct k_mutex devices_map_mutex;

void *devices_map;

// DATA STRUCTURE DEFINITIONS

struct ranging_t
{
    double distance;
    double delay;
    uint64_t tx_timestamp;
    uint64_t rx_timestamp;
};

struct device_t
{
    uint32_t id;
    UAV_TYPE uav_type;
    double GPS[2];
    struct ranging_t ranging;
};

struct rx_queue_t
{
    dwt_cb_data_t *cb_data;
    uint8_t *buffer_rx;
    uint64_t rx_timestamp;
    int32_t carrier_integrator;
};

struct tx_queue_t
{
    uint16_t frame_length;
    uint8_t *frame_buffer;
    int ranging;
    uint8_t tx_mode;
    uint32_t tx_delay;
    uint64_t *tx_timestamp;
};

// FUNCTIONS
void process_beacon(beacon_msg *beacon);
void process_ranging(ranging_msg *ranging, struct rx_queue_t *queue_data);

static uint64_t get_rx_timestamp_u64(void);
static uint64_t get_tx_timestamp_u64(void);

// THREADS
void uwb_beacon_thread(void);
void uwb_ranging_thread(void);

void uwb_tx_thread(void);
void uwb_rx_thread(void);

void dwt_isr_thread(void);

K_THREAD_DEFINE(uwb_beacon_thr, 2048, uwb_beacon_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(uwb_ranging_thr, 2048, uwb_ranging_thread, NULL, NULL, NULL, 5, 0, 0);

K_THREAD_DEFINE(uwb_rx_thr, 2048, uwb_rx_thread, NULL, NULL, NULL, -2, 0, 0);
K_THREAD_DEFINE(uwb_tx_thr, 2048, uwb_tx_thread, NULL, NULL, NULL, -2, 0, 0);

K_THREAD_DEFINE(dwt_isr_thr, 2048, dwt_isr_thread, NULL, NULL, NULL, -1, 0, 0);

// CALLBACKS
void uwb_txdone(const dwt_cb_data_t *data);
void uwb_rxto(const dwt_cb_data_t *data);
void uwb_rxerr(const dwt_cb_data_t *data);
void uwb_rxok(const dwt_cb_data_t *data);

void dwm_int_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

// QUEUES
K_QUEUE_DEFINE(rx_queue);
K_QUEUE_DEFINE(tx_queue);

void main(void)
{
    // INITIALIZE

    printk("%s\n\r", APP_NAME);
    printk("%s\n\r", buildString);

    device_id = NRF_FICR->DEVICEID[0];

    printk("Device ID: 0x%X\n\r", device_id);

    k_mutex_init(&devices_map_mutex);
    devices_map = um_create();

    k_queue_init(&rx_queue);
    k_queue_init(&tx_queue);

    // SETUP DW1000

    k_mutex_init(&dwt_mutex);
    k_mutex_lock(&dwt_mutex, K_FOREVER);

    dwt_hardreset();
    dwt_hardinterrupt(dwm_int_callback);

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        printk("INIT FAILED");
        while (1)
        {
        };
    }
    spi_set_rate_high();

    dwt_configure(&config);

    uint32_t ant_delay;
    dwt_otpread(0x01c, &ant_delay, 1);

    dwt_setrxantennadelay(ant_delay);
    dwt_settxantennadelay(ant_delay);

    dwt_setcallbacks(uwb_txdone, uwb_rxok, uwb_rxto, uwb_rxerr);

    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG, 1);
    k_mutex_unlock(&dwt_mutex);

    printk("Settings done!\n\r");

    // START THE THREADS

    k_thread_resume(uwb_rx_thr);
    sleep_ms(100);
    k_thread_resume(uwb_tx_thr);
    sleep_ms(100);
    k_thread_resume(uwb_beacon_thr);
    sleep_ms(100);

    return;
}

// GET RX TIMESTAMP IN 40-BIT FORMAT
static uint64_t get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts = (ts << 8) + ts_tab[i];
    }
    ts &= (uint64_t)0x000000ffffffffff;
    return ts;
}

// GET TX TIMESTAMP IN 40-BIT FORMAT
static uint64_t get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts = (ts << 8) + ts_tab[i];
    }
    ts &= (uint64_t)0x000000ffffffffff;
    return ts;
}

// PROCESS BEACON TYPE MESSAGE
void process_beacon(beacon_msg *beacon)
{
    printf("Received ID: 0x%X at GPS coordinates N%f E%f\n\r",
           beacon->id,
           beacon->GPS[0],
           beacon->GPS[1]);

    // FIND DEVICE WITH THE ID IN MAP
    struct device_t *device = um_find(devices_map);

    // IF NOT FOUND, CREATE ONE
    if (device == NULL)
    {
        device = k_malloc(sizeof(struct device_t));
        um_add(devices_map, beacon->id, device);
    }

    // INITIALIZE THE STRUCTURE
    device->id = beacon->id;
    device->uav_type = beacon->uav_type;
    device->GPS[0] = beacon->GPS[0];
    device->GPS[1] = beacon->GPS[1];

    // IF THERE IS AT LEAST ONE DEVICE, RANGING THREAD CAN START
    k_thread_resume(uwb_ranging_thr);
}

// PROCESS RANGING TYPE OF MESSAGE
void process_ranging(ranging_msg *ranging, struct rx_queue_t *rx_metadata)
{
    switch (ranging->which_data)
    {
    case ranging_msg_ranging_init_tag:
    {
        // RECIEVED INITIALIZER -> TRANSIM RESPONSE
        ranging_init_msg *ranging_init = &(ranging->data.ranging_init);
        if (ranging_init->to_id != device_id)
            break;

        // CALCULATE TX TIMESTAMP
        uint64_t tx_timestamp = rx_metadata->rx_timestamp + (12000 * UUS_TO_DWT_TIME);
        tx_timestamp &= 0xffffffffffffff00;
        uint64_t delay = tx_timestamp - rx_metadata->rx_timestamp;

        // CREATE RESPONSE DATA
        data_msg data_tx = data_msg_init_default;
        data_tx.which_data = data_msg_ranging_tag;
        data_tx.data.ranging.which_data = ranging_msg_ranging_response_tag;

        data_tx.data.ranging.data.ranging_response.from_id = device_id;
        data_tx.data.ranging.data.ranging_response.to_id = ranging_init->from_id;
        data_tx.data.ranging.data.ranging_response.delay = delay;

        // ALLOCATE MEMORY FOR tx BUFFER AND QUEUE DATA
        uint8_t *buffer_tx = k_malloc(data_msg_size);
        struct tx_queue_t *queue_data = k_malloc(sizeof(struct tx_queue_t));

        // ENCODE THE MESSAGE
        pb_ostream_t stream = pb_ostream_from_buffer(buffer_tx, data_msg_size);
        pb_encode(&stream, data_msg_fields, &data_tx);
        int msg_length = stream.bytes_written;

        // INITIALIZE QUEUE DATA
        queue_data->frame_length = msg_length;
        queue_data->frame_buffer = buffer_tx;

        queue_data->ranging = 1;
        queue_data->tx_delay = (uint32_t) (tx_timestamp >> 8);
        queue_data->tx_mode = DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED;

        queue_data->tx_timestamp = NULL;

        // SEND DATA TO TX QUEUE
        k_queue_append(&tx_queue, queue_data);
        k_yield();

        break;
    }
    case ranging_msg_ranging_response_tag:
    {
        // RESPONSE MESSAGE RECEIVED
        ranging_response_msg *ranging_response = &(ranging->data.ranging_response);
        if (ranging_response->to_id != device_id)
            break;

        // RETRIEVE DATA FROM MAP
        struct device_t *device = (struct device_t *)um_find(devices_map, ranging_response->from_id);
        if (device == NULL)
            break;

        double clockOffsetRatio = rx_metadata->carrier_integrator * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6);

        // CALCULATE DISTANCE
        uint64_t rx_timestamp = rx_metadata->rx_timestamp;
        uint64_t tx_timestamp = device->ranging.tx_timestamp;

        uint64_t timespan = rx_timestamp - tx_timestamp;
        double diff = timespan - (1 - clockOffsetRatio) * ranging_response->delay;

        double tof = ((diff / 2.) * DWT_TIME_UNITS)-1.28156358e-7;
        double dist = tof * SPEED_OF_LIGHT;
        printf("tod: %g\n\r", tof);
        printf("distance: %g\n\r", dist);
        break;
    }
    default:
        printk("Unknown message type\n\r");
    }
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
    data_tx.data.beacon.id = device_id;
    data_tx.data.beacon.uav_type = UAV_TYPE_DEFAULT;
    data_tx.data.beacon.GPS[0] = 50.4995652542;
    data_tx.data.beacon.GPS[1] = 13.4499037391;

    while (1)
    {
        printk("Beacon!\n\r");

        // ALLOCATE MEMORY FOR BUFFER AND QUEUE DATA
        uint8_t *buffer_tx = k_malloc(data_msg_size);
        struct tx_queue_t *queue_data = k_malloc(sizeof(struct tx_queue_t));

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
        k_queue_append(&tx_queue, queue_data);

        sleep_ms(10000);
    }
}

// RANGING THREAD
void uwb_ranging_thread(void)
{
    k_tid_t thread_id = k_current_get();
    k_thread_suspend(thread_id);

    printk("Ranging thread started\n\r");

    while (1)
    {
        int size = um_size(devices_map);

        uint32_t keys[size];
        um_get_keys(devices_map, keys);

        // CYCLE THROUGH ALL THE DEVICES IN MAP
        for (int dev = 0; dev < size; dev++)
        {
            struct device_t *device = (struct device_t *)um_find(devices_map, keys[dev]);
            if (device == NULL)
                continue;

            // ALLOCATE MEMORY FOR BUFFER AND QUEUE DATA
            uint8_t *buffer_tx = k_malloc(data_msg_size);
            struct tx_queue_t *queue_data = k_malloc(sizeof(struct tx_queue_t));

            // SET MESSAGE DATA
            data_msg data_tx = data_msg_init_default;

            data_tx.which_data = data_msg_ranging_tag;
            data_tx.data.ranging.which_data = ranging_msg_ranging_init_tag;
            data_tx.data.ranging.data.ranging_init.from_id = device_id;
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
            k_queue_append(&tx_queue, queue_data);

            sleep_ms(2000);
        }
    }
}

// RECIEVE THREAD
void uwb_rx_thread(void)
{
    k_tid_t thread_id = k_current_get();
    k_thread_suspend(thread_id);

    k_mutex_lock(&dwt_mutex, K_FOREVER);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    k_mutex_unlock(&dwt_mutex);

    printk("Rx thread started\n\r");

    while (1)
    {
        // RECEIVE DATA FROM INTERRUPT THROUGH QUEUE
        struct rx_queue_t *data = (struct rx_queue_t *)k_queue_get(&rx_queue, K_FOREVER);
        data_msg data_rx;

        // DECODE THE DATA
        pb_istream_t stream = pb_istream_from_buffer(data->buffer_rx, data->cb_data->datalength);
        pb_decode(&stream, data_msg_fields, &data_rx);

        // PROCESS THE MESSAGE
        switch (data_rx.which_data)
        {
        case data_msg_beacon_tag:
            process_beacon(&data_rx.data.beacon);
            break;
        case data_msg_ranging_tag:
            process_ranging(&data_rx.data.ranging, data);
            break;
        default:
            printk("Unknown message\n\r");
            break;
        }

        // FREE QUEUE DATA
        k_free(data->buffer_rx);
        k_free(data);
    }
}

K_CONDVAR_DEFINE(tx_condvar);

// TRANSMIT THREAD
void uwb_tx_thread(void)
{
    k_tid_t thread_id = k_current_get();
    k_thread_suspend(thread_id);

    printk("Tx thread started\n\r");
    k_condvar_init(&tx_condvar);

    while (1)
    {
        // GET DATA FOR TRANSMITTING THROUGH QUEUE
        struct tx_queue_t *data_tx = (struct tx_queue_t *)k_queue_get(&tx_queue, K_FOREVER);
        k_mutex_lock(&dwt_mutex, K_FOREVER);

        // TURN OFF RECEIVER
        dwt_forcetrxoff();

        // WRITE DATA TO TX BUFFER
        dwt_writetxdata(data_tx->frame_length, data_tx->frame_buffer, 0);
        dwt_writetxfctrl(data_tx->frame_length, 0, data_tx->ranging);

        // SET DELAYED TX TIMESTAMP
        if (data_tx->tx_mode & DWT_START_TX_DELAYED)
        {
            dwt_setdelayedtrxtime(data_tx->tx_delay);
        }

        // START TRANSMITTING
        int status = dwt_starttx(data_tx->tx_mode);

        if (status == DWT_SUCCESS)
        {
            // SLEEP UNTIL TIME OR WAKE UP BY CALL FROM ISR
            int status = k_condvar_wait(&tx_condvar, &dwt_mutex, K_MSEC(100));

            if (status != EAGAIN)
            {
                uint64_t *timestamp_ptr = data_tx->tx_timestamp;
                if (timestamp_ptr != NULL)
                {
                    uint64_t timestamp = get_tx_timestamp_u64();
                    *timestamp_ptr = timestamp;
                }
            }
            else
            {
                dwt_forcetrxoff();
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
            }
        }
        else
        {
            dwt_forcetrxoff();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        // UNLOCK THE MTX AND FREE THE DATA
        k_mutex_unlock(&dwt_mutex);
        k_free(data_tx->frame_buffer);
        k_free(data_tx);
    }
}

// CALLBACK FOR TRANSMITT DONE EVENT
void uwb_txdone(const dwt_cb_data_t *data)
{
    k_condvar_signal(&tx_condvar);
}
// RX TIMEOUT
void uwb_rxto(const dwt_cb_data_t *data)
{
}
// RX ERROR
void uwb_rxerr(const dwt_cb_data_t *data)
{
}
// RECEIVE OK
void uwb_rxok(const dwt_cb_data_t *data)
{
    // INIT DATA
    uint16_t msg_length = data->datalength;
    uint8_t *buffer_rx = k_malloc(msg_length);
    struct rx_queue_t *queue = k_malloc(sizeof(struct rx_queue_t));

    // READ THE DATA AND ENABLE RX
    dwt_readrxdata(buffer_rx, msg_length, 0);
    uint64_t rx_ts = get_rx_timestamp_u64();
    int32_t integrator = dwt_readcarrierintegrator();

    // INIT QUEUE DATA AND SEND TO RX THREAD THROUGH QUEUE
    queue->cb_data = data;
    queue->buffer_rx = buffer_rx;
    queue->rx_timestamp = rx_ts;
    queue->carrier_integrator = integrator;

    k_queue_append(&rx_queue, queue);
}

void dwt_isr_thread(void)
{
    while (1)
    {
        k_sleep(K_FOREVER);
        k_mutex_lock(&dwt_mutex, K_FOREVER);
        dwt_isr();
        k_mutex_unlock(&dwt_mutex);
    }
}

void dwm_int_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_wakeup(dwt_isr_thr);
    return;
}
