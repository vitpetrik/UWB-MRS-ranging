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
#include <timing/timing.h>
#include <string.h>

#include "deca_regs.h"
#include "platform.h"

#include "common_types.h"
#include "common_macro.h"
#include "ranging.h"

/* Example application name and version to display on LCD screen. */
#define APP_NAME "MRS UWB v0.1"
const char *buildString = "Build was compiled at " __DATE__ ", " __TIME__ ".";

uint32_t DEVICE_ID;

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

// GLOBAL VARIABLES

struct k_mutex dwt_mutex;

// FUNCTIONS
static uint64_t get_rx_timestamp_u64(void);
static uint64_t get_tx_timestamp_u64(void);

// THREADS
void uwb_tx_thread(void);
void uwb_rx_thread(void);

void dwt_isr_thread(void);

K_THREAD_DEFINE(uwb_beacon_thr, 2048, uwb_beacon_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(uwb_ranging_thr, 2048, uwb_ranging_thread, NULL, NULL, NULL, 5, 0, 0);

K_THREAD_DEFINE(uwb_rx_thr, 2048, uwb_rx_thread, NULL, NULL, NULL, -2, 0, 0);
K_THREAD_DEFINE(uwb_tx_thr, 2048, uwb_tx_thread, NULL, NULL, NULL, -1, 0, 0);

K_THREAD_DEFINE(dwt_isr_thr, 2048, dwt_isr_thread, NULL, NULL, NULL, -1, 0, 0);

// CALLBACKS
void uwb_txdone(const dwt_cb_data_t *data);
void uwb_rxto(const dwt_cb_data_t *data);
void uwb_rxerr(const dwt_cb_data_t *data);
void uwb_rxok(const dwt_cb_data_t *data);

void dwm_int_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

// QUEUES
K_FIFO_DEFINE(tx_fifo);
K_FIFO_DEFINE(rx_fifo);

void main(void)
{
    // INITIALIZE

    printk("%s\n\r", APP_NAME);
    printk("%s\n\r", buildString);

    DEVICE_ID = NRF_FICR->DEVICEID[0];

    printk("Device ID: 0x%X\n\r", DEVICE_ID);

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

    dwt_settxantennadelay(21920);
    dwt_setrxantennadelay(21920);

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
    k_thread_resume(uwb_ranging_thr);

    return;
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
        struct rx_queue_t *data = (struct rx_queue_t *)k_fifo_get(&rx_fifo, K_FOREVER);
        data_msg data_rx;

        // DECODE THE DATA
        pb_istream_t stream = pb_istream_from_buffer(data->buffer_rx, data->cb_data->datalength);
        pb_decode(&stream, data_msg_fields, &data_rx);

        int status = SUCCESS;

        // PROCESS THE MESSAGE
        switch (data_rx.which_data)
        {
        case data_msg_beacon_tag:
            status = process_beacon(&data_rx.data.beacon);
            break;
        case data_msg_ranging_tag:
            status = process_ranging(&data_rx.data.ranging, data);
            break;
        default:
            printk("Unknown message\n\r");
            break;
        }

        if (status == RX_ENABLE)
        {
            k_mutex_lock(&dwt_mutex, K_FOREVER);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            k_mutex_unlock(&dwt_mutex);
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
        struct tx_queue_t *data_tx = (struct tx_queue_t *)k_fifo_get(&tx_fifo, K_FOREVER);

        k_mutex_lock(&dwt_mutex, K_FOREVER);

        // TURN OFF RECEIVER
        if (dwt_read32bitreg(SYS_STATE_ID) & 0x0f00)
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
        if ((dwt_starttx(data_tx->tx_mode) == DWT_SUCCESS) && (k_condvar_wait(&tx_condvar, &dwt_mutex, K_MSEC(10)) == 0))
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

    k_fifo_put(&rx_fifo, queue);

    return;
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
