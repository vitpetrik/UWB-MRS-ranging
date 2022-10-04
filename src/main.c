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

uint16_t DEVICE_ID;
uint16_t PAN_ID;
uint8_t SEQ_NUM;

// LEDS

static const struct gpio_dt_spec led0red = GPIO_DT_SPEC_GET(DT_NODELABEL(led0_red), gpios);
static const struct gpio_dt_spec led1green = GPIO_DT_SPEC_GET(DT_NODELABEL(led1_green), gpios);
static const struct gpio_dt_spec led2red = GPIO_DT_SPEC_GET(DT_NODELABEL(led2_red), gpios);
static const struct gpio_dt_spec led3blue = GPIO_DT_SPEC_GET(DT_NODELABEL(led3_blue), gpios);

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

static uint64_t get_sys_timestamp_u64(void);

int process_MAC(struct mac_data_t *mac_data, uint8_t *rx_buffer);

// THREADS
void uwb_tx_thread(void);
void uwb_rx_thread(void);

void dwt_isr_thread(void);

K_THREAD_DEFINE(uwb_beacon_thr, 1024, uwb_beacon_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(uwb_ranging_thr, 1024, uwb_ranging_thread, NULL, NULL, NULL, 5, 0, 0);

K_THREAD_DEFINE(uwb_ranging_print_thr, 1024, uwb_ranging_print_thread, NULL, NULL, NULL, 7, 0, 0);

K_THREAD_DEFINE(uwb_rx_thr, 1024, uwb_rx_thread, NULL, NULL, NULL, 3, 0, 0);
K_THREAD_DEFINE(uwb_tx_thr, 1024, uwb_tx_thread, NULL, NULL, NULL, -2, 0, 0);

K_THREAD_DEFINE(dwt_isr_thr, 1024, dwt_isr_thread, NULL, NULL, NULL, -1, 0, 0);

// CALLBACKS
void uwb_txdone(const dwt_cb_data_t *data);
void uwb_rxto(const dwt_cb_data_t *data);
void uwb_rxerr(const dwt_cb_data_t *data);
void uwb_rxfrej(const dwt_cb_data_t *data);
void uwb_rxok(const dwt_cb_data_t *data);

void dwm_int_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

// QUEUES
K_FIFO_DEFINE(tx_fifo);
K_FIFO_DEFINE(rx_fifo);

void main(void)
{
    // INITIALIZE
    printf("%s\n\r", APP_NAME);
    printf("%s\n\r", buildString);

    DEVICE_ID = NRF_FICR->DEVICEID[0] >> 16;
    PAN_ID = 0xdeca;
    SEQ_NUM = 0;

    printf("Device ID: 0x%X\n\r", DEVICE_ID);

    gpio_pin_configure_dt(&led0red, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led1green, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led2red, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led3blue, GPIO_OUTPUT_INACTIVE);

    // SETUP DW1000
    k_mutex_init(&dwt_mutex);
    k_mutex_lock(&dwt_mutex, K_FOREVER);

    dwt_hardreset();
    dwt_hardinterrupt(dwm_int_callback);

    dwt_spi_set_rate_slow();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        printf("INIT FAILED");
        while (1)
        {
        };
    }
    dwt_spi_set_rate_fast();

    dwt_configure(&config);

    dwt_setleds(DWT_LEDS_ENABLE);

    dwt_setaddress16(DEVICE_ID);
    dwt_setpanid(PAN_ID);

    dwt_enableframefilter(DWT_FF_DATA_EN);
    dwt_setdblrxbuffmode(1);

    dwt_settxantennadelay(21920);
    dwt_setrxantennadelay(21920);

    dwt_setcallbacks(uwb_txdone, uwb_rxok, uwb_rxto, uwb_rxerr, uwb_rxfrej);

    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_ARFE, 1);
    dwt_enable_interrupt();

    k_mutex_unlock(&dwt_mutex);

    printf("Settings done!\n\r");

    // START THE THREADS
    k_thread_resume(uwb_rx_thr);
    k_thread_resume(uwb_tx_thr);
    sleep_ms(500);
    k_thread_resume(uwb_beacon_thr);
    k_thread_resume(uwb_ranging_thr);

    while (1)
    {
        gpio_pin_toggle_dt(&led1green);
        sleep_ms(200);
    }

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

    printf("Rx thread started\n\r");

    while (1)
    {
        struct rx_queue_t *data = (struct rx_queue_t *)k_fifo_get(&rx_fifo, K_FOREVER);

        rx_message(data);

        // FREE QUEUE DATA
        k_free(data->buffer_rx_free_ptr);
        k_free(data);
    }
}

K_CONDVAR_DEFINE(tx_condvar);

// TRANSMIT THREAD
void uwb_tx_thread(void)
{
    k_tid_t thread_id = k_current_get();
    k_thread_suspend(thread_id);

    printf("Tx thread started\n\r");
    k_condvar_init(&tx_condvar);

    while (1)
    {
        // GET DATA FOR TRANSMITTING THROUGH QUEUE
        struct tx_queue_t *data_tx = (struct tx_queue_t *)k_fifo_get(&tx_fifo, K_FOREVER);

        k_mutex_lock(&dwt_mutex, K_FOREVER);

        // TURN OFF RECEIVER
        if (dwt_read32bitreg(SYS_STATE_ID) & 0x0f00)
            dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8)SYS_CTRL_TRXOFF); // Disable the radio

        // SET DELAYED TX TIMESTAMP
        if (data_tx->tx_details.tx_mode & DWT_START_TX_DELAYED)
        {
            uint64_t sys_time = get_sys_timestamp_u64();
            uint64_t tx_timestamp = sys_time + (data_tx->tx_details.tx_delay.reserved_time * UUS_TO_DWT_TIME);

            tx_timestamp &= 0xffffffffffffff00;
            uint32_t delay = tx_timestamp - data_tx->tx_details.tx_delay.rx_timestamp;

            memcpy(&(data_tx->frame_buffer[10]), &delay, sizeof(uint32_t));
            dwt_setdelayedtrxtime((uint32_t)(tx_timestamp >> 8));
        }

        // WRITE DATA TO TX BUFFER
        dwt_writetxdata(data_tx->frame_length, data_tx->frame_buffer, 0);
        dwt_writetxfctrl(data_tx->frame_length, 0, data_tx->tx_details.ranging);

        // START TRANSMITTING
        if ((dwt_starttx(data_tx->tx_details.tx_mode) == DWT_SUCCESS) && (k_condvar_wait(&tx_condvar, &dwt_mutex, K_MSEC(1)) == 0))
        {
            uint64_t *timestamp_ptr = data_tx->tx_details.tx_timestamp;
            if (timestamp_ptr != NULL)
            {
                uint64_t timestamp = get_tx_timestamp_u64();
                *timestamp_ptr = timestamp;
            }
        }
        else
        {
            printf("Tx fail\n\r");
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
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}
// RX ERROR
void uwb_rxerr(const dwt_cb_data_t *data)
{
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}
void uwb_rxfrej(const dwt_cb_data_t *data)
{
    dwt_rxenable(DWT_START_RX_IMMEDIATE | DWT_NO_SYNC_PTRS);
}
// RECEIVE OK
void uwb_rxok(const dwt_cb_data_t *data)
{
    //! READ the integrator first - after rxenable the value gets overwritten
    int32_t integrator = dwt_readcarrierintegrator();
    dwt_rxenable(DWT_START_RX_IMMEDIATE | DWT_NO_SYNC_PTRS);

    // INIT DATA
    uint16_t msg_length = data->datalength;
    uint8_t *buffer_rx = k_malloc(msg_length);

    // READ THE DATA AND ENABLE RX
    uint64_t rx_ts = get_rx_timestamp_u64();
    dwt_readrxdata(buffer_rx, msg_length, 0);

    struct rx_details_t rx_details = {.carrier_integrator = integrator, .rx_timestamp = rx_ts};

    // INIT QUEUE DATA AND SEND TO RX THREAD THROUGH QUEUE

    struct mac_data_t mac_data;
    int mac_lenght = process_MAC(&mac_data, buffer_rx);

    if (mac_data.pan_id != PAN_ID || (mac_data.destination_id != DEVICE_ID && mac_data.destination_id != 0xffff))
    {
        k_free(buffer_rx);
        return;
    }

    struct rx_queue_t *queue = k_malloc(sizeof(struct rx_queue_t));
    queue->mac_data = mac_data;
    queue->buffer_rx = buffer_rx + mac_lenght;
    queue->buffer_rx_free_ptr = buffer_rx;
    queue->rx_details = rx_details;

    k_fifo_alloc_put(&rx_fifo, queue);

    return;
}

K_SEM_DEFINE(isr_semaphore, 0, 10);

void dwt_isr_thread(void)
{
    while (1)
    {
        k_sem_take(&isr_semaphore, K_FOREVER);
        k_mutex_lock(&dwt_mutex, K_FOREVER);
        dwt_isr();
        k_mutex_unlock(&dwt_mutex);
    }
}

void dwm_int_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_sem_give(&isr_semaphore);
    return;
}

int process_MAC(struct mac_data_t *mac_data, uint8_t *buffer_rx)
{
    int length = 0;

    memcpy(&mac_data->frame_ctrl, buffer_rx, sizeof(uint16_t));
    buffer_rx += sizeof(uint16_t);
    length += sizeof(uint16_t);

    // SEQUENCE NUMBER
    memcpy(&mac_data->seq_num, buffer_rx, sizeof(uint8_t));
    buffer_rx += sizeof(uint8_t);
    length += sizeof(uint8_t);

    memcpy(&mac_data->pan_id, buffer_rx, sizeof(uint16_t));
    buffer_rx += sizeof(uint16_t);
    length += sizeof(uint16_t);

    memcpy(&mac_data->destination_id, buffer_rx, sizeof(uint16_t));
    buffer_rx += sizeof(uint16_t);
    length += sizeof(uint16_t);

    memcpy(&mac_data->source_id, buffer_rx, sizeof(uint16_t));
    buffer_rx += sizeof(uint16_t);
    length += sizeof(uint16_t);

    memcpy(&mac_data->msg_type, buffer_rx, sizeof(uint8_t));
    buffer_rx += sizeof(uint8_t);
    length += sizeof(uint8_t);

    memcpy(&mac_data->tx_delay, buffer_rx, sizeof(uint32_t));
    buffer_rx += sizeof(uint32_t);
    length += sizeof(uint32_t);

    return length;
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

// GET TX TIMESTAMP IN 40-BIT FORMAT
static uint64_t get_sys_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readsystime(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts = (ts << 8) + ts_tab[i];
    }
    ts &= (uint64_t)0x000000ffffffffff;
    return ts;
}
