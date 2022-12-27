/**
 * @file uwb_threads.c
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief
 * @version 0.1
 * @date 2022-11-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uwb_threads);

#include <stdio.h>

#include "deca_regs.h"
#include "platform.h"

#include "common_types.h"
#include "common_macro.h"
#include "common_variables.h"
#include "ranging.h"

#include "mac.h"

// GLOBAL VARIABLES
struct k_mutex dwt_mutex;

// QUEUES
// K_FIFO_DEFINE(uwb_tx_fifo);
K_MSGQ_DEFINE(uwb_tx_msgq, sizeof(struct tx_queue_t), 10, 4);
K_MSGQ_DEFINE(uwb_rx_msgq, sizeof(struct rx_queue_t), 10, 4);

// Condvar for waiting for the interrupt
K_CONDVAR_DEFINE(tx_condvar);

/**
 * @brief Transmit data from the queue
 *
 * @param tx_queue pointer to queue data
 */
void uwb_tx(struct tx_queue_t *tx_queue)
{
    static uint32_t reset_rx_time = 0;

    // Get MAC data and Transmit details
    struct mac_data_t *mac_data = (struct mac_data_t *)&(tx_queue->mac_data);
    struct tx_details_t *tx_details = (struct tx_details_t *)&(tx_queue->tx_details);

    // Serialize MAC
    int mac_length = encode_MAC(mac_data, tx_queue->frame_buffer);

    // SET DELAYED TX TIMESTAMP
    if (tx_queue->tx_details.tx_mode & DWT_START_TX_DELAYED)
    {
        // calculate Tx timestamp accoring to tx_details data
        uint64_t sys_time = get_sys_timestamp_u64();
        uint64_t tx_timestamp = sys_time + (tx_details->tx_delay.reserved_time * UUS_TO_DWT_TIME);

        // Actual data sent to UWB doesnt have lowest 8 bits
        tx_timestamp &= 0xffffffffffffff00;
        uint32_t delay = tx_timestamp - tx_details->tx_delay.rx_timestamp;

        // handle overflow
        tx_timestamp %= 0x000000ffffffffffU;

        // set the final tx data to the UWB and to MAC data
        dwt_setdelayedtrxtime((uint32_t)(tx_timestamp >> 8));

        memcpy(&tx_queue->frame_buffer[mac_length+tx_details->tx_delay.offset], &delay, sizeof(delay));
        LOG_DBG("Setting delay to %d", delay);
    }

    // WRITE DATA TO TX BUFFER
    dwt_writetxdata(mac_length + tx_queue->frame_length + 2, tx_queue->frame_buffer, 0);
    dwt_writetxfctrl(mac_length + tx_queue->frame_length + 2, 0, tx_details->ranging);

    // TURN OFF RECEIVER
    dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8)SYS_CTRL_TRXOFF); // Disable the radio

    int status = dwt_starttx(tx_details->tx_mode);

    __ASSERT(status == DWT_SUCCESS, "Could not start UWB transmission");

    status = k_condvar_wait(&tx_condvar, &dwt_mutex, K_MSEC(10));

    //! Workaround for issue described at DW1000 errata "TX-1"
    //! Don't you ever dare to delete this!
    if(status != 0 || k_uptime_get_32() - reset_rx_time > 60000)
    {
        LOG_WRN("Resseting RX as an workaround");
        dwt_forcetrxoff();
        dwt_rxreset();
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        reset_rx_time = k_uptime_get_32();
    }

    // Save Tx timestamp if needed
    uint64_t *timestamp_ptr = tx_details->tx_timestamp;
    if (timestamp_ptr != NULL)
    {
        uint64_t timestamp = get_tx_timestamp_u64();
        *timestamp_ptr = timestamp;
    }

    return;
}

/**
 * @brief Transmitting thread, waits for queue
 *
 */
void uwb_tx_thread(void)
{
    k_tid_t thread = k_current_get();
    k_thread_suspend(thread);

    LOG_DBG("Tx thread started");
    int status = k_condvar_init(&tx_condvar);

    __ASSERT(status == 0, "Condvar initializtion didnt go through");

    struct tx_queue_t tx_queue;

    while (1)
    {
        // GET DATA FOR TRANSMITTING THROUGH QUEUE
        status = k_msgq_get(&uwb_tx_msgq, &tx_queue, K_FOREVER);

        __ASSERT(status == 0, "Getting message failed");

        // Get the mutex and send the data to lower layer
        status = k_mutex_lock(&dwt_mutex, K_MSEC(100));

        __ASSERT(status == 0, "Mutex did not lock");

        uwb_tx(&tx_queue);

        // UNLOCK THE MTX AND FREE THE DATA
        status = k_mutex_unlock(&dwt_mutex);

        __ASSERT(status == 0, "Mutex did not unlock");

        k_free(tx_queue.frame_buffer);
    }
}

/**
 * @brief Thread that handles dwt interrupts, gets woken up by interrupt
 *
 */
void dwt_isr_handler(struct k_work *item)
{
    LOG_DBG("Working on dwt_isr routine");

    int status = k_mutex_lock(&dwt_mutex, K_MSEC(100));
    __ASSERT(status == 0, "Mutex did not lock");

    dwt_isr();

    k_mutex_unlock(&dwt_mutex);
    return;
}
K_WORK_DEFINE(isr_work, dwt_isr_handler);

/**
 * @brief Interrupt Transission is dones
 *
 * @param data Pointer to callback data
 */
void uwb_txdone(const dwt_cb_data_t *data)
{
    LOG_DBG("Issuing txdone condvar");
    int status = k_condvar_signal(&tx_condvar);
    __ASSERT(status == 0, "Did not signal to condvar!");
    return;
}

/**
 * @brief Receiver timeout interrupt
 *
 * @param data Pointer to callback data
 */
void uwb_rxto(const dwt_cb_data_t *data)
{
    LOG_DBG("RX timeout callback");
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    return;
}

/**
 * @brief Receiver error interrupt
 *
 * @param data Pointer to callback data
 */
void uwb_rxerr(const dwt_cb_data_t *data)
{
    LOG_DBG("RX error callback");
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    LOG_ERR("RX ERROR detected 0x%X", data->status);

    if(data->status & SYS_STATUS_RXOVRR)
    {
        LOG_ERR("RXOVRR error happend!");
    }
    return;
}

/**
 * @brief Receiver frame rejection interrupt
 *
 * @param data Pointer to callback data
 */
void uwb_rxfrej(const dwt_cb_data_t *data)
{
    LOG_DBG("RX frame rejection callback");
    dwt_rxenable(DWT_START_RX_IMMEDIATE | DWT_NO_SYNC_PTRS);
    return;
}

/**
 * @brief Receiver successful
 *
 * @param data Pointer to callback data
 */
void uwb_rxok(const dwt_cb_data_t *data)
{
    LOG_DBG("RX OK callback");

    // INIT DATA
    int status;

    static struct rx_queue_t queue;
    uint16_t msg_length = data->datalength;
    queue.frame_length = msg_length;

    uint8_t *buffer = (uint8_t*) k_malloc(msg_length);

    __ASSERT(buffer != NULL, "Failed to allocate buffer");

    queue.buffer_rx = buffer;

    // READ THE DATA AND ENABLE RX
    uint64_t rx_ts = get_rx_timestamp_u64();
    dwt_readrxdata(queue.buffer_rx, msg_length, 0);

    struct rx_details_t rx_details = {.carrier_integrator = 0, .rx_timestamp = rx_ts, .rx_power = data->rx_power};
    queue.rx_details = rx_details;

    // INIT QUEUE DATA AND SEND TO RX THREAD THROUGH QUEUE
    queue.buf_offset = decode_MAC(&queue.mac_data, queue.buffer_rx);

    status = k_msgq_put(&uwb_rx_msgq, &queue, K_FOREVER);
    __ASSERT(status == 0, "Putting message to queue Fail!");

    return;
}

/**
 * @brief Gets called when UWB trigger hardware reciver
 *
 * @param dev Pointer to GPIO Bank
 * @param cb Callback data
 * @param pins Pin that got interrupted
 */
void dwm_int_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    int status = k_work_submit(&isr_work);
    __ASSERT(status >= 0, "Submitting to workqueue Failed!");

    return;
}
