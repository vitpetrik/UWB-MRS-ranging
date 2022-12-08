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


#include <zephyr/zephyr.h>
#include <zephyr/sys/__assert.h>

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
K_MSGQ_DEFINE(uwb_tx_msgq, sizeof(struct tx_queue_t), 5, 4);
K_MSGQ_DEFINE(uwb_rx_msgq, sizeof(struct rx_queue_t), 5, 4);

// Condvar for waiting for the interrupt
K_CONDVAR_DEFINE(tx_condvar);

/**
 * @brief Transmit data from the queue
 * 
 * @param tx_queue pointer to queue data
 */
void uwb_tx(struct tx_queue_t *tx_queue)
{
    // Get MAC data and Transmit details
    struct mac_data_t *mac_data = (struct mac_data_t *) &(tx_queue->mac_data);
    struct tx_details_t *tx_details = (struct tx_details_t *) &(tx_queue->tx_details);

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
        mac_data->tx_delay = delay;
    }

    // Serialize MAC
    int mac_length = encode_MAC(mac_data, tx_queue->frame_buffer);

    // WRITE DATA TO TX BUFFER
    dwt_writetxdata(mac_length + tx_queue->frame_length + 2, tx_queue->frame_buffer, 0);
    dwt_writetxfctrl(mac_length + tx_queue->frame_length + 2, 0, tx_details->ranging);

    // TURN OFF RECEIVER
    dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8)SYS_CTRL_TRXOFF); // Disable the radio
    // START TRANSMITTING
    if ((dwt_starttx(tx_details->tx_mode) == DWT_SUCCESS) && (k_condvar_wait(&tx_condvar, &dwt_mutex, K_MSEC(1)) == 0))
    {
        // Save Tx timestamp if needed
        uint64_t *timestamp_ptr = tx_details->tx_timestamp;
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
}

/**
 * @brief Transmitting thread, waits for queue
 * 
 */
void uwb_tx_thread(void)
{
    printf("Tx thread started\n\r");
    k_condvar_init(&tx_condvar);

    struct tx_queue_t tx_queue;

    while (1)
    {
        // GET DATA FOR TRANSMITTING THROUGH QUEUE
        k_msgq_get(&uwb_tx_msgq, &tx_queue, K_FOREVER);

        // Get the mutex and send the data to lower layer
        k_mutex_lock(&dwt_mutex, K_FOREVER);

        uwb_tx(&tx_queue);

        // UNLOCK THE MTX AND FREE THE DATA
        k_mutex_unlock(&dwt_mutex);
    }
}

/**
 * @brief Thread that handles dwt interrupts, gets woken up by interrupt
 * 
 */
void dwt_isr_handler(struct k_work *item)
{
    k_mutex_lock(&dwt_mutex, K_FOREVER);
    dwt_isr();
    k_mutex_unlock(&dwt_mutex);
}
K_WORK_DEFINE(isr_work , dwt_isr_handler);


/**
 * @brief Interrupt Transission is dones
 * 
 * @param data Pointer to callback data
 */
void uwb_txdone(const dwt_cb_data_t *data)
{
    k_condvar_signal(&tx_condvar);
    return;
}

/**
 * @brief Receiver timeout interrupt
 * 
 * @param data Pointer to callback data
 */
void uwb_rxto(const dwt_cb_data_t *data)
{
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
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    return;
}

/**
 * @brief Receiver frame rejection interrupt
 * 
 * @param data Pointer to callback data
 */
void uwb_rxfrej(const dwt_cb_data_t *data)
{
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
    //! READ the integrator first - after rxenable the value gets overwritten
    // int32_t integrator = dwt_readcarrierintegrator();
    // dwt_rxenable(DWT_START_RX_IMMEDIATE | DWT_NO_SYNC_PTRS);

    // INIT DATA
    static struct rx_queue_t queue;
    uint16_t msg_length = data->datalength;

    // READ THE DATA AND ENABLE RX
    uint64_t rx_ts = get_rx_timestamp_u64();
    dwt_readrxdata(queue.buffer_rx, msg_length, 0);

    struct rx_details_t rx_details = {.carrier_integrator = 0, .rx_timestamp = rx_ts, .rx_power = data->rx_power};
    queue.rx_details = rx_details;

    // INIT QUEUE DATA AND SEND TO RX THREAD THROUGH QUEUE
    queue.buf_offset = decode_MAC(&queue.mac_data, queue.buffer_rx);

    k_msgq_put(&uwb_rx_msgq, &queue, K_FOREVER);

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
    k_work_submit(&isr_work);

    return;
}