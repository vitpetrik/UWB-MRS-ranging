/**
 * @file uwb_threads.h
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief 
 * @version 0.1
 * @date 2022-11-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __UWB_THREADS_H__
#define __UWB_THREADS_H__

/**
 * @brief Transmitting thread, waits for queue
 *
 */
void uwb_tx_thread(void);

/**
 * @brief Receiving thread, waits foe queue
 *
 */
void uwb_rx_thread(void);

/**
 * @brief Thread that handles dwt interrupts, gets woken up by interrupt
 * 
 */
void dwt_isr_thread(void);

/**
 * @brief Interrupt Transission is dones
 * 
 * @param data Pointer to callback data
 */
void uwb_txdone(const dwt_cb_data_t *data);

/**
 * @brief Receiver timeout interrupt
 * 
 * @param data Pointer to callback data
 */
void uwb_rxto(const dwt_cb_data_t *data);

/**
 * @brief Receiver error interrupt
 * 
 * @param data Pointer to callback data
 */
void uwb_rxerr(const dwt_cb_data_t *data);

/**
 * @brief Receiver frame rejection interrupt
 * 
 * @param data Pointer to callback data
 */
void uwb_rxfrej(const dwt_cb_data_t *data);

/**
 * @brief Receiver successful
 * 
 * @param data Pointer to callback data
 */
void uwb_rxok(const dwt_cb_data_t *data);

/**
 * @brief Gets called when UWB trigger hardware reciver
 * 
 * @param dev Pointer to GPIO Bank
 * @param cb Callback data
 * @param pins Pin that got interrupted
 */
void dwm_int_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

#endif