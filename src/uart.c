/**
 * @file uart.c
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief Threads to handle ROS communication at the lowest level
 * @version 0.1
 * @date 2022-11-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <zephyr/zephyr.h>
#include <zephyr/drivers/uart.h>
#include "uart.h"
#include <stdio.h>


#include "common_variables.h"

struct k_mutex uart_rx_mutex;
struct k_mutex uart_tx_mutex;
K_SEM_DEFINE(rx_semaphore, 0, 1);
K_SEM_DEFINE(tx_semaphore, 1, 1);


/**
 * @brief Setup UART peripherial
 * 
 */
void uart_setup()
{
    uart_callback_set(uart_dev, uart_callback, NULL);
    k_mutex_init(&uart_rx_mutex);
    k_mutex_init(&uart_tx_mutex);

    return;
}

/**
 * @brief Write single character
 * 
 * @param c
 */
void write_char(uint8_t c)
{
    write_buffer(&c, 1);
}

/**
 * @brief Write buffer to UART
 * 
 * @param buffer pointer to buffer that needs to be written out
 * @param len length of the data
 */
void write_buffer(uint8_t *buffer, int len)
{
    k_mutex_lock(&uart_tx_mutex, K_FOREVER);
    k_sem_take(&tx_semaphore, K_FOREVER);
    int status = uart_tx(uart_dev, buffer, len, SYS_FOREVER_US);
    k_mutex_unlock(&uart_tx_mutex);

    return;
}

/**
 * @brief Read single character
 * 
 * @return uint8_t character from UART
 */
uint8_t read_char()
{
    uint8_t c;

    read_buffer(&c, 1);

    return c;
}

/**
 * @brief Read bigger chunk of data to buffer
 * 
 * @param buf pointer to buffer
 * @param len length of the data to be received
 */
void read_buffer(uint8_t *buf, int len)
{
    k_mutex_lock(&uart_rx_mutex, K_FOREVER);


    uart_rx_enable(uart_dev, buf, len, 0);

    k_sem_take(&rx_semaphore, K_FOREVER);
    k_mutex_unlock(&uart_rx_mutex);

    return;
}

/**
 * @brief UART callback interrupt
 * 
 * @param dev pointer to device structure
 * @param evt pointer to event structure
 * @param user_data user data
 */
void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
    int event_type = evt->type;

    switch (event_type)
    {
    case UART_RX_DISABLED:
        k_sem_give(&rx_semaphore);
        break;
    case UART_TX_DONE:
        k_sem_give(&tx_semaphore);
    default:
        break;
    }

    return;
}
