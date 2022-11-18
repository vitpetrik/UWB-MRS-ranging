/**
 * @file uart.h
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

/**
 * @brief Setup UART peripherial
 * 
 */
void uart_setup();

/**
 * @brief Write single character
 * 
 * @param c
 */
void write_char(uint8_t c);

/**
 * @brief Write buffer to UART
 * 
 * @param buffer pointer to buffer that needs to be written out
 * @param len length of the data
 */
void write_buffer(uint8_t *buffer, int len);

/**
 * @brief Read single character
 * 
 * @return uint8_t character from UART
 */
uint8_t read_char();

/**
 * @brief Read bigger chunk of data to buffer
 * 
 * @param buf pointer to buffer
 * @param len length of the data to be received
 */
void read_buffer(uint8_t *buf, int len);

/**
 * @brief UART callback interrupt
 * 
 * @param dev pointer to device structure
 * @param evt pointer to event structure
 * @param user_data user data
 */
void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data);
