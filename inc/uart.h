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

#ifndef __UART_H__
#define __UART_H__


#include <zephyr/zephyr.h>
#include <zephyr/drivers/uart.h>

struct baca_protocol {
    uint8_t cksum;

    uint8_t payload_size;
    uint8_t payload[255];
};

#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Setup UART peripherial
 * 
 */
void uart_setup();

/**
 * @brief Write buffer to COM port
 * 
 * @param data pointer to data
 * @param data_length length of the data in uint8_t size
 * @return uint8_t returns checksum of sent data
 */
uint8_t calc_cksum(uint8_t *data, int data_length);

/**
 * @brief Write buffer accoring to BACA protocol
 * 
 * @param data pointer to data
 * @param data_length length of the data in uint8_t size
 */
void write_baca(struct baca_protocol *msg);

/**
 * @brief Read data into buffer
 * 
 * @param buffer pointer to received payload
 * @param buflen length of the buffer
 * @return int length of received data
 */
int read_baca(struct baca_protocol *msg);

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

#ifdef __cplusplus
}
#endif

#endif
