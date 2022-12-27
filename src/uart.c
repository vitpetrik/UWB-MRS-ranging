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
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/console/tty.h>
#include "uart.h"
#include <stdio.h>

#include "common_variables.h"
#include "common_macro.h"

struct tty_serial tty;

struct k_mutex uart_rx_mutex;
struct k_mutex uart_tx_mutex;
K_SEM_DEFINE(rx_semaphore, 0, 1);
K_SEM_DEFINE(tx_semaphore, 1, 1);

uint8_t rx_buffer[1024];
uint8_t tx_buffer[1024];


/**
 * @brief Setup UART peripherial
 * 
 */
void uart_setup()
{
    int status;
    status = tty_init(&tty, uart_dev);
    __ASSERT(status == 0, "initiating tty failed");

    status  = tty_set_rx_buf(&tty, (void*) rx_buffer, sizeof(rx_buffer));
    __ASSERT(status == 0, "Wrong buffer");
    tty_set_tx_buf(&tty, (void*) tx_buffer, sizeof(tx_buffer));
    __ASSERT(status == 0, "Wrong buffer");

    return;
}

/**
 * @brief Write buffer to COM port
 *
 * @param data pointer to data
 * @param data_length length of the data in uint8_t size
 * @return uint8_t returns checksum of sent data
 */
uint8_t calc_cksum(uint8_t* data, int data_length)
{
    uint8_t checksum = 0;

    for (int i = 0; i < data_length; i++) {
        checksum += data[i];
	}

    return checksum;
}

/**
 * @brief Write buffer accoring to BACA protocol
 *
 * @param data pointer to data
 * @param data_length length of the data in uint8_t size
 */
void write_baca(struct baca_protocol *msg)
{
    // write protocol header

    uint8_t out_buffer[256];

    out_buffer[0] = 'b';
    out_buffer[1] = msg->payload_size;
    memcpy(&out_buffer[2], msg->payload, msg->payload_size);

    out_buffer[2 + msg->payload_size] = calc_cksum(out_buffer, 2 + msg->payload_size);

    write_buffer(out_buffer, 2 + msg->payload_size + 1);
    return;
}

/**
 * @brief Read data into buffer
 * 
 * @param buffer pointer to received payload
 * @param buflen length of the buffer
 * @return int length of received data
 */
int read_baca(struct baca_protocol *msg)
{
    msg->cksum = 0;
    msg->payload_size = 0;

    uint8_t c = read_char();
    msg->cksum += c;

    if(c != 'b')
        return -1;

    msg->payload_size = read_char();
    msg->cksum += msg->payload_size;

    if (255 < msg->payload_size)
        return -1;

    msg->payload = k_malloc(sizeof(uint8_t)*msg->payload_size);

    __ASSERT(msg->payload != NULL, "Failed allocating buffer of size %d Bytes!", msg->payload_size);

    read_buffer(msg->payload, msg->payload_size);

    msg->cksum += calc_cksum(msg->payload, msg->payload_size);

    uint8_t rx_cksum = read_char();

    if (rx_cksum != msg->cksum)
    {
        k_free(msg->payload);
        return -1;
    }

    return msg->payload_size;
}

/**
 * @brief Write single character
 * 
 * @param c
 */
void write_char(uint8_t c)
{
    int status = tty_write(&tty, (const void*) &c, 1);
    __ASSERT(status >= 0, "TTY Write failed!");
}

/**
 * @brief Write buffer to UART
 * 
 * @param buffer pointer to buffer that needs to be written out
 * @param len length of the data
 */
void write_buffer(uint8_t *buffer, int len)
{
    int status = tty_write(&tty, (const void*) buffer, len);
    __ASSERT(status >= 0, "TTY Write failed!");
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

    int status = tty_read(&tty, (void*) &c, 1);
    __ASSERT(status >= 0, "TTY read failed!");

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
    int status = tty_read(&tty, (void*) buf, len);
    __ASSERT(status >= 0, "TTY read failed!");

    return;
}
