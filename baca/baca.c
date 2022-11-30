/**
 * @file baca.c
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief Sending data through com port with BACA protocol
 * Info about BACA protocol at https://github.com/ctu-mrs/mrs_serial#mrs-serial-protocol
 * @version 0.1
 * @date 2022-11-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include "common_variables.h"

#include <string.h>
#include "baca.h"

#include "uart.h"

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
void write_baca(uint8_t* data, int data_length)
{
    // write protocol header
    uint8_t checksum = 'b';
    write_char('b');

    // write payload size
    uint8_t payload_size = data_length;
    write_char(payload_size);
    checksum += payload_size;

    // write the actual data
    write_buffer(data, data_length);
    checksum += calc_cksum(data, data_length);

    // send checksum
    write_char(checksum);

    return;
}

/**
 * @brief Read data into buffer
 * 
 * @param buffer pointer to received payload
 * @param buflen length of the buffer
 * @return int length of received data
 */
int read_baca(uint8_t *buffer, int buflen)
{
    uint8_t cksum = 0;

    uint8_t c = read_char();
    cksum += c;

    if(c != 'b')
        return -1;

    uint8_t payload_length = read_char();
    cksum += payload_length;

    if (buflen < payload_length)
        return -1;

    read_buffer(buffer, payload_length);

    cksum += calc_cksum(buffer, payload_length);

    uint8_t rx_cksum = read_char();

    if (rx_cksum != cksum)
        return -1;

    return payload_length;
}
