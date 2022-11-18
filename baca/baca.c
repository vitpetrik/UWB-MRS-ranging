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

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_NODELABEL(uart0)

struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/**
 * @brief Write buffer to COM port
 *
 * @param data pointer to data
 * @param data_length length of the data in uint8_t size
 * @return uint8_t returns checksum of sent data
 */
uint8_t write_buf(void* data, int data_length)
{
    unsigned char *data_char = (unsigned char *) data;
    uint8_t checksum = 0;

    for (int i = 0; i < data_length; i++) {
		uart_poll_out(uart_dev, data_char[i]);
        checksum += data_char[i];
	}

    return checksum;
}

/**
 * @brief Write buffer accoring to BACA protocol
 *
 * @param data pointer to data
 * @param data_length length of the data in uint8_t size
 */
void write_baca(void* data, int data_length)
{
    // write protocol header
    uint8_t checksum = 'b';
    uart_poll_out(uart_dev, 'b');

    // write payload size
    uint8_t payload_size = 1 + data_length;
    uart_poll_out(uart_dev, payload_size);
    checksum += payload_size;

    // write message id
    uart_poll_out(uart_dev, MESSAGE_ID);
    checksum += MESSAGE_ID;

    // write the actual data
    checksum += write_buf(data, data_length);

    // send checksum
    uart_poll_out(uart_dev, checksum);

    return;
}

/**
 * @brief Read data into buffer
 * 
 * @param buffer pointer to receiver buffer
 * @param buflen length of the buffer
 * @return int length of received data
 */
int read_baca(void *buffer, int buflen)
{
  //   if (Serial.available() > 2) {
  //   uint8_t checksum = 0;
  //   uint8_t tmp_in;
  //   uint8_t id;

  //   tmp_in = Serial.read();

  //   //start of message
  //   if (tmp_in == 'b') {
  //     checksum += tmp_in;
  //     tmp_in = Serial.read();

  //     // payload
  //     if (tmp_in == 1) {
  //       checksum += tmp_in;

  //       // id
  //       id = Serial.read();
  //       checksum += id;


  //       // checksum
  //       if (checksum == Serial.read()) {
  //         return id;
  //       }
  //     }
  //   }
  //   // bad checksum
  //   return 255;
  // }
}
