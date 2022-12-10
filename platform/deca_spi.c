/*! ----------------------------------------------------------------------------
 * @file	deca_spi.c
 * @brief	SPI access functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include <string.h>

#include <zephyr/zephyr.h>
#include <zephyr/sys/__assert.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "deca_device_api.h"
#include "platform.h"

struct device *spi = DEVICE_DT_GET(DT_NODELABEL(spi2));

const struct spi_cs_control cs_ctrl = {
    .gpio = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
        .pin = 17,
        .dt_flags = GPIO_ACTIVE_LOW},
    .delay = 0,
};

struct spi_config dwt_spi_cfg_slow = {
    .frequency = 2000000U,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
    .cs = &cs_ctrl};
struct spi_config dwt_spi_cfg_fast = {
    .frequency = 8000000U,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
    .cs = &cs_ctrl};

struct spi_config *dwt_spi_cfg;

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
    struct spi_buf spi_buf_tx[] = {
        {.buf = (uint8_t*) headerBuffer,
         .len = headerLength},
        {.buf = (uint8_t*) bodyBuffer,
         .len = bodylength}};

    struct spi_buf spi_buf_rx[] = {
        {.buf = NULL,
         .len = headerLength},
        {.buf = NULL,
         .len = bodylength}};

    struct spi_buf_set buff_tx = {.buffers = spi_buf_tx, .count = 2};
    struct spi_buf_set buff_rx = {.buffers = spi_buf_rx, .count = 2};

    int error = spi_transceive(spi, dwt_spi_cfg, &buff_tx, &buff_rx);
    __ASSERT(error == 0, "Error in SPI transmission");

    return 0;

} // end writetospi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
    uint8 empty_buffer[readlength];
    memset(empty_buffer, 0, readlength);
    const struct spi_buf spi_buf_tx[] = {
        {.buf = (uint8_t*) headerBuffer,
         .len = headerLength},
        {.buf = empty_buffer,
         .len = readlength}};
    struct spi_buf spi_buf_rx[] = {
        {.buf = NULL,
         .len = headerLength},
        {.buf = readBuffer,
         .len = readlength}};

    struct spi_buf_set buff_tx = {.buffers = spi_buf_tx, .count = 2};
    struct spi_buf_set buff_rx = {.buffers = spi_buf_rx, .count = 2};

    int error = spi_transceive(spi, dwt_spi_cfg, &buff_tx, &buff_rx);
    __ASSERT(error == 0, "Error in SPI transmission");

    return 0;
} // end readfromspi()

void dwt_spi_set_rate_slow()
{
    dwt_spi_cfg = &dwt_spi_cfg_slow;
}
void dwt_spi_set_rate_fast()
{
    dwt_spi_cfg = &dwt_spi_cfg_fast;
}
