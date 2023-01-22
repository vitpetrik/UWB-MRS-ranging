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

#include <zephyr/drivers/pinctrl.h>

#include "deca_device_api.h"
#include "platform.h"

#include <nrfx_spim.h>

#define SPIM_NODE  DT_NODELABEL(spi2)
static nrfx_spim_t spim = NRFX_SPIM_INSTANCE(2);

const struct spi_cs_control cs_ctrl = {
    .gpio = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
        .pin = 17,
        .dt_flags = GPIO_ACTIVE_LOW},
    .delay = 0,
};

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
    uint8_t tx_data[headerLength + bodylength];

    memcpy(&tx_data[0], headerBuffer, headerLength);
    memcpy(&tx_data[headerLength], bodyBuffer, bodylength);

    nrfx_spim_xfer_desc_t xfer_desc = {
		.p_tx_buffer = tx_data,
		.tx_length = headerLength + bodylength,
		.p_rx_buffer = NULL,
		.rx_length = 0,
	};


	nrf_gpio_pin_write(NRF_DT_GPIOS_TO_PSEL(SPIM_NODE, cs_gpios), 0);
    int error = nrfx_spim_xfer(&spim, &xfer_desc, 0);
	nrf_gpio_pin_write(NRF_DT_GPIOS_TO_PSEL(SPIM_NODE, cs_gpios), 1);
    __ASSERT(error == NRFX_SUCCESS, "Error in SPI transmission");

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
    uint8_t tx_data[headerLength + readlength];
    uint8_t rx_data[headerLength + readlength];
    memset(tx_data, headerLength + readlength, sizeof(uint8_t));
    memset(rx_data, headerLength + readlength, sizeof(uint8_t));

    memcpy(tx_data, headerBuffer, headerLength);

    nrfx_spim_xfer_desc_t xfer_desc = {
		.p_tx_buffer = tx_data,
		.tx_length = headerLength + readlength,
		.p_rx_buffer = rx_data,
		.rx_length = headerLength + readlength,
	};

	nrf_gpio_pin_write(NRF_DT_GPIOS_TO_PSEL(SPIM_NODE, cs_gpios), 0);
    int error = nrfx_spim_xfer(&spim, &xfer_desc, 0);
	nrf_gpio_pin_write(NRF_DT_GPIOS_TO_PSEL(SPIM_NODE, cs_gpios), 1);
    __ASSERT(error == NRFX_SUCCESS, "Error in SPI transmission");

    memcpy(readBuffer, &rx_data[headerLength], readlength);

    return 0;
} // end readfromspi()

void dwt_spi_set_rate_slow()
{
    PINCTRL_DT_DEFINE(SPIM_NODE);

	nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG(
		NRFX_SPIM_PIN_NOT_USED,
		NRFX_SPIM_PIN_NOT_USED,
		NRFX_SPIM_PIN_NOT_USED,
		NRF_DT_GPIOS_TO_PSEL(SPIM_NODE, cs_gpios));
	spim_config.frequency = NRF_SPIM_FREQ_2M;
	spim_config.skip_gpio_cfg = true;
	spim_config.skip_psel_cfg = true;

	pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(SPIM_NODE),
				  PINCTRL_STATE_DEFAULT);
	nrfx_spim_init(&spim, &spim_config, NULL, NULL);

    nrf_gpio_pin_write(NRF_DT_GPIOS_TO_PSEL(SPIM_NODE, cs_gpios), 1);
	nrf_gpio_cfg(NRF_DT_GPIOS_TO_PSEL(SPIM_NODE, cs_gpios),
					NRF_GPIO_PIN_DIR_OUTPUT,
					NRF_GPIO_PIN_INPUT_DISCONNECT,
					NRF_GPIO_PIN_NOPULL,
					NRF_GPIO_PIN_S0S1,
					NRF_GPIO_PIN_NOSENSE);

    return;
}

void dwt_spi_set_rate_fast()
{
    PINCTRL_DT_DEFINE(SPIM_NODE);

	nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG(
		NRFX_SPIM_PIN_NOT_USED,
		NRFX_SPIM_PIN_NOT_USED,
		NRFX_SPIM_PIN_NOT_USED,
		NRF_DT_GPIOS_TO_PSEL(SPIM_NODE, cs_gpios));
	spim_config.frequency = NRF_SPIM_FREQ_8M;
	spim_config.skip_gpio_cfg = true;
	spim_config.skip_psel_cfg = true;

	pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(SPIM_NODE),
				  PINCTRL_STATE_DEFAULT);
	nrfx_spim_init(&spim, &spim_config, NULL, NULL);

    nrf_gpio_pin_write(NRF_DT_GPIOS_TO_PSEL(SPIM_NODE, cs_gpios), 1);
	nrf_gpio_cfg(NRF_DT_GPIOS_TO_PSEL(SPIM_NODE, cs_gpios),
					NRF_GPIO_PIN_DIR_OUTPUT,
					NRF_GPIO_PIN_INPUT_DISCONNECT,
					NRF_GPIO_PIN_NOPULL,
					NRF_GPIO_PIN_S0S1,
					NRF_GPIO_PIN_NOSENSE);

    return;
}
