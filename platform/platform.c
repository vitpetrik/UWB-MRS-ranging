#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "deca_device_api.h"

#include "platform.h"

void dwm_int_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
volatile struct gpio_callback dwm_int_cb_data;

volatile struct gpio_dt_spec dwm_int = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
	.pin = 19,
	.dt_flags = 0};

void dwt_hardreset()
{
	const struct gpio_dt_spec dwm_reset = {
		.port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
		.pin = 24,
		.dt_flags = 0};

	gpio_pin_configure_dt(&dwm_reset, GPIO_OUTPUT_HIGH);
	k_sleep(K_MSEC(10));
	gpio_pin_configure_dt(&dwm_reset, GPIO_OUTPUT_LOW);
	k_sleep(K_MSEC(10));
	gpio_pin_configure_dt(&dwm_reset, GPIO_OUTPUT_HIGH);
}

void dwt_hardinterrupt()
{
	gpio_pin_configure_dt(&dwm_int, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&dwm_int, GPIO_INT_EDGE_TO_ACTIVE);

	gpio_init_callback(&dwm_int_cb_data, dwm_int_callback, BIT(dwm_int.pin));
	gpio_add_callback(dwm_int.port, &dwm_int_cb_data);
}

void dwm_int_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	dwt_isr();
}