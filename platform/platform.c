#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "deca_device_api.h"
#include "deca_regs.h"

#include "platform.h"

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

void dwt_hardinterrupt(void *callback)
{
	gpio_pin_configure_dt(&dwm_int, GPIO_INPUT);

	gpio_init_callback(&dwm_int_cb_data, callback, BIT(dwm_int.pin));
	gpio_add_callback(dwm_int.port, &dwm_int_cb_data);
}

void dwt_disable_interrupt()
{
	gpio_pin_interrupt_configure_dt(&dwm_int, GPIO_INT_DISABLE);
}
void dwt_enable_interrupt()
{
	gpio_pin_interrupt_configure_dt(&dwm_int, GPIO_INT_EDGE_TO_ACTIVE);
}

// GET RX TIMESTAMP IN 40-BIT FORMAT
uint64_t get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts = (ts << 8) + ts_tab[i];
    }
    ts &= (uint64_t)0x000000ffffffffff;
    return ts;
}

// GET TX TIMESTAMP IN 40-BIT FORMAT
uint64_t get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts = (ts << 8) + ts_tab[i];
    }
    ts &= (uint64_t)0x000000ffffffffff;
    return ts;
}

// GET TX TIMESTAMP IN 40-BIT FORMAT
uint64_t get_sys_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readsystime(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts = (ts << 8) + ts_tab[i];
    }
    ts &= (uint64_t)0x000000ffffffffff;
    return ts;
}
