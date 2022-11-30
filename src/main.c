/**
 * @file main.c
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief Main entry point to UWB positioning sytem
 * @version 0.1
 * @date 2022-11-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <zephyr/zephyr.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <timing/timing.h>
#include <string.h>

#include <stdio.h>

#include "deca_regs.h"
#include "platform.h"

#include "common_types.h"
#include "common_macro.h"
#include "common_variables.h"
#include "uwb_threads.h"
#include "ranging.h"

#include "mac.h"
#include "uart.h"
#include "ros.h"

/* Example application name and version to display on LCD screen. */
#define APP_NAME "MRS UWB v0.1"
const char *buildString = "Build was compiled at " __DATE__ ", " __TIME__ ".";

uint16_t DEVICE_ID;
uint16_t PAN_ID;
uint8_t SEQ_NUM;

// UART

#define UART_DEVICE_NODE DT_NODELABEL(uart0)
struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

// LEDS

static const struct gpio_dt_spec led0red = GPIO_DT_SPEC_GET(DT_NODELABEL(led0_red), gpios);
static const struct gpio_dt_spec led1green = GPIO_DT_SPEC_GET(DT_NODELABEL(led1_green), gpios);
static const struct gpio_dt_spec led2red = GPIO_DT_SPEC_GET(DT_NODELABEL(led2_red), gpios);
static const struct gpio_dt_spec led3blue = GPIO_DT_SPEC_GET(DT_NODELABEL(led3_blue), gpios);

/* Default communication configuration. We use here EVK1000's mode 4. See NOTE 1 below. */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (128 + 8 + 1)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

K_MSGQ_DEFINE(uwb_msgq, sizeof(struct uwb_msg_t), 10, 4);

// Threads declarations

/**
 * @brief Receive and process data from ROS (or PC)
 *
 */
void ros_rx_thread(void);
void ros_tx_thread(void);

// Threads definitions

K_THREAD_DEFINE(ros_rx_thr, 1024, ros_rx_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(ros_tx_thr, 1024, ros_tx_thread, NULL, NULL, NULL, 5, 0, 0);

K_THREAD_DEFINE(uwb_beacon_thr, 1024, uwb_beacon_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(uwb_ranging_thr, 1024, uwb_ranging_thread, NULL, NULL, NULL, 5, 0, 0);

K_THREAD_DEFINE(uwb_ranging_print_thr, 1024, uwb_ranging_print_thread, NULL, NULL, NULL, 7, 0, 0);

K_THREAD_DEFINE(ranging_thr, 1024, ranging_thread, NULL, NULL, NULL, -3, 0, 0);
K_THREAD_DEFINE(uwb_tx_thr, 1024, uwb_tx_thread, NULL, NULL, NULL, -2, 0, 0);

K_THREAD_DEFINE(dwt_isr_thr, 1024, dwt_isr_thread, NULL, NULL, NULL, -1, 0, 0);

void main(void)
{
    // INITIALIZE
    printf("%s\n\r", APP_NAME);
    printf("%s\n\r", buildString);

    uart_setup();

    DEVICE_ID = NRF_FICR->DEVICEID[0] >> 16;
    PAN_ID = 0xdeca;
    SEQ_NUM = 0;

    printf("Device ID: 0x%X\n\r", DEVICE_ID);

    // Initiliaze LEDs GPIO
    gpio_pin_configure_dt(&led0red, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led1green, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led2red, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led3blue, GPIO_OUTPUT_INACTIVE);

    // Get DWT Mutex
    k_mutex_init(&dwt_mutex);
    k_mutex_lock(&dwt_mutex, K_FOREVER);

    // Reset DWT to delete previous config
    dwt_hardreset();
    dwt_hardinterrupt(dwm_int_callback);

    // The SPI need to be set to lower speed setting
    dwt_spi_set_rate_slow();

    __ASSERT_NO_MSG(dwt_initialise(DWT_LOADUCODE) == DWT_SUCCESS);

    // Now, we can enable full speed on SPI bus
    dwt_spi_set_rate_fast();

    // Set our configuration
    dwt_configure(&config);

    // Set transimtting power
    //! Need later revision
    dwt_txconfig_t txconfig = {.PGdly = 0xC0, .power = 0x25466788};
    dwt_configuretxrf(&txconfig);
    dwt_setsmarttxpower(true);

    dwt_setleds(DWT_LEDS_ENABLE);

    //? Might need to be changed from ROS
    dwt_setaddress16(DEVICE_ID);
    dwt_setpanid(PAN_ID);

    // Enable frame filtering and rejection
    dwt_enableframefilter(DWT_FF_DATA_EN, true);
    dwt_setdblrxbuffmode(1);

    //? Experimentaly set antenna delays
    //? Not that much accurate
    dwt_settxantennadelay(21875);
    dwt_setrxantennadelay(21875);

    dwt_setcallbacks(uwb_txdone, uwb_rxok, uwb_rxto, uwb_rxerr, uwb_rxfrej);

    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG, 1);
    dwt_enable_interrupt();

    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    k_mutex_unlock(&dwt_mutex);

    printf("Settings done!\n\r");

    //? Heart beat LED
    //? Good to recognize bad things happening.. SEG FAULT etc.

    while (1)
    {
        gpio_pin_set_dt(&led1green, 1);
        sleep_ms(200);
        gpio_pin_set_dt(&led1green, 0);
        sleep_ms(200);
        gpio_pin_set_dt(&led1green, 1);
        sleep_ms(200);
        gpio_pin_set_dt(&led1green, 0);
        sleep_ms(600);

        struct uwb_msg_t uwb_msg;

        uwb_msg.msg_type = RANGING_DATA;
        uwb_msg.data.ranging_msg.source_mac = 0x69;
        uwb_msg.data.ranging_msg.range = 69.0F;
        uwb_msg.data.ranging_msg.variance = 255.0F;

        k_msgq_put(&uwb_msgq, &uwb_msg, K_FOREVER);
    }

    return;
}

void ros_tx_thread(void)
{
    struct baca_protocol msg;
    struct ros_msg_t ros;

    struct uwb_msg_t uwb_msg;

    while(1)
    {
        int status = k_msgq_get(&uwb_msgq, (void*) &uwb_msg, K_FOREVER);

        if(status != 0)
            continue;

        switch (uwb_msg.msg_type)
        {
        case RANGING_DATA:
            ros.mode = 'a';
            ros.address = RANGING_RESULT;

            ros.data.ranging_msg = uwb_msg.data.ranging_msg;
            break;
        case UWB_DATA:
            ros.mode = 'a';
            ros.address = TRX_DATA;

            ros.data.uwb_data_msg = uwb_msg.data.uwb_data_msg;
            break;
        default:
            break;
        }

        msg.payload_size = serialize_ros(&ros, msg.payload);
        write_baca(&msg);
    }
}

/**
 * @brief Receive and process data from ROS
 *
 */
void ros_rx_thread(void)
{
    struct baca_protocol msg;
    struct ros_msg_t ros;

    while (1)
    {
        int payload_length = read_baca(&msg);

        deserialize_ros(&ros, &msg.payload);

        if (ros.address == WHO_I_AM && ros.mode == 'r')
        {
            ros.mode = 'a';

            memcpy (&ros.data.id_msg.id, APP_NAME, sizeof(APP_NAME));

            msg.payload_size = serialize_ros(&ros, msg.payload);
            write_baca(&msg);
        }
    }
}
