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

#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/timing/timing.h>

#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
LOG_MODULE_REGISTER(main, 2);

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <string.h>
#include <stdbool.h>

#include <stdio.h>

#include "deca_regs.h"
#include "platform.h"

#include "common_types.h"
#include "common_macro.h"
#include "common_variables.h"
#include "uwb_threads.h"
#include "uwb_transport.h"
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

struct mrs_ranging_t mrs_ranging;

// UART

#define UART_DEVICE_NODE DT_NODELABEL(uart0)
struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

// LEDS

static const struct gpio_dt_spec led0red = GPIO_DT_SPEC_GET(DT_NODELABEL(led0_red), gpios);
static const struct gpio_dt_spec led1green = GPIO_DT_SPEC_GET(DT_NODELABEL(led1_green), gpios);
static const struct gpio_dt_spec led2red = GPIO_DT_SPEC_GET(DT_NODELABEL(led2_red), gpios);
static const struct gpio_dt_spec led3blue = GPIO_DT_SPEC_GET(DT_NODELABEL(led3_blue), gpios);

// BUTTONS

#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct gpio_callback button_cb_data;

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

void uwb_multiplex_rx();

// Threads definitions

K_THREAD_DEFINE(ros_rx_thr, 1024, ros_rx_thread, NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(ros_tx_thr, 1024, ros_tx_thread, NULL, NULL, NULL, 6, 0, 0);

K_THREAD_DEFINE(uwb_ranging_thr, 1024, uwb_ranging_thread, NULL, NULL, NULL, 5, 0, 0);

// K_THREAD_DEFINE(ranging_thr, 1024, ranging_thread, NULL, NULL, NULL, -3, 0, 0);
K_THREAD_DEFINE(uwb_multiplex_thr, 1024, uwb_multiplex_rx, NULL, NULL, NULL, -3, 0, 0);
K_THREAD_DEFINE(uwb_tx_thr, 1024, uwb_tx_thread, NULL, NULL, NULL, -2, 0, 0);

// CALLBACKS

void send_anchor_beacon_cb(struct k_timer *timer);
K_TIMER_DEFINE(send_anchor_beacon_timer, send_anchor_beacon_cb, NULL);

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

/**
 * @brief Set up the DW1000 tranciever
 * 
 */
void setup_dwt()
{
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
    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_MAC_EN, true);
    dwt_setdblrxbuffmode(1);

    //? Experimentaly set antenna delays
    //? Not that much accurate
    dwt_settxantennadelay(21875);
    dwt_setrxantennadelay(21875);

    dwt_setcallbacks(uwb_txdone, uwb_rxok, uwb_rxto, uwb_rxerr, uwb_rxfrej);

    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RXOVRR | DWT_INT_RFSL | DWT_INT_RFCE | DWT_INT_RPHE, 1);
    dwt_enable_interrupt();

    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    k_mutex_unlock(&dwt_mutex);

    LOG_INF("Settings done!");
}

void main(void)
{
    // INITIALIZE
    LOG_INF("%s", APP_NAME);
    LOG_INF("%s", buildString);

    timing_init();
    timing_start();

    // Initiliaze LEDs GPIO
    gpio_pin_configure_dt(&led0red, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led1green, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led2red, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led3blue, GPIO_OUTPUT_INACTIVE);

    // Set button interrupt
    gpio_pin_configure_dt(&button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

    // Set UART communications for ROS 
    uart_setup();

    // Set global state data
    mrs_ranging.control == UNDETERMINED;
    mrs_ranging.dwt_config = config;

    // Wait loop to determine whether to use Standalone or ROS mode
    // It's dead simple but works as intended
    for (int i = 0; i < 14; i++)
    {
        if (mrs_ranging.control != UNDETERMINED)
            break;

        gpio_pin_toggle_dt(&led3blue);
        sleep_ms(400);
    }

    // make sure the blue LED is switched OFF
    gpio_pin_set_dt(&led3blue, 0);

    // If the user did not pressed the button
    // then set ROS mode
    if (mrs_ranging.control == UNDETERMINED)
        mrs_ranging.control = ROS;

    LOG_INF("Selected mode: %s", mrs_ranging.control == ROS ? "ROS" : "STANDALONE");

    mrs_ranging.L2_address = NRF_FICR->DEVICEID[0] >> 16;
    mrs_ranging.PAN_ID = 0xabcd;

    DEVICE_ID = NRF_FICR->DEVICEID[0] >> 16;
    PAN_ID = 0xabcd;
    SEQ_NUM = 0;

    setup_dwt();

    // Start threads based on control mode
    if (mrs_ranging.control == STANDALONE)
    {
        k_thread_abort(uwb_ranging_thr);
        k_thread_abort(ros_tx_thr);

        k_timer_start(&send_anchor_beacon_timer, K_SECONDS(0), K_SECONDS(10));
    }
    else
    {
        k_thread_resume(uwb_ranging_thr);
        k_thread_resume(ros_tx_thr);
    }

    k_thread_resume(ros_rx_thr);
    k_thread_resume(uwb_multiplex_thr);
    k_thread_resume(uwb_tx_thr);

    LOG_INF("Device ID: 0x%X", DEVICE_ID);

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
    }

    return;
}

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (mrs_ranging.control == UNDETERMINED)
        mrs_ranging.control = STANDALONE;

    return;
}

void uwb_multiplex_rx()
{
    k_tid_t thread = k_current_get();
    k_thread_suspend(thread);

    int status;

    LOG_INF("UWB multiplex thread started");
    struct rx_queue_t data;

    struct uwb_msg_t msg;

    while (1)
    {
        // wait for data
        read_uwb(&data);

        int source_id = data.mac_data.source_id;
        struct rx_details_t *rx_details = &data.rx_details;

        void *offseted_buf = &data.buffer_rx[data.buf_offset];

        LOG_HEXDUMP_DBG(offseted_buf, data.frame_length, "Received data from UWB");

        switch (data.mac_data.msg_type)
        {
        case RANGING_TYPE:
            // Handle only ranging data
            rx_ranging_ds(source_id, offseted_buf, rx_details);
            break;
        default:
            if(mrs_ranging.control == STANDALONE)
                break;
            // If generic data is received than send it over UART to ROS
            msg.msg_type = UWB_DATA;
            msg.data.uwb_data_msg.source_mac = source_id;
            msg.data.uwb_data_msg.destination_mac = data.mac_data.destination_id;
            msg.data.uwb_data_msg.msg_type = data.mac_data.msg_type;
            msg.data.uwb_data_msg.payload_size = data.frame_length;

            msg.data.uwb_data_msg.payload = k_malloc(msg.data.uwb_data_msg.payload_size);
            __ASSERT(msg.data.uwb_data_msg.payload != NULL, "Failed to allocate buffer");

            memcpy(msg.data.uwb_data_msg.payload, offseted_buf, msg.data.uwb_data_msg.payload_size);

            status = k_msgq_put(&uwb_msgq, &msg, K_FOREVER);
            __ASSERT(status == 0, "Putting message to queue Failed!");

            break;
        }

        k_free(data.buffer_rx);
    }
}

void send_anchor_beacon(struct k_work *work)
{
    // if in standalone mode send Anchor data
    struct tx_details_t tx_details = {
        .ranging = 0,
        .tx_mode = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED,
        .tx_timestamp = NULL};

    struct anchor_msg_t anchor_msg;

    anchor_msg.address = ANCHOR_BEACON;
    anchor_msg.mode = 'w';
    anchor_msg.data.anchor_beacon.capabilities = RDEV;

    uint8_t buffer[sizeof(struct anchor_msg_t)];

    int msg_length = serialize_anchor_msg(&anchor_msg, buffer);

    LOG_HEXDUMP_DBG(buffer, msg_length, "Sending anchor packet");

    write_uwb(0xffff, DATA, ANCHOR_TYPE, buffer, msg_length, &tx_details);

    return;
}

K_WORK_DEFINE(send_anchor_work, send_anchor_beacon);

void send_anchor_beacon_cb(struct k_timer *timer)
{
    k_work_submit(&send_anchor_work);
    return;
}

void ros_tx_thread(void)
{
    struct baca_protocol msg_tx;
    struct ros_msg_t ros;

    struct uwb_msg_t uwb_msg;

    msg_tx.payload = k_malloc(256);
    __ASSERT(msg_tx.payload != NULL, "Failed to allocate buffer!");

    while (1)
    {
        // Get data from UWB
        int status = k_msgq_get(&uwb_msgq, (void *)&uwb_msg, K_FOREVER);
        gpio_pin_set_dt(&led2red, 1);

        if (status != 0)
            continue;

        // Setup message
        switch (uwb_msg.msg_type)
        {
        case RANGING_DATA:
            ros.address = RANGING_RESULT;
            ros.mode = 'w';

            ros.data.ranging_msg = uwb_msg.data.ranging_msg;
            break;
        case UWB_DATA:
            ros.address = TRX_DATA;
            ros.mode = 'w';

            ros.data.uwb_data_msg = uwb_msg.data.uwb_data_msg;
            break;
        default:
            break;
        }

        // Serialize and send the data
        msg_tx.payload_size = serialize_ros(&ros, msg_tx.payload);
        write_baca(&msg_tx);

        gpio_pin_set_dt(&led2red, 0);
    }

    k_free(msg_tx.payload);
}

/**
 * @brief Receive and process data from ROS
 *
 */
void ros_rx_thread(void)
{
    struct baca_protocol msg_rx;
    struct baca_protocol msg_tx;
    struct ros_msg_t ros;

    msg_tx.payload = k_malloc(256);
    bool request_send;

    struct tx_details_t tx_details = {
        .ranging = 0,
        .tx_mode = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED,
        .tx_timestamp = NULL};

    while (1)
    {
        // Read data from UART send over BACA
        int payload_length = read_baca(&msg_rx);

        if (payload_length < 0)
        {
            LOG_INF("Received wrong checksum! Discarding the data");
            continue;
        }

        LOG_HEXDUMP_DBG(msg_rx.payload, payload_length, "Received buffer from ROS");

        gpio_pin_set_dt(&led3blue, 1);
        deserialize_ros(&ros, msg_rx.payload);

        k_free(msg_rx.payload);

        LOG_DBG("Ros command at address: 0x%X", ros.address);
        request_send = false;

        // handle the data based on address
        switch (ros.address)
        {
        case WHO_I_AM:
            if (ros.mode != 'r')
                break;
            LOG_DBG("Received WHO_I_AM request");
            ros.mode = 'w';
            memcpy(ros.data.id_msg.id, APP_NAME, sizeof(APP_NAME));
            request_send = true;
            break;
        case TRX_DATA:
            LOG_HEXDUMP_DBG(ros.data.uwb_data_msg.payload, ros.data.uwb_data_msg.payload_size, "Sending TRX data");
            write_uwb(ros.data.uwb_data_msg.destination_mac,
                      DATA,
                      ros.data.uwb_data_msg.msg_type,
                      ros.data.uwb_data_msg.payload,
                      ros.data.uwb_data_msg.payload_size,
                      &tx_details);

            k_free(ros.data.uwb_data_msg.payload);
            break;
        case RESET:
            log_panic();
            LOG_INF("Rebooting system on request from ROS");
            sys_reboot(0);
            break;
        case ROS_CONTROL:
            if (mrs_ranging.control == UNDETERMINED)
                mrs_ranging.control = ros.data.control;
            break;
        case REQUEST_RANGING:
            request_ranging(ros.data.request_ranging.target_id, ros.data.request_ranging.preprocessing);
            break;
        default:
            break;
        }

        gpio_pin_set_dt(&led3blue, 0);

        if (!request_send)
            continue;

        msg_tx.payload_size = serialize_ros(&ros, msg_tx.payload);
        write_baca(&msg_tx);

        LOG_HEXDUMP_DBG(msg_tx.payload, msg_tx.payload_size, "Tx to ROS");
    }
}
