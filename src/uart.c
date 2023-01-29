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

#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include "uart.h"
#include <string.h>

#include <nrfx_uarte.h>
#include <nrfx_gpiote.h>

#include <stdio.h>

#include "common_variables.h"
#include "common_macro.h"

/** Position of the pin field. */
#define NRF_PIN_POS 0U
/** Mask for the pin field. */
#define NRF_PIN_MSK 0x7FU

#define NRF_PINSEL(port, pin)						       \
	(((((port) * 32U) + (pin)) & NRF_PIN_MSK) << NRF_PIN_POS)

K_SEM_DEFINE(rx_sem, 0, 1);
K_SEM_DEFINE(tx_sem, 1, 1);

/** @brief Symbol specifying UARTE instance to be used. */
#define UARTE_INST_IDX 0

/** @brief Symbol specifying TX pin number of UARTE. */
#define UARTE_TX_PIN NRF_PINSEL(0, 5)

/** @brief Symbol specifying RX pin number of UARTE. */
#define UARTE_RX_PIN NRF_PINSEL(0, 11)

#define UARTE_INST         NRFX_CONCAT_2(NRF_UARTE, UARTE_INST_IDX)
#define UARTE_INST_HANDLER NRFX_CONCAT_3(nrfx_uarte_, UARTE_INST_IDX, _irq_handler)

nrfx_uarte_t uarte_inst = NRFX_UARTE_INSTANCE(UARTE_INST_IDX);
nrfx_uarte_config_t uarte_config = NRFX_UARTE_DEFAULT_CONFIG(UARTE_TX_PIN, UARTE_RX_PIN);

/**
 * @brief Function for handling UARTE driver events.
 *
 * @param[in] p_event   Pointer to event structure. Event is allocated on the stack so it is available
 *                      only within the context of the event handler.
 * @param[in] p_context Context passed to the interrupt handler, set on initialization. In this example
 *                      p_context is used to pass the address of the UARTE instance that calls this handler.
 */
static void uarte_handler(nrfx_uarte_event_t const * p_event, void * p_context)
{
    nrfx_uarte_t * p_inst = p_context;
    if (p_event->type == NRFX_UARTE_EVT_TX_DONE)
    {
        k_sem_give(&tx_sem);
    }
    else if (p_event->type == NRFX_UARTE_EVT_RX_DONE)
    {
        k_sem_give(&rx_sem);
    }
}

/**
 * @brief Setup UART peripherial
 * 
 */
void uart_setup()
{
    nrfx_err_t status;
    (void)status;

    uarte_config.baudrate = NRF_UARTE_BAUDRATE_115200;
    uarte_config.p_context = &uarte_inst;
    uarte_config.skip_gpio_cfg = false;
    status = nrfx_uarte_init(&uarte_inst, &uarte_config, uarte_handler);
    nrf_gpio_cfg_input(uarte_config.pselrxd, NRF_GPIO_PIN_PULLUP);

    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(UARTE_INST), 0, UARTE_INST_HANDLER, NULL, 0);

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
    int status;

    msg->cksum = 0;
    msg->payload_size = 0;

    uint8_t c = '\0';

    while(c != 'b')
        c = read_char();

    msg->cksum = c;

    status = read_buffer(&msg->payload_size, sizeof(msg->payload_size), K_MSEC(10));

    if(status != 0)
        return -1;

    msg->cksum += msg->payload_size;

    if (msg->payload_size == 0 || msg->payload_size > 255)
        return -1;

    msg->payload = k_malloc(sizeof(uint8_t)*msg->payload_size);

    __ASSERT(msg->payload != NULL, "Failed allocating buffer of size %d Bytes!", msg->payload_size);

    status = read_buffer(msg->payload, msg->payload_size, K_MSEC(10));

    if(status != 0)
    {
        k_free(msg->payload);
        return -1;
    }

    msg->cksum += calc_cksum(msg->payload, msg->payload_size);

    uint8_t rx_cksum;
    status = read_buffer(&rx_cksum, sizeof(rx_cksum), K_MSEC(10));

    if(status != 0 || rx_cksum != msg->cksum)
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
    write_buffer(&c, 1);
    return;
}

/**
 * @brief Write buffer to UART
 * 
 * @param buffer pointer to buffer that needs to be written out
 * @param len length of the data
 */
void write_buffer(uint8_t *buffer, int len)
{
    k_sem_take(&tx_sem, K_FOREVER);
    int status = nrfx_uarte_tx(&uarte_inst, buffer, len);
    __ASSERT(status == NRFX_SUCCESS, "UARTE write fail!");

    status = k_sem_take(&tx_sem, K_MSEC(10));

    if(status != 0)
    {
        nrfx_uarte_tx_abort(&uarte_inst);
    }

    k_sem_give(&tx_sem);
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

    read_buffer(&c, 1);

    return c;
}

/**
 * @brief Read bigger chunk of data to buffer
 * 
 * @param buf pointer to buffer
 * @param len length of the data to be received
 */
int read_buffer(uint8_t *buf, int len, k_timeout_t timeout)
{
    int status = nrfx_uarte_rx(&uarte_inst, buf, len);
    __ASSERT(status == NRFX_SUCCESS, "UARTE read failed!");

    status = k_sem_take(&rx_sem, timeout);

    return status;
}
