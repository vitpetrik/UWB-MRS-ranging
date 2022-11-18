/**
 * @file uwb_transport.c
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief Transport layer for UWB
 * @version 0.1
 * @date 2022-11-15
 *
 * @copyright Copyright (c) 2022
 * 
 */

#include "uwb_transport.h"
#include "common_variables.h"

/**
 * @brief Transport data to tx queue
 *
 * @param destination_id id of destinatio uwb
 * @param frame_type Type of frame DATA/BEACON...
 * @param msg_type Type of message
 * @param msg pointer to message data
 * @param len length of the message data
 * @param tx_details details of the transission e.g timestamp...
 */
void write_uwb(const uint16_t destination_id, const frame_type_t frame_type, const int msg_type, const uint8_t *msg, int len, struct tx_details_t *tx_details)
{
    // ALLOCATE MEMORY FOR tx BUFFER AND QUEUE DATA
    uint8_t *buffer_tx = (uint8_t *)k_calloc(ENCODED_MAC_LENGTH + len, sizeof(uint8_t));
    __ASSERT_NO_MSG(buffer_tx != NULL);

    struct tx_queue_t *queue_data = (struct tx_queue_t *)k_calloc(1, sizeof(struct tx_queue_t));
    __ASSERT_NO_MSG(queue_data != NULL);

    struct mac_data_t mac_data = {
        .frame_ctrl = 0x9840 | frame_type,
        .seq_num = SEQ_NUM++,
        .pan_id = PAN_ID,
        .destination_id = destination_id,
        .source_id = DEVICE_ID,
        .msg_type = msg_type,
        .tx_delay = 0,
    };

    memcpy(&buffer_tx[ENCODED_MAC_LENGTH], msg, len);

    // INITIALIZE QUEUE DATA
    queue_data->frame_buffer = buffer_tx;
    queue_data->frame_length = len;
    queue_data->tx_details = *tx_details;
    queue_data->mac_data = mac_data;

    // SEND DATA TO TX QUEUE
    k_fifo_alloc_put(&uwb_tx_fifo, queue_data);
}


struct rx_queue_t* read_uwb()
{
    struct rx_queue_t *data = (struct rx_queue_t *)k_fifo_get(&uwb_rx_fifo, K_FOREVER);

    return data;
}