#include "mac.h"

int decode_MAC(struct mac_data_t *mac_data, const uint8_t *buffer_rx)
{
    memcpy(&mac_data->frame_ctrl, buffer_rx, sizeof(uint16_t));
    buffer_rx += sizeof(uint16_t);

    // SEQUENCE NUMBER
    memcpy(&mac_data->seq_num, buffer_rx, sizeof(uint8_t));
    buffer_rx += sizeof(uint8_t);

    memcpy(&mac_data->pan_id, buffer_rx, sizeof(uint16_t));
    buffer_rx += sizeof(uint16_t);

    memcpy(&mac_data->destination_id, buffer_rx, sizeof(uint16_t));
    buffer_rx += sizeof(uint16_t);

    memcpy(&mac_data->source_id, buffer_rx, sizeof(uint16_t));
    buffer_rx += sizeof(uint16_t);

    memcpy(&mac_data->msg_type, buffer_rx, sizeof(uint8_t));
    buffer_rx += sizeof(uint8_t);

    return ENCODED_MAC_LENGTH;
}

int encode_MAC(const struct mac_data_t *mac_data, uint8_t *buffer_tx)
{
    memcpy(buffer_tx, &mac_data->frame_ctrl, sizeof(uint16_t));
    buffer_tx += sizeof(uint16_t);

    memcpy(buffer_tx, &mac_data->seq_num, sizeof(uint8_t));
    buffer_tx += sizeof(uint8_t);

    memcpy(buffer_tx, &mac_data->pan_id, sizeof(uint16_t));
    buffer_tx += sizeof(uint16_t);

    memcpy(buffer_tx, &mac_data->destination_id, sizeof(uint16_t));
    buffer_tx += sizeof(uint16_t);

    memcpy(buffer_tx, &mac_data->source_id, sizeof(uint16_t));
    buffer_tx += sizeof(uint16_t);

    memcpy(buffer_tx, &mac_data->msg_type, sizeof(uint8_t));
    buffer_tx += sizeof(uint8_t);

    return ENCODED_MAC_LENGTH;
}