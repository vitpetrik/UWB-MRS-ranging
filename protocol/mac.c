#include "mac.h"

int decode_MAC(struct mac_data_t *mac_data, const uint8_t *buffer_rx)
{
    int length = 0;

    memcpy(&mac_data->frame_ctrl, buffer_rx, sizeof(uint16_t));
    buffer_rx += sizeof(uint16_t);
    length += sizeof(uint16_t);

    // SEQUENCE NUMBER
    memcpy(&mac_data->seq_num, buffer_rx, sizeof(uint8_t));
    buffer_rx += sizeof(uint8_t);
    length += sizeof(uint8_t);

    memcpy(&mac_data->pan_id, buffer_rx, sizeof(uint16_t));
    buffer_rx += sizeof(uint16_t);
    length += sizeof(uint16_t);

    memcpy(&mac_data->destination_id, buffer_rx, sizeof(uint16_t));
    buffer_rx += sizeof(uint16_t);
    length += sizeof(uint16_t);

    memcpy(&mac_data->source_id, buffer_rx, sizeof(uint16_t));
    buffer_rx += sizeof(uint16_t);
    length += sizeof(uint16_t);

    memcpy(&mac_data->msg_type, buffer_rx, sizeof(uint8_t));
    buffer_rx += sizeof(uint8_t);
    length += sizeof(uint8_t);

    memcpy(&mac_data->tx_delay, buffer_rx, sizeof(uint32_t));
    buffer_rx += sizeof(uint32_t);
    length += sizeof(uint32_t);

    return length;
}

int encode_MAC(const struct mac_data_t *mac_data, uint8_t *buffer_tx)
{
    int frame_length = 0;

    memcpy(buffer_tx, &mac_data->frame_ctrl, sizeof(uint16_t));
    buffer_tx += sizeof(uint16_t);
    frame_length += sizeof(uint16_t);

    memcpy(buffer_tx, &mac_data->seq_num, sizeof(uint8_t));
    buffer_tx += sizeof(uint8_t);
    frame_length += sizeof(uint8_t);

    memcpy(buffer_tx, &mac_data->pan_id, sizeof(uint16_t));
    buffer_tx += sizeof(uint16_t);
    frame_length += sizeof(uint16_t);

    memcpy(buffer_tx, &mac_data->destination_id, sizeof(uint16_t));
    buffer_tx += sizeof(uint16_t);
    frame_length += sizeof(uint16_t);

    memcpy(buffer_tx, &mac_data->source_id, sizeof(uint16_t));
    buffer_tx += sizeof(uint16_t);
    frame_length += sizeof(uint16_t);

    memcpy(buffer_tx, &mac_data->msg_type, sizeof(uint8_t));
    buffer_tx += sizeof(uint8_t);
    frame_length += sizeof(uint8_t);

    memcpy(buffer_tx, &mac_data->tx_delay, sizeof(uint32_t));
    buffer_tx += sizeof(uint32_t);
    frame_length += sizeof(uint32_t);

    return frame_length;
}