#ifndef __MAC_H__
#define __MAC_H__

#include <zephyr/zephyr.h>

#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

#define ENCODED_MAC_LENGTH 10

struct mac_data_t {
    uint16_t frame_ctrl;
    uint8_t seq_num;
    uint16_t pan_id;
    uint16_t destination_id;
    uint16_t source_id;
    uint8_t msg_type;
};

int decode_MAC(struct mac_data_t *mac_data, const uint8_t *buffer_rx);

int encode_MAC(const struct mac_data_t *mac_data, uint8_t *buffer_tx);

#ifdef __cplusplus
}
#endif

#endif

