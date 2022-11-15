/**
 * @file uwb_transport.h
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief Transport layer for UWB
 * @version 0.1
 * @date 2022-11-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __UWB_TRANSPORT_H__
#define __UWB_TRANSPORT_H__

#include <zephyr/zephyr.h>
#include "common_types.h"
#include "ranging.h"
#include "mac.h"

#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

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
void tx_message(const uint16_t destination_id, const frame_type_t frame_type, const int msg_type, const uint8_t *msg, int len, struct tx_details_t *tx_details);

#ifdef __cplusplus
}
#endif

#endif