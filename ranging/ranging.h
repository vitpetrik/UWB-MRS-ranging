/**
 * @file ranging.h
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief Ranging Related function
 * @version 0.1
 * @date 2022-09-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __RANGING_H__
#define __RANGING_H__

#include <zephyr/zephyr.h>
#include <string.h>

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 �s and 1 �s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 64267

#define SPEED_OF_LIGHT 299702547.0

enum
{
    SUCCESS,
    RX_ENABLE
};

#define RANGING_MSG_TYPE 0x10


#include "common_types.h"
#include "deca_device_api.h"

#define ENCODED_RANGING_PKT_LENGTH 13

struct ranging_pkt_t {
    uint8_t packet_number;
    uint32_t RoundA;
    uint32_t DelayA;
    uint32_t RoundB;
    uint32_t DelayB;
};

#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Decode ranging buffer
     * 
     * @param ranging_pkt pointer to ranging paket
     * @param buffer_rx pointer to received buffer
     * @return int returns length of message
     */
    int decode_ranging_pkt(struct ranging_pkt_t *ranging_pkt, const uint8_t *buffer_rx);

    /**
     * @brief 
     * 
     * @param ranging_pkt pointer to ranging paket
     * @param buffer_tx pointer to transmit buffer
     * @return int returns length of message
     */
    int encode_ranging_pkt(const struct ranging_pkt_t *ranging_pkt, uint8_t *buffer_tx);

    /**
     * @brief Sends ranging request to target_id
     * 
     * @param target_id MAC address of target node
     * @param preprocessing Size of averaging windows
     */
    void request_ranging(uint16_t target_id, int preprocessing);

    /**
     * @brief Disable ranging for target_id
     * 
     * @param target_id MAC address of target node
     */
    void disable_ranging(uint16_t target_id);

    /**
     * @brief Function to handle Double sided ranging
     * 
     * @param source_id MAC address of received packet
     * @param msg received buffer
     * @param rx_details Details about received packet
     * @return int status (Not actually used)
     */
    int rx_ranging_ds(const uint16_t source_id, void *msg, struct rx_details_t *queue_data);

    /**
     * @brief Ranging thread periodically checks status of nodes and request ranging if packet lost happend
     */
    void uwb_ranging_thread(void);


#ifdef __cplusplus
}
#endif

#endif