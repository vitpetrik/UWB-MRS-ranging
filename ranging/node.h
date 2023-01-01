/**
 * @file node.h
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief Node struct thats mandatory for ranging
 * @version 0.1
 * @date 2022-12-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __NODE_H__
#define __NODE_H__

#include <stdint.h>
#include "statistics.h"

typedef enum
{
    INITIATOR,
    RESPONDER
} participant_t;

typedef enum
{
    ENABLED,
    DISABLED
} ranging_state_t;

struct ranging_t
{
    ranging_state_t ranging_state;
    participant_t role;
    struct statistics_t stats;
    uint8_t expected_packet_number;
    int error_counter;
    uint32_t last_meas_time;
    uint64_t tx_timestamp;
};

struct node_t
{
    uint16_t id;
    struct ranging_t ranging;
};


#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initialize Node struct
     * 
     * @param node pointer to node_t
     * @param id Id of node
     * @param window_size Windows size for averaging
     */
    void node_init(struct node_t *node, uint16_t id, int window_size);

    /**
     * @brief Reset node
     * 
     * @param node pointer to node_t
     */
    void node_reset(struct node_t *node);

    /**
     * @brief Destroy node_t object
     * 
     * @param node pointer to node_t
     */
    void node_destroy(struct node_t *node);

#ifdef __cplusplus
}
#endif

#endif