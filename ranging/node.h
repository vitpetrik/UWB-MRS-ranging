#ifndef __NODE_H__
#define __NODE_H__

#include <stdint.h>
#include "statistics.h"

#define AVERAGE_WINDOW 20

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

void node_init(struct node_t *node, uint16_t id, int window_size);

void node_reset(struct node_t *node);

void node_destroy(struct node_t *node);

#ifdef __cplusplus
}
#endif

#endif