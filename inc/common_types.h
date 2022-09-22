#ifndef __COMMON_TYPES_H__
#define __COMMON_TYPES_H__

#include <zephyr/zephyr.h>
#include "data_msg.pb.h"

#include <pb_encode.h>
#include <pb_decode.h>
#include "deca_device_api.h"

typedef enum
{
    BEACON = 0,
    DATA,
    ACK,
    MAC_COMMAND,
    MRS_BEACON,
    RESERVED_2,
    RESERVED_3,
    RESERVED_4,
} frame_type_t;

struct rx_queue_t
{
    const dwt_cb_data_t *cb_data;
    uint8_t *buffer_rx;
    uint64_t rx_timestamp;
    int32_t carrier_integrator;
};

struct tx_details_t
{
    int ranging;
    uint8_t tx_mode;
    uint32_t tx_delay;
    uint64_t *tx_timestamp;
};

struct tx_queue_t
{
    uint16_t frame_length;
    uint8_t *frame_buffer;
    struct tx_details_t tx_details;
};

// DATA STRUCTURE DEFINITIONS
struct ranging_t
{
    double distance;
    double delay;
    uint64_t tx_timestamp;
    uint64_t rx_timestamp;
};

struct device_t
{
    uint16_t id;
    UAV_TYPE uav_type;
    double GPS[2];
    struct ranging_t ranging;
};

#endif