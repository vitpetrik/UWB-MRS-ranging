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

struct rx_details_t
{
    uint64_t rx_timestamp;
    int32_t carrier_integrator;
    uint32_t tx_delay;
};

struct mac_data_t {
    uint16_t frame_ctrl;
    uint8_t seq_num;
    uint16_t pan_id;
    uint16_t destination_id;
    uint16_t source_id;
    uint8_t msg_type;
    uint32_t tx_delay;
};

struct rx_queue_t
{
    uint8_t *buffer_rx;
    uint8_t *buffer_rx_free_ptr;
    struct rx_details_t rx_details;
    struct mac_data_t mac_data;
};

struct tx_delay_t
{
    uint64_t rx_timestamp;
    uint32_t reserved_time;
};

struct tx_details_t
{
    int ranging;
    uint8_t tx_mode;
    uint64_t *tx_timestamp;
    struct tx_delay_t tx_delay;
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
    uint32_t counter;
};

struct device_t
{
    uint16_t id;
    uint8_t uav_type;
    double GPS[2];
    struct ranging_t ranging;
};

#endif