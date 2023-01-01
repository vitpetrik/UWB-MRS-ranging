#ifndef __COMMON_TYPES_H__
#define __COMMON_TYPES_H__

#include <zephyr/zephyr.h>
#include "deca_device_api.h"

#include "mac.h"
#include "statistics.h"

#define PACKED __attribute__((__packed__))

typedef enum {
    UNDETERMINED,
    STANDALONE,
    ROS
} control_t;

struct mrs_ranging_t {
    control_t control;
    dwt_config_t dwt_config;

    uint16_t L2_address;
    uint16_t PAN_ID;
};

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
    float rx_power;
};

struct rx_queue_t
{
    uint8_t buf_offset;
    struct rx_details_t rx_details;
    struct mac_data_t mac_data;
    uint16_t frame_length;
    uint8_t *buffer_rx;
}__attribute__((aligned(4)));

struct tx_delay_t
{
    uint64_t rx_timestamp;
    uint32_t reserved_time;
    int offset;
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
    struct tx_details_t tx_details;
    struct mac_data_t mac_data;
    uint16_t frame_length;
    uint8_t *frame_buffer;
}__attribute__((aligned(4)));

// DATA STRUCTURE DEFINITIONS

struct ranging_msg_t {
    uint16_t source_mac;

    float range;
    float variance;

    float raw;
};

struct uwb_data_msg_t {
    uint16_t source_mac;
    uint16_t destination_mac;

    uint8_t msg_type;
    uint8_t payload_size;
    uint8_t *payload;
};

enum {
    RANGING_DATA,
    UWB_DATA
};

struct uwb_msg_t {
    uint8_t msg_type;

    union {
        struct ranging_msg_t ranging_msg;
        struct uwb_data_msg_t uwb_data_msg;
    } data;
}__attribute__((aligned(4)));

#endif