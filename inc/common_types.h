#ifndef __COMMON_TYPES_H__
#define __COMMON_TYPES_H__

#include <zephyr/zephyr.h>
#include "data_msg.pb.h"

#include <pb_encode.h>
#include <pb_decode.h>
#include "deca_device_api.h"

#include "mac.h"
#include "statistics.h"

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
struct ranging_t
{
    uint8_t new_data;
    uint64_t tx_timestamp;
    uint64_t rx_timestamp;
    uint32_t last_meas_time;

    struct statistics_t stats;
};

struct device_t
{
    uint16_t id;
    uint8_t uav_type;
    double GPS[2];
    struct ranging_t ranging;
};

struct ranging_msg_t {
    uint16_t source_mac;

    float range;
    float variance;
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