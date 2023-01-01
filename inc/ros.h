/**
 * @file ros.h
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief Definitions of structures and functions for ROS communication
 * @version 0.1
 * @date 2022-11-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __ROS_H__
#define __ROS_H__

#include "stdio.h"

#include "common_types.h"
#include "ranging.h"

enum PACKET_MSG_TYPES
{
    ROS_TYPE,
    ANCHOR_TYPE,
    RANGING_TYPE = RANGING_MSG_TYPE
};

enum address_t
{
    WHO_I_AM,
    RADIO_CONFIG,
    RANGING_MODE,
    RANGING_RESULT,
    TRX_DATA,
    RESET,
    ROS_CONTROL,
    REQUEST_RANGING,
};

struct ros_id_msg_t
{
    char id[16];
};

struct ros_radio_config_msg_t
{
};

struct ros_ranging_mode_msg_t
{
};

struct request_ranging_t
{
    uint16_t target_id;
    uint8_t preprocessing;
};

struct ros_msg_t
{
    uint8_t address;
    char mode;
    union
    {
        struct ros_id_msg_t id_msg;
        struct ros_radio_config_msg_t radio_config_msg;
        struct ros_ranging_mode_msg_t ranging_mode_msg;
        struct ranging_msg_t ranging_msg;
        struct uwb_data_msg_t uwb_data_msg;
        uint8_t reset;
        uint8_t control;
        struct request_ranging_t request_ranging;
    } data;
};

enum address_anchor_t
{
    ANCHOR_BEACON,
};

enum capabilities_t
{
    NONE,
    RDEV
};

struct anchor_beacon_t
{
    uint8_t capabilities;
};

struct anchor_msg_t
{
    uint8_t address;
    char mode;

    union
    {
        struct anchor_beacon_t anchor_beacon;
    } data;
};

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Serialize ros_msg_t to buffer
     *
     * @param msg pointer to ros_msg_t structure
     * @param buf pointer to buffer
     * @return int returns the overall size of serialized data
     */
    int serialize_ros(const struct ros_msg_t *msg, uint8_t *buf);

    /**
     * @brief Deserialize buffer data to ros_msg_t
     *
     * @param msg pointer to ros_msg_t structure
     * @param buf pointer to buffer in which the data lies
     */
    void deserialize_ros(struct ros_msg_t *msg, const uint8_t *buf);

    /**
     * @brief Serialize anchor_msg_t to buffer
     * 
     * @param msg pointer to anchor_msg_t structure
     * @param buf pointer to buffer
     * @return int returns the overall size of serialized data
     */
    int serialize_anchor_msg(const struct anchor_msg_t *msg, uint8_t *buf);

#ifdef __cplusplus
}
#endif

#endif
