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

typedef enum
{
    WHO_I_AM,
    RADIO_CONFIG,
    RANGING_MODE,
    RANGING_RESULT,
    TRX_DATA,
    RESET,
    ROS_CONTROL
} address_t;

struct ros_id_msg_t {
    char id[16];
};

struct ros_radio_config_msg_t {};

struct ros_ranging_mode_msg_t {};

struct ros_msg_t
{
    address_t address;
    char mode;
    union
    {
        struct ros_id_msg_t id_msg;
        struct ros_radio_config_msg_t radio_config_msg;
        struct ros_ranging_mode_msg_t ranging_mode_msg;
        struct ranging_msg_t ranging_msg;
        struct uwb_data_msg_t uwb_data_msg;
        uint8_t reset;
        control_t control;

    } data;
};


#ifdef __cplusplus
extern "C" {
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
 * @brief Desiralize buffer data to ros_msg_t
 * 
 * @param msg pointer to ros_msg_t structure
 * @param buf pointer to buffer in which the data lies
 */
void deserialize_ros(struct ros_msg_t *msg, const uint8_t *buf);

#ifdef __cplusplus
}
#endif

#endif
