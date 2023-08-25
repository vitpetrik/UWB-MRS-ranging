
/**
 * @file ros.c
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief Definitions of structures and functions for ROS communication
 * @version 0.1
 * @date 2022-11-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "ros.h"
#include <zephyr/sys/__assert.h>

/**
 * ! All of this is using extreamly dumb and repetitive way
 * ! of doing things... more elegant solution might be possible
 * ! but dont change what ain't broken! :-)
 */

/**
 * @brief Serialize ros_msg_t to buffer
 *
 * @param msg pointer to ros_msg_t structure
 * @param buf pointer to buffer
 * @return int returns the overall size of serialized data
 */
int serialize_ros(const struct ros_msg_t *msg, uint8_t *buf)
{
    int index = 0;

    buf[index++] = msg->address;
    buf[index++] = msg->mode;

    if (msg->mode == 'r')
        return index;

    switch (msg->address)
    {
    case WHO_I_AM:
        memcpy(&buf[index], msg->data.id_msg.id, sizeof(msg->data.id_msg.id));
        index += sizeof(msg->data.id_msg.id);
        break;
    case RADIO_CONFIG:
        break;
    case RANGING_MODE:
        break;
    case RANGING_RESULT:
        memcpy(&buf[index], &msg->data.ranging_msg.source_mac, sizeof(msg->data.ranging_msg.source_mac));
        index += sizeof(msg->data.ranging_msg.source_mac);
        memcpy(&buf[index], &msg->data.ranging_msg.range, sizeof(msg->data.ranging_msg.range));
        index += sizeof(msg->data.ranging_msg.range);
        memcpy(&buf[index], &msg->data.ranging_msg.variance, sizeof(msg->data.ranging_msg.variance));
        index += sizeof(msg->data.ranging_msg.variance);
        memcpy(&buf[index], &msg->data.ranging_msg.raw, sizeof(msg->data.ranging_msg.raw));
        index += sizeof(msg->data.ranging_msg.raw);
        memcpy(&buf[index], &msg->data.ranging_msg.power_a, sizeof(msg->data.ranging_msg.power_a));
        index += sizeof(msg->data.ranging_msg.power_a);
        memcpy(&buf[index], &msg->data.ranging_msg.power_b, sizeof(msg->data.ranging_msg.power_b));
        index += sizeof(msg->data.ranging_msg.power_b);
        break;
    case TRX_DATA:
        memcpy(&buf[index], &msg->data.uwb_data_msg.source_mac, sizeof(msg->data.uwb_data_msg.source_mac));
        index += sizeof(msg->data.uwb_data_msg.source_mac);
        memcpy(&buf[index], &msg->data.uwb_data_msg.destination_mac, sizeof(msg->data.uwb_data_msg.destination_mac));
        index += sizeof(msg->data.uwb_data_msg.destination_mac);
        memcpy(&buf[index], &msg->data.uwb_data_msg.msg_type, sizeof(msg->data.uwb_data_msg.msg_type));
        index += sizeof(msg->data.uwb_data_msg.msg_type);
        memcpy(&buf[index], &msg->data.uwb_data_msg.payload_size, sizeof(msg->data.uwb_data_msg.payload_size));
        index += sizeof(msg->data.uwb_data_msg.payload_size);
        memcpy(&buf[index], msg->data.uwb_data_msg.payload, msg->data.uwb_data_msg.payload_size);
        index += msg->data.uwb_data_msg.payload_size;
        k_free(msg->data.uwb_data_msg.payload);
        break;
    case RESET:
        memcpy(&buf[index], &msg->data.reset, sizeof(msg->data.reset));
        index += sizeof(msg->data.reset);
        break;
    case ROS_CONTROL:
        memcpy(&buf[index], &msg->data.control, sizeof(msg->data.control));
        index += sizeof(msg->data.control);
        break;
    default:
        break;
    }

    // index == len at end of the transmission
    return index;
}

/**
 * @brief Desiralize buffer data to ros_msg_t
 *
 * @param msg pointer to ros_msg_t structure
 * @param buf pointer to buffer in which the data lies
 */
void deserialize_ros(struct ros_msg_t *msg, const uint8_t *buf)
{
    int index = 0;

    msg->address = buf[index++];
    msg->mode = buf[index++];

    if (msg->mode == 'r')
        return;

    switch (msg->address)
    {
    case WHO_I_AM:
        memcpy(msg->data.id_msg.id, &buf[index], sizeof(msg->data.id_msg.id));
        index += sizeof(msg->data.id_msg.id);
        break;
    case RADIO_CONFIG:
        break;
    case RANGING_MODE:
        break;
    case RANGING_RESULT:
        memcpy(&msg->data.ranging_msg.source_mac, &buf[index], sizeof(msg->data.ranging_msg.source_mac));
        index += sizeof(msg->data.ranging_msg.source_mac);
        memcpy(&msg->data.ranging_msg.range, &buf[index], sizeof(msg->data.ranging_msg.range));
        index += sizeof(msg->data.ranging_msg.range);
        memcpy(&msg->data.ranging_msg.variance, &buf[index], sizeof(msg->data.ranging_msg.variance));
        index += sizeof(msg->data.ranging_msg.variance);
        memcpy(&msg->data.ranging_msg.raw, &buf[index], sizeof(msg->data.ranging_msg.raw));
        index += sizeof(msg->data.ranging_msg.raw);
        break;
    case TRX_DATA:
        memcpy(&msg->data.uwb_data_msg.source_mac, &buf[index], sizeof(msg->data.uwb_data_msg.source_mac));
        index += sizeof(msg->data.uwb_data_msg.source_mac);
        memcpy(&msg->data.uwb_data_msg.destination_mac, &buf[index], sizeof(msg->data.uwb_data_msg.destination_mac));
        index += sizeof(msg->data.uwb_data_msg.destination_mac);
        memcpy(&msg->data.uwb_data_msg.msg_type, &buf[index], sizeof(msg->data.uwb_data_msg.msg_type));
        index += sizeof(msg->data.uwb_data_msg.msg_type);
        memcpy(&msg->data.uwb_data_msg.payload_size, &buf[index], sizeof(msg->data.uwb_data_msg.payload_size));
        index += sizeof(msg->data.uwb_data_msg.payload_size);

        msg->data.uwb_data_msg.payload = k_malloc(msg->data.uwb_data_msg.payload_size);
        __ASSERT(msg->data.uwb_data_msg.payload != NULL, "Failed to allocate buffer!");

        memcpy(msg->data.uwb_data_msg.payload, &buf[index], msg->data.uwb_data_msg.payload_size);
        index += msg->data.uwb_data_msg.payload_size;
        break;
    case RESET:
        memcpy(&msg->data.reset, &buf[index], sizeof(msg->data.reset));
        index += sizeof(msg->data.reset);
        break;
    case ROS_CONTROL:
        memcpy(&msg->data.control, &buf[index], sizeof(msg->data.control));
        index += sizeof(msg->data.control);
        break;
    case REQUEST_RANGING:
        memcpy(&msg->data.request_ranging.target_id, &buf[index], sizeof(msg->data.request_ranging.target_id));
        index += sizeof(msg->data.request_ranging.target_id);
        memcpy(&msg->data.request_ranging.preprocessing, &buf[index], sizeof(msg->data.request_ranging.preprocessing));
        index += sizeof(msg->data.request_ranging.preprocessing);
        break;

    default:
        break;
    }
}

/**
 * @brief Serialize anchor_msg_t to buffer
 * 
 * @param msg pointer to anchor_msg_t structure
 * @param buf pointer to buffer
 * @return int returns the overall size of serialized data
 */
int serialize_anchor_msg(const struct anchor_msg_t *msg, uint8_t *buf)
{
    int index = 0;

    buf[index++] = msg->address;
    buf[index++] = msg->mode;

    switch (msg->address)
    {
    case ANCHOR_BEACON:
        memcpy(&buf[index], &msg->data.anchor_beacon.capabilities, sizeof(msg->data.anchor_beacon.capabilities));
        index += sizeof(msg->data.anchor_beacon.capabilities);
        break;
    default:
        break;
    }

    return index;
}
