/**
 * @file node.c
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief Node struct thats mandatory for ranging
 * @version 0.1
 * @date 2022-12-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "node.h"

/**
 * @brief Initialize Node struct
 * 
 * @param node pointer to node_t
 * @param id Id of node
 * @param window_size Windows size for averaging
 */
void node_init(struct node_t *node, uint16_t id, int window_size)
{
    node->id = id;
    node->ranging.ranging_state = DISABLED;
    node->ranging.error_counter = 0;
    node->ranging.expected_packet_number = 0;
    node->ranging.role = INITIATOR;
    node->ranging.tx_timestamp = 0;
    node->ranging.last_meas_time = 0;

    if (window_size < 1)
        return;

    stats_init(&node->ranging.stats, window_size);
    return;
}

/**
 * @brief Reset node
 * 
 * @param node pointer to node_t
 */
void node_reset(struct node_t *node)
{
    node->ranging.ranging_state = DISABLED;
    node->ranging.error_counter = 0;
    node->ranging.expected_packet_number = 0;
    node->ranging.role = INITIATOR;
    node->ranging.tx_timestamp = 0;
    node->ranging.last_meas_time = 0;

    stats_reset(&node->ranging.stats);
    return;
}

/**
 * @brief Destroy node_t object
 * 
 * @param node pointer to node_t
 */
void node_destroy(struct node_t *node)
{
    stats_destroy(&node->ranging.stats);

    k_free(node);
    return;
}