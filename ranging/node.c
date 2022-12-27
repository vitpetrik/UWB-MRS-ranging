#include "node.h"

void node_init(struct node_t *node, uint16_t id)
{
    node->id = id;
    node->ranging.error_counter = 0;
    node->ranging.expected_packet_number = 0;
    node->ranging.role = INITIATOR;
    node->ranging.tx_timestamp = 0;
    node->ranging.last_meas_time = 0;

    stats_init(&node->ranging.stats, AVERAGE_WINDOW);
}

void node_reset(struct node_t *node)
{
    node->ranging.error_counter = 0;
    node->ranging.expected_packet_number = 0;
    node->ranging.role = INITIATOR;
    node->ranging.tx_timestamp = 0;
    node->ranging.last_meas_time = 0;

    stats_reset(&node->ranging.stats);
}

void node_destroy(struct node_t *node)
{
    stats_destroy(&node->ranging.stats);

    k_free(node);
}