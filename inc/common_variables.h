/**
 * @file common_variables.h
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief Declarations of global variables
 * @version 0.1
 * @date 2022-11-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __GLOBAL_VARIABLES_H__
#define __GLOBAL_VARIABLES_H__

#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include "common_types.h"

extern struct mrs_ranging_t mrs_ranging;

extern uint16_t DEVICE_ID;
extern uint16_t PAN_ID;
extern uint8_t SEQ_NUM;

extern struct k_msgq uwb_tx_msgq;
extern struct k_msgq uwb_rx_msgq;

extern struct k_msgq uwb_msgq;

extern struct k_mutex dwt_mutex;

extern struct device *uart_dev;

#endif
