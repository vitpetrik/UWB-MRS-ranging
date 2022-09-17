#ifndef __RANGING_H__
#define __RANGING_H__

#include <zephyr/zephyr.h>
#include <string.h>

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 �s and 1 �s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 64267

#define SPEED_OF_LIGHT 299702547.0

enum {
    SUCCESS,
    RX_ENABLE
};

#include "common_types.h"
#include "deca_device_api.h"

#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

// THREADS
void uwb_beacon_thread(void);
void uwb_ranging_thread(void);

// FUNCTIONS
int process_beacon(beacon_msg *beacon);
int process_ranging(ranging_msg *ranging, struct rx_queue_t *queue_data);

#ifdef __cplusplus
}
#endif

#endif