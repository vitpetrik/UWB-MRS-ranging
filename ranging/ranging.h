#ifndef __RANGING_H__
#define __RANGING_H__

#include <zephyr/zephyr.h>
#include <string.h>

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 �s and 1 �s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 64267

#define SPEED_OF_LIGHT 299702547.0

enum
{
    SUCCESS,
    RX_ENABLE
};


#include "common_types.h"
#include "deca_device_api.h"


enum
{
    BEACON_MSG,
    RANGING_INIT_MSG,
    RANGING_RESPONSE_MSG,
    RANGING_DS_MSG,
};

enum
{
    UAV_TYPE_DEFAULT = 0
};

#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Receiving thread, waits for queue
     *
     */
    void ranging_thread(void);

    // THREADS
    void uwb_beacon_thread(void);
    void uwb_ranging_thread(void);
    void uwb_ranging_print_thread(void);

    // FUNCTIONS
    int rx_message(struct rx_queue_t *queue_data);

    int rx_beacon(const uint16_t source_id, void *msg);
    int rx_ranging_ds(const uint16_t source_id, void *msg, struct rx_details_t *queue_data);

#ifdef __cplusplus
}
#endif

#endif