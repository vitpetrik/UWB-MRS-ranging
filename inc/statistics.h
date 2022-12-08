/**
 * @file statistics.h
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief Little library to do some statistics
 * @version 0.1
 * @date 2022-12-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __STATISTICS_H__
#define __STATISTICS_H__

#include <zephyr/sys/ring_buffer.h>

struct statistics_t {
    float mean;
    float sum2;
    float raw;
    int step;
    struct ring_buf ring_val;
    struct ring_buf ring_sum;
};

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @brief Initialize struct for handling statistics data
 * 
 * @param stats pointer to statistics_t
 * @param size size of ring buffer (aka sliding window size)
 */
void stats_init(struct statistics_t *stats, int size);

/**
 * @brief Update statistics with new value
 * 
 * @param stats pointer to statistics_t 
 * @param val new value
 */
void stats_update(struct statistics_t *stats, const float val);


/**
 * @brief Reset struct to default statw
 * 
 * @param stats pointer to statistics_t
 */
void stats_reset(struct statistics_t *stats);

/**
 * @brief Destroy dynamic arrays
 * 
 * @param stats pointer to statistics_t
 */
void stats_destroy(struct statistics_t *stats);

/**
 * @brief Gets value of mean
 * 
 * @param stats pointer to structure_t
 * @return float mean
 */
float stats_get_mean(const struct statistics_t *stats);

/**
 * @brief Gets value of mean
 * 
 * @param stats pointer to structure_t
 * @return float raw value
 */
float stats_get_raw(const struct statistics_t *stats);

/**
 * @brief Gets value of variance
 * 
 * @param stats pointer to structure_t
 * @return float variance
 */
float stats_get_variance(const struct statistics_t *stats);

/**
 * @brief Gets step of calculation
 * 
 * @param stats pointer to structure_t
 * @return int step
 */
int stats_get_step(const struct statistics_t *stats);

#ifdef __cplusplus
}
#endif

#endif
