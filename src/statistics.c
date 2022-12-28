/**
 * @file statistics.c
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief Little library to do some statistics
 * @version 0.1
 * @date 2022-12-03
 *
 * @copyright Copyright (c) 2022
 *
 * Calculation of Mean and Variance is based on Welford's algortigm
 * more info at https://nullbuffer.com/articles/welford_algorithm.html
 *
 */

#include <zephyr/sys/__assert.h>
#include "statistics.h"
#include <string.h>

/**
 * @brief Initialize struct for handling statistics data
 *
 * @param stats pointer to statistics_t
 */
void stats_init(struct statistics_t *stats, int size)
{
    if(size < 1)
        size = 1;

    stats->mean = 0;
    stats->raw = 0;
    stats->sum2 = 0;
    stats->step = 0;
    stats->window_size = size;

    stats->init = 1;

    uint32_t *buf_val = k_malloc(2*stats->window_size*sizeof(uint32_t));
    __ASSERT(buf_val != NULL, "Could not allocate buf_val!");

    uint32_t *buf_sum = k_malloc(2*stats->window_size*sizeof(uint32_t));
    __ASSERT(buf_val != NULL, "Could not allocate buf_sum!");

    ring_buf_item_init(&stats->ring_val, stats->window_size, buf_val);
    ring_buf_item_init(&stats->ring_sum, stats->window_size, buf_sum);

    return;
}

/**
 * @brief Checks whether the stats is initialized
 * 
 * @param stats pointer to statistics_t
 * @return int status of the instance
 */
int stats_is_initialized(struct statistics_t *stats)
{
    return stats->init;
}

/**
 * @brief Update statistics with new value
 *
 * @param stats pointer to statistics_t
 * @param val new value
 */
void stats_update(struct statistics_t *stats, const float val)
{
    stats->raw = val;

    if(stats->window_size < 2)
        return;

    if(ring_buf_item_space_get(&stats->ring_val) < 2)
    {
        uint32_t temp;
        float data;

        uint16_t type;
        uint8_t value, size;

        uint32_t buf[2];

        size = 2;

        ring_buf_item_get(&stats->ring_val, &type, &value, buf, &size);
        memcpy(&data, &buf[0], sizeof(float));
        stats->mean = ((stats->mean * stats->step) - data) / (stats->step - 1);

        size = 2;
        ring_buf_item_get(&stats->ring_sum, &type, &value, buf, &size);
        memcpy(&data, &buf[0], sizeof(float));
        stats->sum2 -= data;
        stats->step--;
    }

    float sum = 0;

    if (stats->step == 0)
    {
        stats->mean = stats->raw;
        stats->sum2 = 0;
    }

    else
    {
        float prev = stats->mean;

        stats->mean += (stats->raw - stats->mean) / (stats->step + 1);
        sum = (stats->raw - prev) * (stats->raw - stats->mean);
        stats->sum2 += sum;
    }

    ring_buf_item_put(&stats->ring_val, 0, 0, &stats->raw, 1);
    ring_buf_item_put(&stats->ring_sum, 0, 0, &sum, 1);

    stats->step++;
    return;
}

/**
 * @brief Reset struct to default statw
 *
 * @param stats pointer to statistics_t
 */
void stats_reset(struct statistics_t *stats)
{
    stats->mean = 0;
    stats->raw = 0;
    stats->sum2 = 0;
    stats->step = 0;

    ring_buf_reset(&stats->ring_val);
    ring_buf_reset(&stats->ring_sum);

    return;
}

/**
 * @brief Destroy dynamic arrays
 * 
 * @param stats pointer to statistics_t
 */
void stats_destroy(struct statistics_t *stats)
{
    stats->init = 0;
    k_free(stats->ring_sum.buffer);
    k_free(stats->ring_val.buffer);

    return;
}

/**
 * @brief Gets value of mean
 *
 * @param stats pointer to structure_t
 * @return float mean
 */
float stats_get_mean(const struct statistics_t *stats)
{
    if(stats->window_size < 2)
        return stats_get_raw(stats);

    return stats->mean;
}

/**
 * @brief Gets value of mean
 *
 * @param stats pointer to structure_t
 * @return float raw value
 */
float stats_get_raw(const struct statistics_t *stats)
{
    return stats->raw;
}

/**
 * @brief Gets value of variance
 *
 * @param stats pointer to structure_t
 * @return float variance
 */
float stats_get_variance(const struct statistics_t *stats)
{
    if (stats->step < 2)
        return stats->mean;

    return stats->sum2 / (stats->step - 1);
}

/**
 * @brief Gets step of calculation
 *
 * @param stats pointer to structure_t
 * @return int step
 */
int stats_get_step(const struct statistics_t *stats)
{
    return stats->step;
}

/**
 * @brief Gets size of averaging windows
 *
 * @param stats pointer to strcutre_t
 * @return int windows size
 */
int stats_get_window_size(const struct statistics_t *stats)
{
    return stats->window_size;
}
