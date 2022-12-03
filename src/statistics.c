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

#include "statistics.h"

/**
 * @brief Initialize struct for handling statistics data
 * 
 * @param stats pointer to statistics_t
 */
void stats_init(struct statistics_t *stats)
{
    stats->mean = 0;
    stats->sum = 0;
    stats->step = 0;
    return;
}

/**
 * @brief Update statistics with new value
 * 
 * @param stats pointer to statistics_t 
 * @param val new value
 */
void stats_update(struct statistics_t *stats, const float val)
{
    if (stats->step == 0)
    {
        stats->mean = val;
        stats->sum = 0;
    }

    else
    {
        float prev = stats->mean;

        stats->mean += (val-stats->mean)/(stats->step+1);
        stats->sum += (val - prev)*(val - stats->mean);
    }

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
    stats_init(stats);
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
    return stats->mean;
}

/**
 * @brief Gets value of variance
 * 
 * @param stats pointer to structure_t
 * @return float variance
 */
float stats_get_variance(const struct statistics_t *stats)
{
    if (stats < 2)
        return stats->mean;

    return stats->sum/(stats->step-1);
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
