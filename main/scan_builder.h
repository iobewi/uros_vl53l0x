#pragma once
#include <stdbool.h>

#include <sensor_msgs/msg/laser_scan.h>
#include "tof_provider.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float angle_min;
    float angle_inc;
    int bins;

    float range_min;
    float range_max;

    const char *frame_id;

    // Per-ToF physical angles in radians; size = TOF_COUNT
    const float *tof_angles_rad;
} scan_config_t;

/**
 * @brief Initialize a LaserScan message, allocating the ranges buffer once.
 *        intensities are left empty.
 */
bool scan_builder_init(sensor_msgs__msg__LaserScan *msg, const scan_config_t *cfg);

/**
 * @brief Fill the LaserScan using a snapshot of TOF_COUNT samples.
 *        All bins are reset to NAN, then bins corresponding to ToF angles are filled.
 */
void scan_builder_fill(sensor_msgs__msg__LaserScan *msg,
                       const scan_config_t *cfg,
                       const tof_sample_t samples[TOF_COUNT]);

#ifdef __cplusplus
}
#endif
