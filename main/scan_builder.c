#include "scan_builder.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline float wrap_to_pi(float a)
{
    while (a > (float)M_PI)  a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static inline int angle_to_index(float angle_rad, float angle_min, float angle_inc, int bins)
{
    float a = wrap_to_pi(angle_rad);
    int idx = (int)lroundf((a - angle_min) / angle_inc);
    if (idx < 0) idx = 0;
    if (idx >= bins) idx = bins - 1;
    return idx;
}

bool scan_builder_init(sensor_msgs__msg__LaserScan *msg, const scan_config_t *cfg)
{
    if (!msg || !cfg || !cfg->frame_id || cfg->bins <= 0) return false;

    sensor_msgs__msg__LaserScan__init(msg);

    // frame_id points to a constant string (no dynamic allocation)
    msg->header.frame_id.data = (char*)cfg->frame_id;
    msg->header.frame_id.size = strlen(cfg->frame_id);
    msg->header.frame_id.capacity = msg->header.frame_id.size + 1;

    msg->angle_min = cfg->angle_min;
    msg->angle_increment = cfg->angle_inc;
    msg->angle_max = cfg->angle_min + (cfg->bins - 1) * cfg->angle_inc;

    msg->range_min = cfg->range_min;
    msg->range_max = cfg->range_max;

    msg->time_increment = 0.0f;
    msg->scan_time = 0.0f;

    // Allocate ranges once
    msg->ranges.data = (float*)malloc(sizeof(float) * (size_t)cfg->bins);
    if (!msg->ranges.data) return false;
    msg->ranges.size = (size_t)cfg->bins;
    msg->ranges.capacity = (size_t)cfg->bins;

    // No intensities
    msg->intensities.data = NULL;
    msg->intensities.size = 0;
    msg->intensities.capacity = 0;

    // Initialize to NAN
    for (int i = 0; i < cfg->bins; i++) {
        msg->ranges.data[i] = NAN;
    }

    return true;
}

void scan_builder_fill(sensor_msgs__msg__LaserScan *msg,
                       const scan_config_t *cfg,
                       const tof_sample_t samples[TOF_COUNT])
{
    if (!msg || !cfg || !samples || !cfg->tof_angles_rad) return;

    // Reset all bins to NAN
    for (int i = 0; i < cfg->bins; i++) {
        msg->ranges.data[i] = NAN;
    }

    // Place the TOF_COUNT samples into their bins
    for (int i = 0; i < TOF_COUNT; i++) {
        const int idx = angle_to_index(cfg->tof_angles_rad[i], cfg->angle_min, cfg->angle_inc, cfg->bins);

        if (!samples[i].valid || samples[i].status != 0 || isnan(samples[i].range_m)) {
            msg->ranges.data[idx] = NAN;
            continue;
        }

        const float r = samples[i].range_m;
        if (r < cfg->range_min || r > cfg->range_max) {
            msg->ranges.data[idx] = NAN;
        } else {
            msg->ranges.data[idx] = r;
        }
    }
}
