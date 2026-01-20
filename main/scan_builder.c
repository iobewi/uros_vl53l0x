#include "scan_builder.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

bool scan_builder_init(sensor_msgs__msg__LaserScan *msg, const scan_config_t *cfg)
{
    if (!msg || !cfg || !cfg->frame_id || cfg->bins <= 0) return false;

    sensor_msgs__msg__LaserScan__init(msg);

    const size_t frame_len = strlen(cfg->frame_id);
    msg->header.frame_id.data = (char *)malloc(frame_len + 1);
    if (!msg->header.frame_id.data) {
        sensor_msgs__msg__LaserScan__fini(msg);
        return false;
    }
    memcpy(msg->header.frame_id.data, cfg->frame_id, frame_len + 1);
    msg->header.frame_id.size = frame_len;
    msg->header.frame_id.capacity = frame_len + 1;

    msg->angle_min = cfg->angle_min;
    msg->angle_increment = cfg->angle_inc;
    msg->angle_max = cfg->angle_min + (cfg->bins - 1) * cfg->angle_inc;

    msg->range_min = cfg->range_min;
    msg->range_max = cfg->range_max;

    msg->time_increment = 0.0f;
    msg->scan_time = 0.0f;

    msg->ranges.data = (float*)malloc(sizeof(float) * (size_t)cfg->bins);
    if (!msg->ranges.data) {
        sensor_msgs__msg__LaserScan__fini(msg);
        return false;
    }
    msg->ranges.size = (size_t)cfg->bins;
    msg->ranges.capacity = (size_t)cfg->bins;

    msg->intensities.data = NULL;
    msg->intensities.size = 0;
    msg->intensities.capacity = 0;

    for (int i = 0; i < cfg->bins; i++) {
        msg->ranges.data[i] = NAN;
    }

    return true;
}

void scan_builder_deinit(sensor_msgs__msg__LaserScan *msg)
{
    if (!msg) return;

    sensor_msgs__msg__LaserScan__fini(msg);
}

void scan_builder_fill(sensor_msgs__msg__LaserScan *msg,
                       const scan_config_t *cfg,
                       const tof_sample_t samples[TOF_COUNT],
                       const uint8_t idx_map[TOF_COUNT])
{
    if (!msg || !cfg || !samples || !idx_map || cfg->bins <= 0) return;

    // Clear scan
    for (int i = 0; i < cfg->bins; i++) {
        msg->ranges.data[i] = NAN;
    }

    for (int s = 0; s < TOF_COUNT; s++) {
        const int idx = (int)idx_map[s];
        if ((unsigned)idx >= (unsigned)cfg->bins) continue;

        if (!samples[s].valid || samples[s].status != 0 || isnan(samples[s].range_m)) {
            continue; // leave NAN
        }

        const float r = samples[s].range_m;
        if (r < cfg->range_min || r > cfg->range_max) continue;

        msg->ranges.data[idx] = r;
    }
}
