#include "scan_builder.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define SCAN_BINS_MAX CONFIG_MICRO_ROS_SCAN_BINS

#if CONFIG_MICRO_ROS_SCAN_ALLOC_GUARD
#include "esp_heap_caps.h"
#include "esp_log.h"
#endif

#if CONFIG_MICRO_ROS_SCAN_ALLOC_GUARD
typedef struct {
    size_t free_bytes;
    size_t largest_block;
} heap_guard_t;

static const char *TAG = "scan_builder";

static heap_guard_t heap_guard_begin(void)
{
    heap_guard_t guard = {
        .free_bytes = heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
        .largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT),
    };
    return guard;
}

static void heap_guard_end(heap_guard_t before, const char *label)
{
    const size_t free_after = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    const size_t largest_after = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    if (free_after != before.free_bytes || largest_after != before.largest_block) {
        ESP_LOGE(TAG,
                 "Heap changed during %s (free=%zu->%zu, largest=%zu->%zu)",
                 label, before.free_bytes, free_after, before.largest_block, largest_after);
    }
}
#endif

bool scan_builder_init(sensor_msgs__msg__LaserScan *msg,
                       const scan_config_t *cfg,
                       scan_builder_storage_t *storage)
{
    if (!msg || !cfg || !storage || !cfg->frame_id || cfg->bins <= 0) return false;
    if (cfg->bins > SCAN_BINS_MAX) return false;

    const size_t frame_len = strlen(cfg->frame_id);

    storage->owns_ranges_buffer = false;
    storage->owns_frame_id_buffer = false;

#if CONFIG_MICRO_ROS_SCAN_BUILDER_ALLOC_MALLOC
    if (!storage->ranges_buffer) {
        storage->ranges_buffer = calloc((size_t)cfg->bins, sizeof(float));
        if (!storage->ranges_buffer) return false;
        storage->ranges_capacity = (size_t)cfg->bins;
        storage->owns_ranges_buffer = true;
    }
    if (!storage->frame_id_buffer) {
        storage->frame_id_buffer = malloc(frame_len + 1);
        if (!storage->frame_id_buffer) {
            if (storage->owns_ranges_buffer) {
                free(storage->ranges_buffer);
                storage->ranges_buffer = NULL;
                storage->ranges_capacity = 0;
                storage->owns_ranges_buffer = false;
            }
            return false;
        }
        storage->frame_id_capacity = frame_len + 1;
        storage->owns_frame_id_buffer = true;
    }
#else
    if (!storage->ranges_buffer || !storage->frame_id_buffer) return false;
#endif

    if (storage->ranges_capacity < (size_t)cfg->bins ||
        storage->frame_id_capacity < frame_len + 1) {
        if (storage->owns_frame_id_buffer) {
            free(storage->frame_id_buffer);
            storage->frame_id_buffer = NULL;
            storage->frame_id_capacity = 0;
            storage->owns_frame_id_buffer = false;
        }
        if (storage->owns_ranges_buffer) {
            free(storage->ranges_buffer);
            storage->ranges_buffer = NULL;
            storage->ranges_capacity = 0;
            storage->owns_ranges_buffer = false;
        }
        return false;
    }

    sensor_msgs__msg__LaserScan__init(msg);

    memcpy(storage->frame_id_buffer, cfg->frame_id, frame_len + 1);
    msg->header.frame_id.data = storage->frame_id_buffer;
    msg->header.frame_id.size = frame_len;
    msg->header.frame_id.capacity = storage->frame_id_capacity;

    msg->angle_min = cfg->angle_min;
    msg->angle_increment = cfg->angle_inc;
    msg->angle_max = cfg->angle_min + (cfg->bins - 1) * cfg->angle_inc;

    msg->range_min = cfg->range_min;
    msg->range_max = cfg->range_max;

    msg->time_increment = cfg->time_increment;
    msg->scan_time = cfg->scan_time;

    msg->ranges.data = storage->ranges_buffer;
    msg->ranges.size = (size_t)cfg->bins;
    msg->ranges.capacity = storage->ranges_capacity;

    msg->intensities.data = NULL;
    msg->intensities.size = 0;
    msg->intensities.capacity = 0;

    for (int i = 0; i < cfg->bins; i++) {
        msg->ranges.data[i] = NAN;
    }

    return true;
}

void scan_builder_deinit(sensor_msgs__msg__LaserScan *msg, scan_builder_storage_t *storage)
{
    if (!msg || !storage) return;

    if (storage->owns_frame_id_buffer && storage->frame_id_buffer) {
        free(storage->frame_id_buffer);
        storage->frame_id_buffer = NULL;
        storage->frame_id_capacity = 0;
        storage->owns_frame_id_buffer = false;
    }
    if (storage->owns_ranges_buffer && storage->ranges_buffer) {
        free(storage->ranges_buffer);
        storage->ranges_buffer = NULL;
        storage->ranges_capacity = 0;
        storage->owns_ranges_buffer = false;
    }

    msg->header.frame_id.data = NULL;
    msg->header.frame_id.size = 0;
    msg->header.frame_id.capacity = 0;
    msg->ranges.data = NULL;
    msg->ranges.size = 0;
    msg->ranges.capacity = 0;

    sensor_msgs__msg__LaserScan__fini(msg);
}

void scan_builder_fill(sensor_msgs__msg__LaserScan *msg,
                       const scan_config_t *cfg,
                       const tof_sample_t samples[TOF_COUNT],
                       const tof_hw_config_t *hw_cfg)
{
    if (!msg || !cfg || !samples || !hw_cfg || cfg->bins <= 0) return;
    if (!msg->ranges.data || msg->ranges.capacity < (size_t)cfg->bins) return;
#if CONFIG_MICRO_ROS_SCAN_ALLOC_GUARD
    heap_guard_t guard = heap_guard_begin();
#endif

    // Clear scan
    for (int i = 0; i < cfg->bins; i++) {
        msg->ranges.data[i] = NAN;
    }

    for (int s = 0; s < TOF_COUNT; s++) {
        const int idx = (int)hw_cfg[s].bin_idx;
        if ((unsigned)idx >= (unsigned)cfg->bins) continue;

        if (!samples[s].valid || samples[s].status != 0 || isnan(samples[s].range_m)) {
            continue; // leave NAN
        }

        const float r = samples[s].range_m;
        if (r < cfg->range_min || r > cfg->range_max) continue;

        msg->ranges.data[idx] = r;
    }
#if CONFIG_MICRO_ROS_SCAN_ALLOC_GUARD
    heap_guard_end(guard, "scan_builder_fill");
#endif
}
