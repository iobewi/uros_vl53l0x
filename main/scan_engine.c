#include "scan_engine.h"

#include <stdbool.h>

#include "esp_log.h"
#include "sdkconfig.h"

#include "tof_provider.h"

#define N_BINS CONFIG_MICRO_ROS_SCAN_BINS

static const char *TAG = "SCAN_ENGINE";

static tof_sample_t scan_engine_snapshot[TOF_COUNT];

static int64_t scan_engine_default_time_provider(void)
{
    return 0;
}

static bool validate_tof_bin_map(const scan_config_t *cfg, const tof_hw_config_t *hw_cfg)
{
    if (cfg == NULL || hw_cfg == NULL) {
        return false;
    }
    ESP_LOGI(TAG, "TOF bin mapping (tof -> bin):");
    for (int i = 0; i < TOF_COUNT; i++) {
        ESP_LOGI(TAG, "  tof[%d] -> bin[%u]", i, (unsigned)hw_cfg[i].bin_idx);
    }

#if !CONFIG_TOF_BIN_ALLOW_DUPLICATES
    bool used[N_BINS] = {0};
#endif

    bool valid = true;
    for (int i = 0; i < TOF_COUNT; i++) {
        uint8_t idx = hw_cfg[i].bin_idx;
        if ((int)idx >= cfg->bins) {
            ESP_LOGE(TAG,
                     "TOF bin index out of range (tof=%d idx=%u bins=%d)",
                     i, (unsigned)idx, cfg->bins);
            valid = false;
            continue;
        }
#if !CONFIG_TOF_BIN_ALLOW_DUPLICATES
        if (used[idx]) {
            ESP_LOGE(TAG,
                     "Duplicate TOF bin index detected (tof=%d idx=%u)",
                     i, (unsigned)idx);
            valid = false;
        } else {
            used[idx] = true;
        }
#endif
    }

    return valid;
}

static void scan_engine_set_timestamp(scan_engine_t *e, sensor_msgs__msg__LaserScan *msg)
{
    int64_t now_ns = 0;
    if (e != NULL && e->time_provider != NULL) {
        now_ns = e->time_provider();
    }
    if (now_ns > 0) {
        msg->header.stamp.sec = (int32_t)(now_ns / 1000000000LL);
        msg->header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);
    } else {
        msg->header.stamp.sec = 0;
        msg->header.stamp.nanosec = 0;
    }
}

bool scan_engine_init(scan_engine_t *e, const scan_config_t *cfg, const tof_hw_config_t *hw_cfg)
{
    if (e == NULL || cfg == NULL || hw_cfg == NULL) {
        return false;
    }

    if (!validate_tof_bin_map(cfg, hw_cfg)) {
        return false;
    }

    e->cfg = cfg;
    e->hw_cfg = hw_cfg;
    e->time_provider = scan_engine_default_time_provider;

    return true;
}

void scan_engine_deinit(scan_engine_t *e)
{
    if (e == NULL) {
        return;
    }
    e->cfg = NULL;
    e->hw_cfg = NULL;
    e->time_provider = scan_engine_default_time_provider;
}

bool scan_engine_step(scan_engine_t *e, sensor_msgs__msg__LaserScan *out_msg)
{
    if (e == NULL || e->cfg == NULL || e->hw_cfg == NULL || out_msg == NULL) {
        return false;
    }

    tof_provider_snapshot(scan_engine_snapshot);
    scan_builder_fill(out_msg, e->cfg, scan_engine_snapshot, e->hw_cfg);
    scan_engine_set_timestamp(e, out_msg);

    return true;
}

void scan_engine_set_time_provider(scan_engine_t *e, int64_t (*now_ns)(void))
{
    if (e == NULL) {
        return;
    }
    if (now_ns == NULL) {
        e->time_provider = scan_engine_default_time_provider;
        return;
    }
    e->time_provider = now_ns;
}
