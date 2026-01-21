#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <sensor_msgs/msg/laser_scan.h>

#include "scan_builder.h"
#include "tof_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const scan_config_t *cfg;
    const tof_hw_config_t *hw_cfg;
    int64_t (*time_provider)(void);
} scan_engine_t;

bool scan_engine_init(scan_engine_t *e, const scan_config_t *cfg, const tof_hw_config_t *hw_cfg);
void scan_engine_deinit(scan_engine_t *e);
bool scan_engine_step(scan_engine_t *e, sensor_msgs__msg__LaserScan *out_msg);
void scan_engine_set_time_provider(scan_engine_t *e, int64_t (*now_ns)(void));

#ifdef __cplusplus
}
#endif
