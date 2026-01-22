#ifndef TOF_SNAPSHOT_H
#define TOF_SNAPSHOT_H

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

#include "tof_provider.h"

typedef struct {
    TickType_t timeout_ticks;
    uint32_t max_spins;
    uint32_t odd_yield_threshold;
    uint8_t timeout_status;
    TickType_t log_interval_ticks;
} tof_snapshot_config_t;

typedef struct {
    portMUX_TYPE log_mux;
    uint32_t timeout_count;
    TickType_t timeout_last_log_tick;
} tof_snapshot_log_t;

void tof_snapshot_read(const char *tag,
                       const tof_snapshot_config_t *config,
                       const tof_sample_t samples[TOF_COUNT],
                       const uint32_t seq[TOF_COUNT],
                       tof_snapshot_log_t *log_state,
                       tof_sample_t out[TOF_COUNT]);

#endif
