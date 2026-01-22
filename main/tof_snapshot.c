#include "tof_snapshot.h"

#include <inttypes.h>
#include <math.h>

#include "esp_log.h"

static void log_snapshot_timeout(const char *tag,
                                 const tof_snapshot_config_t *config,
                                 tof_snapshot_log_t *log_state,
                                 int idx,
                                 uint32_t seq)
{
    TickType_t now = xTaskGetTickCount();
    uint32_t suppressed = 0;
    bool should_log = false;

    portENTER_CRITICAL(&log_state->log_mux);
    log_state->timeout_count++;
    if (log_state->timeout_last_log_tick == 0 ||
        (now - log_state->timeout_last_log_tick) >= config->log_interval_ticks) {
        suppressed = log_state->timeout_count - 1;
        log_state->timeout_count = 0;
        log_state->timeout_last_log_tick = now;
        should_log = true;
    }
    portEXIT_CRITICAL(&log_state->log_mux);

    if (!should_log) {
        return;
    }

    if (suppressed > 0) {
        ESP_LOGW(tag,
                 "Snapshot timeout (idx=%d seq=%" PRIu32 ", suppressed=%" PRIu32 ")",
                 idx,
                 seq,
                 suppressed);
        return;
    }

    ESP_LOGW(tag, "Snapshot timeout (idx=%d seq=%" PRIu32 ")", idx, seq);
}

void tof_snapshot_read(const char *tag,
                       const tof_snapshot_config_t *config,
                       const tof_sample_t samples[TOF_COUNT],
                       const uint32_t seq[TOF_COUNT],
                       tof_snapshot_log_t *log_state,
                       tof_sample_t out[TOF_COUNT])
{
    for (int i = 0; i < TOF_COUNT; i++) {
        TickType_t start = xTaskGetTickCount();
        uint32_t spins = 0;
        uint32_t odd_spins = 0;
        while (1) {
            uint32_t seq1 = __atomic_load_n(&seq[i], __ATOMIC_ACQUIRE);
            if (seq1 & 1u) {
                odd_spins++;
                spins++;
                if (odd_spins >= config->odd_yield_threshold) {
                    odd_spins = 0;
                    vTaskDelay(1);
                }
                if (spins >= config->max_spins ||
                    (xTaskGetTickCount() - start) > config->timeout_ticks) {
                    log_snapshot_timeout(tag, config, log_state, i, seq1);
                    out[i] = (tof_sample_t){
                        .valid = false,
                        .status = config->timeout_status,
                        .range_m = NAN,
                        .seq = seq1,
                    };
                    break;
                }
                continue;
            }
            tof_sample_t sample = samples[i];
            uint32_t seq2 = __atomic_load_n(&seq[i], __ATOMIC_ACQUIRE);
            if (seq1 == seq2 && !(seq2 & 1u)) {
                out[i] = sample;
                break;
            }
            spins++;
            if (spins >= config->max_spins ||
                (xTaskGetTickCount() - start) > config->timeout_ticks) {
                log_snapshot_timeout(tag, config, log_state, i, seq2);
                out[i] = (tof_sample_t){
                    .valid = false,
                    .status = config->timeout_status,
                    .range_m = NAN,
                    .seq = seq2,
                };
                break;
            }
            taskYIELD();
        }
    }
}
