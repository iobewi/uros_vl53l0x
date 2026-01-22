#include "tof_provider.h"

#include <math.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

static const char *TAG = "TOF_PROVIDER_MOCK";
static const TickType_t k_snapshot_timeout_ticks = pdMS_TO_TICKS(2);
static const uint32_t k_snapshot_max_spins = 2000;
static const uint32_t k_snapshot_odd_yield_threshold = 50;
static const uint8_t k_snapshot_timeout_status = 252;
static const TickType_t k_snapshot_log_interval_ticks = pdMS_TO_TICKS(1000);

// État partagé (séquence par capteur).
// Les writers mettent à jour chaque capteur avec un compteur de séquence.
// Le snapshot lit chaque capteur sans verrou via un double-check de séquence.
static tof_sample_t g_tof_samples[TOF_COUNT];
static uint32_t g_tof_seq[TOF_COUNT];
static portMUX_TYPE g_tof_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE g_timeout_log_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t g_snapshot_timeout_count;
static TickType_t g_snapshot_timeout_last_log_tick;

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline void update_one(int i, bool valid, uint8_t status, float range_m)
{
    portENTER_CRITICAL(&g_tof_mux);
    uint32_t seq = __atomic_load_n(&g_tof_seq[i], __ATOMIC_RELAXED);
    __atomic_store_n(&g_tof_seq[i], seq + 1u, __ATOMIC_RELAXED);
    g_tof_samples[i].valid = valid;
    g_tof_samples[i].status = status;
    g_tof_samples[i].range_m = range_m;
    g_tof_samples[i].seq = seq + 2u;
    __atomic_store_n(&g_tof_seq[i], seq + 2u, __ATOMIC_RELEASE);
    portEXIT_CRITICAL(&g_tof_mux);
}

static void log_snapshot_timeout(int idx, uint32_t seq)
{
    TickType_t now = xTaskGetTickCount();
    uint32_t suppressed = 0;
    bool should_log = false;

    portENTER_CRITICAL(&g_timeout_log_mux);
    g_snapshot_timeout_count++;
    if (g_snapshot_timeout_last_log_tick == 0 ||
        (now - g_snapshot_timeout_last_log_tick) >= k_snapshot_log_interval_ticks) {
        suppressed = g_snapshot_timeout_count - 1;
        g_snapshot_timeout_count = 0;
        g_snapshot_timeout_last_log_tick = now;
        should_log = true;
    }
    portEXIT_CRITICAL(&g_timeout_log_mux);

    if (!should_log) {
        return;
    }

    if (suppressed > 0) {
        ESP_LOGW(TAG,
                 "Snapshot timeout (idx=%d seq=%" PRIu32 ", suppressed=%" PRIu32 ")",
                 idx,
                 seq,
                 suppressed);
        return;
    }

    ESP_LOGW(TAG, "Snapshot timeout (idx=%d seq=%" PRIu32 ")", idx, seq);
}

void tof_provider_snapshot(tof_sample_t out[TOF_COUNT])
{
    for (int i = 0; i < TOF_COUNT; i++) {
        TickType_t start = xTaskGetTickCount();
        uint32_t spins = 0;
        uint32_t odd_spins = 0;
        while (1) {
            uint32_t seq1 = __atomic_load_n(&g_tof_seq[i], __ATOMIC_ACQUIRE);
            if (seq1 & 1u) {
                odd_spins++;
                spins++;
                if (odd_spins >= k_snapshot_odd_yield_threshold) {
                    odd_spins = 0;
                    vTaskDelay(1);
                }
                if (spins >= k_snapshot_max_spins ||
                    (xTaskGetTickCount() - start) > k_snapshot_timeout_ticks) {
                    log_snapshot_timeout(i, seq1);
                    out[i] = (tof_sample_t){
                        .valid = false,
                        .status = k_snapshot_timeout_status,
                        .range_m = NAN,
                        .seq = seq1,
                    };
                    break;
                }
                continue;
            }
            tof_sample_t sample = g_tof_samples[i];
            uint32_t seq2 = __atomic_load_n(&g_tof_seq[i], __ATOMIC_ACQUIRE);
            if (seq1 == seq2 && !(seq2 & 1u)) {
                out[i] = sample;
                break;
            }
            spins++;
            if (spins >= k_snapshot_max_spins ||
                (xTaskGetTickCount() - start) > k_snapshot_timeout_ticks) {
                log_snapshot_timeout(i, seq2);
                out[i] = (tof_sample_t){
                    .valid = false,
                    .status = k_snapshot_timeout_status,
                    .range_m = NAN,
                    .seq = seq2,
                };
                break;
            }
            taskYIELD();
        }
    }
}

static void mock_task(void *arg)
{
    (void)arg;

    // Init
    for (int i = 0; i < TOF_COUNT; i++) {
        update_one(i, true, 0, 0.15f);
    }

    while (1) {
        const int64_t us = esp_timer_get_time();
        const float t = (float)us * 1e-6f;

        // A "cliff" (large range) moving around the ring
        const int cliff_idx = ((int)(t * 1.0f)) % TOF_COUNT;  // 1 step/sec

        // A short invalid burst moving faster
        const int bad_idx = ((int)(t * 2.0f)) % TOF_COUNT;    // 2 steps/sec

        for (int i = 0; i < TOF_COUNT; i++) {
            float r = 0.15f + 0.02f * sinf(t * 2.5f + 0.7f * (float)i);
            uint8_t status = 0;
            bool valid = true;

            if (i == cliff_idx) {
                r = 1.20f + 0.20f * (0.5f + 0.5f * sinf(t * 1.5f));
            }

            if (i == bad_idx) {
                const float phase = fmodf(t, 0.5f);
                if (phase < 0.12f) {
                    status = 4; // "invalid range" style
                    valid = false;
                    r = NAN;
                }
            }

            r = clampf(r, 0.03f, 2.0f);
            update_one(i, valid, status, r);
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz provider update
    }
}

void tof_provider_init(void)
{
    ESP_LOGI(TAG, "Starting mock ToF provider task");
    BaseType_t ok = xTaskCreate(mock_task, "tof_mock", 4096, NULL, 5, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create mock ToF task");
    }
}
