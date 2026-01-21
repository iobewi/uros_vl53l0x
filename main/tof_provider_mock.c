#include "tof_provider.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

static const char *TAG = "TOF_PROVIDER_MOCK";

// État partagé (double buffer).
// Les writers ne modifient jamais le buffer "actif". Ils écrivent dans le buffer
// inactif, puis publient l’index actif de façon atomique. Le snapshot lit
// uniquement le buffer actif sans verrou (pas de blocage dans le callback timer).
static tof_sample_t g_tof_buffers[2][TOF_COUNT];
static volatile uint32_t g_tof_active_index = 0;
static portMUX_TYPE g_tof_mux = portMUX_INITIALIZER_UNLOCKED;

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline void update_one(int i, bool valid, uint8_t status, float range_m)
{
    portENTER_CRITICAL(&g_tof_mux);
    uint32_t active = __atomic_load_n(&g_tof_active_index, __ATOMIC_RELAXED);
    uint32_t back = 1u - active;
    for (int j = 0; j < TOF_COUNT; j++) {
        g_tof_buffers[back][j] = g_tof_buffers[active][j];
    }
    g_tof_buffers[back][i].valid = valid;
    g_tof_buffers[back][i].status = status;
    g_tof_buffers[back][i].range_m = range_m;
    g_tof_buffers[back][i].seq++;
    __atomic_store_n(&g_tof_active_index, back, __ATOMIC_RELEASE);
    portEXIT_CRITICAL(&g_tof_mux);
}

void tof_provider_snapshot(tof_sample_t out[TOF_COUNT])
{
    uint32_t active = __atomic_load_n(&g_tof_active_index, __ATOMIC_ACQUIRE);
    for (int i = 0; i < TOF_COUNT; i++) {
        out[i] = g_tof_buffers[active][i];
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
