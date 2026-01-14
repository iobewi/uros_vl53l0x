#include "tof_provider.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

static const char *TAG = "TOF_PROVIDER_MOCK";

static tof_sample_t g_tof[TOF_COUNT];
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
    g_tof[i].valid = valid;
    g_tof[i].status = status;
    g_tof[i].range_m = range_m;
    g_tof[i].seq++;
    portEXIT_CRITICAL(&g_tof_mux);
}

void tof_provider_snapshot(tof_sample_t out[TOF_COUNT])
{
    portENTER_CRITICAL(&g_tof_mux);
    for (int i = 0; i < TOF_COUNT; i++) {
        out[i] = g_tof[i];
    }
    portEXIT_CRITICAL(&g_tof_mux);
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
    xTaskCreate(mock_task, "tof_mock", 4096, NULL, 5, NULL);
}
