#include "tof_provider.h"

#include <math.h>
#include <inttypes.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "vl53l0x.h"   // ESP-IDF_VL53L0X

#include "tof_config.h"

static const char *TAG = "TOF_PROVIDER_VL53";
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

static void mark_all_invalid(uint8_t status)
{
    for (int i = 0; i < TOF_COUNT; i++) {
        update_one(i, false, status, NAN);
    }
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

// Contexte task capteur
typedef struct {
    int idx;
    vl53l0x_dev_t *dev;
    SemaphoreHandle_t i2c_mutex;
    TickType_t timeout;
} sensor_task_ctx_t;

static void sensor_task(void *arg)
{
    sensor_task_ctx_t *ctx = (sensor_task_ctx_t *)arg;
    uint32_t consecutive_errors = 0;
    const uint32_t backoff_base_ms = 5;
    const uint32_t backoff_max_ms = 100;

    while (1) {
        VL53L0X_RangingMeasurementData_t data = {0};

        bool valid = false;
        uint8_t status = 255;
        float range_m = NAN;
        bool gpio_timeout = false;
        bool i2c_timeout = false;

        // Attend l’IRQ "data ready" via GPIO
        esp_err_t err = vl53l0x_wait_gpio_ready(ctx->dev, ctx->timeout);
        if (err == ESP_OK) {
            if (xSemaphoreTake(ctx->i2c_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                VL53L0X_Error st = VL53L0X_GetRangingMeasurementData(&ctx->dev->st, &data);
                if (st == VL53L0X_ERROR_NONE) {
                    status = data.RangeStatus;
                    if (status == 0) {
                        valid = true;
                        range_m = (float)data.RangeMilliMeter * 0.001f;
                    }
                }
                // Acknowledge IRQ
                (void)VL53L0X_ClearInterruptMask(&ctx->dev->st, 0);
                xSemaphoreGive(ctx->i2c_mutex);
            } else {
                // mutex I2C timeout -> invalide
                status = 250;
                valid = false;
                range_m = NAN;
                i2c_timeout = true;
            }
        } else {
            // gpio_ready timeout ou autre
            status = 251;
            valid = false;
            range_m = NAN;
            gpio_timeout = true;
        }

        update_one(ctx->idx, valid, status, range_m);

        if (gpio_timeout || i2c_timeout) {
            consecutive_errors++;
        } else {
            consecutive_errors = 0;
        }

        TickType_t delay_ticks = pdMS_TO_TICKS(2);
        if (consecutive_errors > 0) {
            uint32_t backoff_ms = backoff_base_ms;
            uint32_t shift = consecutive_errors - 1;
            if (shift < 31) {
                backoff_ms <<= shift;
            } else {
                backoff_ms = backoff_max_ms;
            }
            if (backoff_ms > backoff_max_ms) {
                backoff_ms = backoff_max_ms;
            }
            delay_ticks += pdMS_TO_TICKS(backoff_ms);
        }

        // Petite respiration (évite starvations)
        vTaskDelay(delay_ticks);
    }
}

void tof_provider_init(void)
{
    const tof_bus_config_t *bus_cfg = tof_get_bus_config();
    const tof_hw_config_t *hw_cfg = tof_get_hw_config();
    SemaphoreHandle_t i2c_mutex = NULL;
    TaskHandle_t task_handles[TOF_COUNT] = {0};
    bool init_ok = false;

    ESP_LOGI(TAG, "Init I2C bus...");
    esp_err_t err = vl53l0x_i2c_master_init(bus_cfg->sda_gpio, bus_cfg->scl_gpio,
                                            bus_cfg->i2c_freq_hz);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    // Slots pour assignation d’adresses via XSHUT
    vl53l0x_slot_t slots[TOF_COUNT];
    for (int i = 0; i < TOF_COUNT; i++) {
        slots[i].xshut_gpio = hw_cfg[i].xshut_gpio;
        slots[i].new_addr_7b = hw_cfg[i].addr_7b;
    }

    ESP_LOGI(TAG, "Assign addresses (multi XSHUT)...");
    err = vl53l0x_multi_assign_addresses(slots, TOF_COUNT, 10);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Address assignment failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    static vl53l0x_dev_t devs[TOF_COUNT];
    for (int i = 0; i < TOF_COUNT; i++) {
        devs[i].addr_7b = hw_cfg[i].addr_7b;
    }

    ESP_LOGI(TAG, "Init %d devices...", TOF_COUNT);
    for (int i = 0; i < TOF_COUNT; i++) {
        err = vl53l0x_init(&devs[i], bus_cfg->timing_budget_us);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Init dev[%d] failed: %s", i, esp_err_to_name(err));
            goto cleanup;
        }

        // Active l’IRQ data-ready sur INT_i
        err = vl53l0x_enable_gpio_ready(&devs[i], hw_cfg[i].int_gpio, true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "GPIO ready init dev[%d] failed: %s", i, esp_err_to_name(err));
            goto cleanup;
        }

        // Démarre la mesure continue
        VL53L0X_Error st = VL53L0X_StartMeasurement(&devs[i].st);
        if (st != VL53L0X_ERROR_NONE) {
            ESP_LOGE(TAG, "StartMeasurement dev[%d] failed: %d", i, (int)st);
            goto cleanup;
        }

        // Init état
        update_one(i, false, 255, NAN);
    }

    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        goto cleanup;
    }

    static sensor_task_ctx_t ctx[TOF_COUNT];
    for (int i = 0; i < TOF_COUNT; i++) {
        ctx[i].idx = i;
        ctx[i].dev = &devs[i];
        ctx[i].i2c_mutex = i2c_mutex;
        ctx[i].timeout = pdMS_TO_TICKS(bus_cfg->gpio_ready_timeout_ms);

        char name[16];
        snprintf(name, sizeof(name), "vl53_%d", i);
        BaseType_t ok = xTaskCreate(sensor_task, name, 4096, &ctx[i], 5,
                                    &task_handles[i]);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "Failed to create sensor task %s", name);
            goto cleanup;
        }
    }

    ESP_LOGI(TAG, "VL53 provider started (%d sensors)", TOF_COUNT);
    init_ok = true;

cleanup:
    if (!init_ok) {
        for (int i = 0; i < TOF_COUNT; i++) {
            if (task_handles[i] != NULL) {
                vTaskDelete(task_handles[i]);
                task_handles[i] = NULL;
            }
        }
        if (i2c_mutex != NULL) {
            vSemaphoreDelete(i2c_mutex);
            i2c_mutex = NULL;
        }
        mark_all_invalid(255);
    }
}
