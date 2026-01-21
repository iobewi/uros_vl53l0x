#include "tof_provider.h"

#include <math.h>
#include <string.h>

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

// État partagé (double buffer).
// Les writers ne modifient jamais le buffer "actif". Ils écrivent dans le buffer
// inactif, puis publient l’index actif de façon atomique. Le snapshot lit
// uniquement le buffer actif sans verrou (pas de blocage dans le callback timer).
static tof_sample_t g_tof_buffers[2][TOF_COUNT];
static volatile uint32_t g_tof_active_index = 0;
static portMUX_TYPE g_tof_mux = portMUX_INITIALIZER_UNLOCKED;

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

static void mark_all_invalid(uint8_t status)
{
    portENTER_CRITICAL(&g_tof_mux);
    uint32_t active = __atomic_load_n(&g_tof_active_index, __ATOMIC_RELAXED);
    uint32_t back = 1u - active;
    for (int i = 0; i < TOF_COUNT; i++) {
        g_tof_buffers[back][i] = g_tof_buffers[active][i];
        g_tof_buffers[back][i].valid = false;
        g_tof_buffers[back][i].status = status;
        g_tof_buffers[back][i].range_m = NAN;
        g_tof_buffers[back][i].seq++;
    }
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

    while (1) {
        VL53L0X_RangingMeasurementData_t data = {0};

        bool valid = false;
        uint8_t status = 255;
        float range_m = NAN;

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
            }
        } else {
            // gpio_ready timeout ou autre
            status = 251;
            valid = false;
            range_m = NAN;
        }

        update_one(ctx->idx, valid, status, range_m);

        // Petite respiration (évite starvations)
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void tof_provider_init(void)
{
    const tof_bus_config_t *bus_cfg = tof_get_bus_config();
    const tof_hw_config_t *hw_cfg = tof_get_hw_config();

    ESP_LOGI(TAG, "Init I2C bus...");
    esp_err_t err = vl53l0x_i2c_master_init(bus_cfg->sda_gpio, bus_cfg->scl_gpio,
                                            bus_cfg->i2c_freq_hz);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        mark_all_invalid(255);
        return;
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
        mark_all_invalid(255);
        return;
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
            mark_all_invalid(255);
            return;
        }

        // Active l’IRQ data-ready sur INT_i
        err = vl53l0x_enable_gpio_ready(&devs[i], hw_cfg[i].int_gpio, true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "GPIO ready init dev[%d] failed: %s", i, esp_err_to_name(err));
            mark_all_invalid(255);
            return;
        }

        // Démarre la mesure continue
        VL53L0X_Error st = VL53L0X_StartMeasurement(&devs[i].st);
        if (st != VL53L0X_ERROR_NONE) {
            ESP_LOGE(TAG, "StartMeasurement dev[%d] failed: %d", i, (int)st);
            mark_all_invalid(255);
            return;
        }

        // Init état
        update_one(i, false, 255, NAN);
    }

    SemaphoreHandle_t i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        mark_all_invalid(255);
        return;
    }

    static sensor_task_ctx_t ctx[TOF_COUNT];
    for (int i = 0; i < TOF_COUNT; i++) {
        ctx[i].idx = i;
        ctx[i].dev = &devs[i];
        ctx[i].i2c_mutex = i2c_mutex;
        ctx[i].timeout = pdMS_TO_TICKS(bus_cfg->gpio_ready_timeout_ms);

        char name[16];
        snprintf(name, sizeof(name), "vl53_%d", i);
        BaseType_t ok = xTaskCreate(sensor_task, name, 4096, &ctx[i], 5, NULL);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "Failed to create sensor task %s", name);
            mark_all_invalid(255);
            return;
        }
    }

    ESP_LOGI(TAG, "VL53 provider started (%d sensors)", TOF_COUNT);
}
