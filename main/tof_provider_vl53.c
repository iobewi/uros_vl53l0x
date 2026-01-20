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

static const char *TAG = "TOF_PROVIDER_VL53";

// ----------- Ajuste à ton PCB -----------
#define PIN_SDA GPIO_NUM_8
#define PIN_SCL GPIO_NUM_9
#define I2C_FREQ_HZ 400000

// 8 capteurs: XSHUT et INT
static const gpio_num_t XSHUT_PINS[TOF_COUNT] = {
    GPIO_NUM_3, GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7,
    GPIO_NUM_10, GPIO_NUM_11, GPIO_NUM_12, GPIO_NUM_13
};

static const gpio_num_t INT_PINS[TOF_COUNT] = {
    GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_14, GPIO_NUM_15,
    GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_19
};

// Adresses I2C 7 bits (0x29 est l’adresse par défaut)
static const uint8_t ADDR_7B[TOF_COUNT] = {
    0x2A, 0x2B, 0x2C, 0x2D,
    0x2E, 0x2F, 0x30, 0x31
};

// Timing budget / configuration capteur
#define TIMING_BUDGET_US 33000
#define GPIO_READY_TIMEOUT_MS 1000
// ---------------------------------------

// État partagé (identique au mock)
static tof_sample_t g_tof[TOF_COUNT];
static portMUX_TYPE g_tof_mux = portMUX_INITIALIZER_UNLOCKED;

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
    for (int i = 0; i < TOF_COUNT; i++) out[i] = g_tof[i];
    portEXIT_CRITICAL(&g_tof_mux);
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
    ESP_LOGI(TAG, "Init I2C bus...");
    esp_err_t err = vl53l0x_i2c_master_init(PIN_SDA, PIN_SCL, I2C_FREQ_HZ);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        return;
    }

    // Slots pour assignation d’adresses via XSHUT
    vl53l0x_slot_t slots[TOF_COUNT];
    for (int i = 0; i < TOF_COUNT; i++) {
        slots[i].xshut_gpio = XSHUT_PINS[i];
        slots[i].new_addr_7b = ADDR_7B[i];
    }

    ESP_LOGI(TAG, "Assign addresses (multi XSHUT)...");
    err = vl53l0x_multi_assign_addresses(slots, TOF_COUNT, 10);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Address assignment failed: %s", esp_err_to_name(err));
        return;
    }

    static vl53l0x_dev_t devs[TOF_COUNT];
    for (int i = 0; i < TOF_COUNT; i++) {
        devs[i].addr_7b = ADDR_7B[i];
    }

    ESP_LOGI(TAG, "Init %d devices...", TOF_COUNT);
    for (int i = 0; i < TOF_COUNT; i++) {
        err = vl53l0x_init(&devs[i], TIMING_BUDGET_US);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Init dev[%d] failed: %s", i, esp_err_to_name(err));
            return;
        }

        // Active l’IRQ data-ready sur INT_i
        err = vl53l0x_enable_gpio_ready(&devs[i], INT_PINS[i], true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "GPIO ready init dev[%d] failed: %s", i, esp_err_to_name(err));
            return;
        }

        // Démarre la mesure continue
        VL53L0X_Error st = VL53L0X_StartMeasurement(&devs[i].st);
        if (st != VL53L0X_ERROR_NONE) {
            ESP_LOGE(TAG, "StartMeasurement dev[%d] failed: %d", i, (int)st);
            return;
        }

        // Init état
        update_one(i, false, 255, NAN);
    }

    SemaphoreHandle_t i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        return;
    }

    static sensor_task_ctx_t ctx[TOF_COUNT];
    for (int i = 0; i < TOF_COUNT; i++) {
        ctx[i].idx = i;
        ctx[i].dev = &devs[i];
        ctx[i].i2c_mutex = i2c_mutex;
        ctx[i].timeout = pdMS_TO_TICKS(GPIO_READY_TIMEOUT_MS);

        char name[16];
        snprintf(name, sizeof(name), "vl53_%d", i);
        xTaskCreate(sensor_task, name, 4096, &ctx[i], 5, NULL);
    }

    ESP_LOGI(TAG, "VL53 provider started (%d sensors)", TOF_COUNT);
}
