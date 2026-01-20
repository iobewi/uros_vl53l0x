#include "led_status.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_MICRO_ROS_STATUS_LED_ENABLE
#include "driver/led_strip.h"

static const char *TAG = "LED_STATUS";

static volatile led_status_state_t led_status_state = LED_STATUS_OFF;
static TaskHandle_t led_status_task_handle = NULL;
static led_strip_handle_t led_strip = NULL;

static uint8_t scale_brightness(uint8_t value)
{
    uint32_t scaled = (uint32_t)value * (uint32_t)CONFIG_MICRO_ROS_STATUS_LED_BRIGHTNESS;
    return (uint8_t)(scaled / 255U);
}

static void led_status_apply_color(uint8_t red, uint8_t green, uint8_t blue)
{
    if (led_strip == NULL) {
        return;
    }

    led_strip_set_pixel(led_strip, 0,
                        scale_brightness(red),
                        scale_brightness(green),
                        scale_brightness(blue));
    led_strip_refresh(led_strip);
}

static void led_status_task(void *arg)
{
    (void)arg;

    while (true) {
        led_status_state_t state = led_status_state;
        switch (state) {
            case LED_STATUS_WAITING:
                led_status_apply_color(LED_STATUS_COLOR_WAITING[0],
                                       LED_STATUS_COLOR_WAITING[1],
                                       LED_STATUS_COLOR_WAITING[2]);
                vTaskDelay(pdMS_TO_TICKS(500));
                led_status_apply_color(0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
                break;
            case LED_STATUS_CONNECTED:
                led_status_apply_color(LED_STATUS_COLOR_CONNECTED[0],
                                       LED_STATUS_COLOR_CONNECTED[1],
                                       LED_STATUS_COLOR_CONNECTED[2]);
                vTaskDelay(pdMS_TO_TICKS(200));
                break;
            case LED_STATUS_ERROR:
                led_status_apply_color(LED_STATUS_COLOR_ERROR[0],
                                       LED_STATUS_COLOR_ERROR[1],
                                       LED_STATUS_COLOR_ERROR[2]);
                vTaskDelay(pdMS_TO_TICKS(150));
                led_status_apply_color(0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(150));
                break;
            case LED_STATUS_OFF:
            default:
                led_status_apply_color(0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(250));
                break;
        }
    }
}

bool led_status_start(void)
{
    if (led_status_task_handle != NULL) {
        return true;
    }

    led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_MICRO_ROS_STATUS_LED_GPIO,
        .max_leds = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_SK6812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags.with_dma = false,
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init LED strip: %s", esp_err_to_name(err));
        return false;
    }

    BaseType_t led_ok = xTaskCreate(
        led_status_task,
        "uros_led",
        2048,
        NULL,
        1,
        &led_status_task_handle);
    if (led_ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LED status task");
        return false;
    }

    led_status_set_state(LED_STATUS_OFF);
    return true;
}

void led_status_set_state(led_status_state_t state)
{
    led_status_state = state;
}
#else
bool led_status_start(void)
{
    return true;
}

void led_status_set_state(led_status_state_t state)
{
    (void)state;
}
#endif
