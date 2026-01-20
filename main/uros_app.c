#include "uros_app.h"

#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/laser_scan.h>

#include "tof_provider.h"
#include "scan_builder.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DOMAIN_ID 42
#define TIMER_PERIOD_MS 100

#define NODE_NAME "tof_ring"
#define TOPIC_NAME "/tof_scan"
#define NUMBER_OF_HANDLES 1

static const char *TAG_TASK = "MICRO_ROS";
static const char *TAG_CB   = "TIMER_CB";

#define N_BINS 84
#define ANGLE_MIN (-(float)M_PI)
#define ANGLE_INC ((2.0f * (float)M_PI) / (float)N_BINS)

static const char *SCAN_FRAME = "base_link";

// Indices CAD-aligned (84 brins)
static const uint8_t TOF_BIN_IDX[TOF_COUNT] = {
    5, 13, 29, 37, 47, 55, 71, 79
};

#include "led_status.h"

#define RCCHECK(fn) {                                                                                \
    rcl_ret_t temp_rc = fn;                                                                          \
    if (temp_rc != RCL_RET_OK) {                                                                     \
        ESP_LOGE("RCCHECK", "Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc);     \
        led_status_set_state(LED_STATUS_ERROR);                                                      \
        vTaskDelete(NULL);                                                                           \
    }                                                                                                \
}

#define RCSOFTCHECK(fn) {                                                                             \
    rcl_ret_t temp_rc = fn;                                                                           \
    if (temp_rc != RCL_RET_OK) {                                                                      \
        ESP_LOGW("RCSOFTCHECK", "Failed status on line %d: %d. Continuing.", __LINE__, (int)temp_rc);\
    }                                                                                                 \
}

static rcl_publisher_t publisher;
static sensor_msgs__msg__LaserScan scan_msg;
static scan_config_t scan_cfg;

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) return;

    tof_sample_t snap[TOF_COUNT];
    tof_provider_snapshot(snap);

    scan_builder_fill(&scan_msg, &scan_cfg, snap, TOF_BIN_IDX);
    RCSOFTCHECK(rcl_publish(&publisher, &scan_msg, NULL));

    static uint32_t div = 0;
    if ((div++ % 10) == 0) {
        ESP_LOGI(TAG_CB, "Published %s (%d bins)", TOPIC_NAME, N_BINS);
    }
}

static void micro_ros_task(void *arg)
{
    (void)arg;

    scan_cfg.angle_min = ANGLE_MIN;
    scan_cfg.angle_inc = ANGLE_INC;
    scan_cfg.bins      = N_BINS;
    scan_cfg.range_min = 0.03f;
    scan_cfg.range_max = 2.00f;
    scan_cfg.frame_id  = SCAN_FRAME;

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support = {0};

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, DOMAIN_ID));

    ESP_LOGI(TAG_TASK, "Waiting for micro-ROS agent...");
    led_status_set_state(LED_STATUS_WAITING);
    while (rmw_uros_ping_agent(100, 5) != RMW_RET_OK) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG_TASK, "Agent detected, starting RCL init");

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_ops = rcl_node_get_default_options();
    RCCHECK(rclc_node_init_with_options(&node, NODE_NAME, "", &support, &node_ops));

    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        TOPIC_NAME));

    if (!scan_builder_init(&scan_msg, &scan_cfg)) {
        ESP_LOGE(TAG_TASK, "scan_builder_init() failed (malloc?)");
        led_status_set_state(LED_STATUS_ERROR);
        scan_builder_deinit(&scan_msg);
        vTaskDelete(NULL);
    }

    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(TIMER_PERIOD_MS),
        timer_callback,
        true));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, NUMBER_OF_HANDLES, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    ESP_LOGI(TAG_TASK, "micro-ROS up: node=%s topic=%s period=%dms",
             NODE_NAME, TOPIC_NAME, TIMER_PERIOD_MS);
    led_status_set_state(LED_STATUS_CONNECTED);

    while (true) {
        rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
        if (spin_ret != RCL_RET_OK) {
            ESP_LOGE(TAG_TASK, "rclc_executor_spin_some() failed");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool uros_app_start(void)
{
    if (!led_status_start()) {
        ESP_LOGE(TAG_TASK, "Failed to start status LED task");
        return false;
    }

    TaskHandle_t task_handle = NULL;

    BaseType_t ok = xTaskCreatePinnedToCore(
        micro_ros_task,
        "uros_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        &task_handle,
        1
    );

    return ok == pdPASS && task_handle != NULL;
}
