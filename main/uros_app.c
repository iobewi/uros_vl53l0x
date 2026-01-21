#include "uros_app.h"

#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

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

#define TIMER_PERIOD_MS CONFIG_MICRO_ROS_TIMER_PERIOD_MS
#define NUMBER_OF_HANDLES 1

static const char *TAG_TASK = "MICRO_ROS";
static const char *TAG_CB   = "TIMER_CB";

#define N_BINS CONFIG_MICRO_ROS_SCAN_BINS
#define ANGLE_MIN (-(float)M_PI)
#define ANGLE_INC ((2.0f * (float)M_PI) / (float)N_BINS)

static const char *SCAN_FRAME = CONFIG_MICRO_ROS_SCAN_FRAME_ID;

// Indices CAD-aligned (84 brins)
static const uint8_t TOF_BIN_IDX[TOF_COUNT] = {
    CONFIG_MICRO_ROS_TOF_BIN_IDX_0,
    CONFIG_MICRO_ROS_TOF_BIN_IDX_1,
    CONFIG_MICRO_ROS_TOF_BIN_IDX_2,
    CONFIG_MICRO_ROS_TOF_BIN_IDX_3,
    CONFIG_MICRO_ROS_TOF_BIN_IDX_4,
    CONFIG_MICRO_ROS_TOF_BIN_IDX_5,
    CONFIG_MICRO_ROS_TOF_BIN_IDX_6,
    CONFIG_MICRO_ROS_TOF_BIN_IDX_7
};

#include "led_status.h"

#define RCCHECK_GOTO(fn, label) {                                                                     \
    rcl_ret_t temp_rc = fn;                                                                           \
    if (temp_rc != RCL_RET_OK) {                                                                      \
        ESP_LOGE("RCCHECK", "Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc);      \
        led_status_set_state(LED_STATUS_ERROR);                                                       \
        goto label;                                                                                   \
    }                                                                                                 \
}

#define RCSOFTCHECK(fn) {                                                                             \
    rcl_ret_t temp_rc = fn;                                                                           \
    if (temp_rc != RCL_RET_OK) {                                                                      \
        ESP_LOGW("RCSOFTCHECK", "Failed status on line %d: %d. Continuing.", __LINE__, (int)temp_rc); \
    }                                                                                                 \
}

static rcl_publisher_t publisher;
static sensor_msgs__msg__LaserScan scan_msg;
static scan_config_t scan_cfg;

static int max_bin_index(void)
{
    int max_idx = 0;
    for (int i = 0; i < TOF_COUNT; i++) {
        if ((int)TOF_BIN_IDX[i] > max_idx) {
            max_idx = (int)TOF_BIN_IDX[i];
        }
    }
    return max_idx;
}

static void scan_msg_set_timestamp(sensor_msgs__msg__LaserScan *msg)
{
    const int64_t now_ns = rmw_uros_epoch_nanos();
    if (now_ns > 0) {
        msg->header.stamp.sec = (int32_t)(now_ns / 1000000000LL);
        msg->header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);
    } else {
        msg->header.stamp.sec = 0;
        msg->header.stamp.nanosec = 0;
    }
}

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) return;

    tof_sample_t snap[TOF_COUNT];
    tof_provider_snapshot(snap);

    scan_builder_fill(&scan_msg, &scan_cfg, snap, TOF_BIN_IDX);
    scan_msg_set_timestamp(&scan_msg);
    RCSOFTCHECK(rcl_publish(&publisher, &scan_msg, NULL));

    static uint32_t div = 0;
    if ((div++ % 10) == 0) {
        ESP_LOGI(TAG_CB, "Published %s (%d bins)", CONFIG_MICRO_ROS_TOPIC_NAME, N_BINS);
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

    while (true) {
        rcl_ret_t rc = RCL_RET_OK;
        bool scan_ready = false;
        bool node_ready = false;
        bool publisher_ready = false;
        bool executor_ready = false;
        bool timer_ready = false;
        bool init_options_ready = false;
        bool context_ready = false;

        rcl_allocator_t allocator = rcl_get_default_allocator();
        publisher = rcl_get_zero_initialized_publisher();
        rclc_support_t support = {0};
        rcl_node_t node = rcl_get_zero_initialized_node();
        rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
        rcl_timer_t timer = rcl_get_zero_initialized_timer();
        rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

        ESP_LOGI(TAG_TASK, "Waiting for micro-ROS agent...");
        led_status_set_state(LED_STATUS_WAITING);
        while (rmw_uros_ping_agent(100, 5) != RMW_RET_OK) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        ESP_LOGI(TAG_TASK, "Agent detected, starting RCL init");

        RCCHECK_GOTO(rcl_init_options_init(&init_options, allocator), cleanup);
        init_options_ready = true;
        RCCHECK_GOTO(rcl_init_options_set_domain_id(&init_options, CONFIG_MICRO_ROS_DOMAIN_ID), cleanup);
        RCCHECK_GOTO(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator), cleanup);
        context_ready = true;

        rcl_node_options_t node_ops = rcl_node_get_default_options();
        RCCHECK_GOTO(rclc_node_init_with_options(&node, CONFIG_MICRO_ROS_NODE_NAME, "", &support, &node_ops), cleanup);
        node_ready = true;

        RCCHECK_GOTO(rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
            CONFIG_MICRO_ROS_TOPIC_NAME),
            cleanup);
        publisher_ready = true;

        int configured_max = max_bin_index();
        if (configured_max >= scan_cfg.bins) {
            ESP_LOGE(TAG_TASK, "TOF bin map exceeds scan bins (max=%d bins=%d)",
                     configured_max, scan_cfg.bins);
            led_status_set_state(LED_STATUS_ERROR);
            goto cleanup;
        }

        if (!scan_builder_init(&scan_msg, &scan_cfg)) {
            ESP_LOGE(TAG_TASK, "scan_builder_init() failed (malloc?)");
            led_status_set_state(LED_STATUS_ERROR);
            goto cleanup;
        }
        scan_ready = true;

        RCCHECK_GOTO(rclc_timer_init_default2(
            &timer,
            &support,
            RCL_MS_TO_NS(TIMER_PERIOD_MS),
            timer_callback,
            true),
            cleanup);
        timer_ready = true;

        RCCHECK_GOTO(rclc_executor_init(&executor, &support.context, NUMBER_OF_HANDLES, &allocator), cleanup);
        executor_ready = true;
        RCCHECK_GOTO(rclc_executor_add_timer(&executor, &timer), cleanup);

        ESP_LOGI(TAG_TASK, "micro-ROS up: node=%s topic=%s period=%dms",
                 CONFIG_MICRO_ROS_NODE_NAME, CONFIG_MICRO_ROS_TOPIC_NAME, TIMER_PERIOD_MS);
        led_status_set_state(LED_STATUS_CONNECTED);

        uint32_t ping_div = 0;
        while (true) {
            rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
            if (spin_ret != RCL_RET_OK) {
                ESP_LOGE(TAG_TASK, "rclc_executor_spin_some() failed");
            }
            if ((ping_div++ % 20) == 0) {
                if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                    ESP_LOGW(TAG_TASK, "micro-ROS agent lost, restarting session");
                    led_status_set_state(LED_STATUS_WAITING);
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

cleanup:
        if (executor_ready) {
            rc = rclc_executor_fini(&executor);
            if (rc != RCL_RET_OK) {
                ESP_LOGW(TAG_TASK, "rclc_executor_fini() failed: %d", (int)rc);
            }
        }
        if (timer_ready) {
            rc = rcl_timer_fini(&timer);
            if (rc != RCL_RET_OK) {
                ESP_LOGW(TAG_TASK, "rcl_timer_fini() failed: %d", (int)rc);
            }
        }
        if (publisher_ready) {
            rc = rcl_publisher_fini(&publisher, &node);
            if (rc != RCL_RET_OK) {
                ESP_LOGW(TAG_TASK, "rcl_publisher_fini() failed: %d", (int)rc);
            }
        }
        if (node_ready) {
            rc = rcl_node_fini(&node);
            if (rc != RCL_RET_OK) {
                ESP_LOGW(TAG_TASK, "rcl_node_fini() failed: %d", (int)rc);
            }
        }
        if (context_ready) {
            rc = rcl_shutdown(&support.context);
            if (rc != RCL_RET_OK) {
                ESP_LOGW(TAG_TASK, "rcl_shutdown() failed: %d", (int)rc);
            }
        }
        if (context_ready) {
            rc = rclc_support_fini(&support);
            if (rc != RCL_RET_OK) {
                ESP_LOGW(TAG_TASK, "rclc_support_fini() failed: %d", (int)rc);
            }
        }
        if (init_options_ready) {
            rc = rcl_init_options_fini(&init_options);
            if (rc != RCL_RET_OK) {
                ESP_LOGW(TAG_TASK, "rcl_init_options_fini() failed: %d", (int)rc);
            }
        }
        if (scan_ready) {
            scan_builder_deinit(&scan_msg);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

bool uros_app_start(void)
{
#if !CONFIG_MICRO_ROS_STATUS_LED_ENABLE
    ESP_LOGW(TAG_TASK, "Status LED disabled (CONFIG_MICRO_ROS_STATUS_LED_ENABLE=n)");
#endif
    if (!led_status_start()) {
        ESP_LOGE(TAG_TASK, "Failed to start status LED task");
        return false;
    }
    led_status_set_state(LED_STATUS_WAITING);

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
