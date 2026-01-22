#include "embedded_metrics.h"

#include <inttypes.h>
#include <stdio.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "rcl/error_handling.h"
#include "rmw_microros/rmw_microros.h"
#include "rclc/rclc.h"
#include "rmw/qos_profiles.h"
#include "std_msgs/msg/string.h"

#define EMBEDDED_METRICS_TOPIC "/embedded/metrics"
#define EMBEDDED_METRICS_LINE_MAX 256

#if CONFIG_MICRO_ROS_METRICS_ENABLE
static const char *TAG = "EMBEDDED_METRICS";

static rcl_publisher_t metrics_publisher;
static rcl_timer_t metrics_timer;
static bool metrics_publisher_ready = false;
static bool metrics_timer_ready = false;
static bool metrics_msg_ready = false;
static SemaphoreHandle_t metrics_rcl_mutex = NULL;
static std_msgs__msg__String metrics_msg;
static char metrics_line_buffer[EMBEDDED_METRICS_LINE_MAX];

static portMUX_TYPE metrics_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t metrics_loop_dt_us = 0;
static uint32_t metrics_xrce_reconnect_count = 0;
static uint32_t metrics_pub_fail_count = 0;

static uint64_t embedded_metrics_stamp_ns(void)
{
    uint64_t stamp_ns = (uint64_t)rmw_uros_epoch_nanos();
    if (stamp_ns == 0) {
        stamp_ns = (uint64_t)esp_timer_get_time() * 1000ULL;
    }
    return stamp_ns;
}

static void embedded_metrics_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) {
        return;
    }

    uint32_t heap_free = (uint32_t)xPortGetFreeHeapSize();
    uint32_t heap_min_free = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);

    uint32_t loop_dt_us = 0;
    uint32_t reconnect_count = 0;
    uint32_t pub_fail_count = 0;

    portENTER_CRITICAL(&metrics_lock);
    loop_dt_us = metrics_loop_dt_us;
    reconnect_count = metrics_xrce_reconnect_count;
    pub_fail_count = metrics_pub_fail_count;
    portEXIT_CRITICAL(&metrics_lock);

    uint64_t stamp_ns = embedded_metrics_stamp_ns();
    uint32_t sec = (uint32_t)(stamp_ns / 1000000000ULL);
    uint32_t nsec = (uint32_t)(stamp_ns % 1000000000ULL);

    int len = snprintf(metrics_line_buffer,
                       sizeof(metrics_line_buffer),
                       "stamp=%" PRIu32 ".%09" PRIu32
                       " heap_free_bytes=%" PRIu32
                       " heap_min_free_bytes=%" PRIu32
                       " loop_dt_us=%" PRIu32
                       " xrce_reconnect_count=%" PRIu32
                       " pub_fail_count=%" PRIu32,
                       sec,
                       nsec,
                       heap_free,
                       heap_min_free,
                       loop_dt_us,
                       reconnect_count,
                       pub_fail_count);
    if (len < 0) {
        return;
    }
    if ((size_t)len >= sizeof(metrics_line_buffer)) {
        len = (int)(sizeof(metrics_line_buffer) - 1);
        metrics_line_buffer[len] = '\0';
    }

    metrics_msg.data.data = metrics_line_buffer;
    metrics_msg.data.size = (size_t)len;
    metrics_msg.data.capacity = sizeof(metrics_line_buffer);

    if (metrics_rcl_mutex != NULL) {
        xSemaphoreTake(metrics_rcl_mutex, portMAX_DELAY);
    }
    rcl_ret_t pub_rc = rcl_publish(&metrics_publisher, &metrics_msg, NULL);
    if (pub_rc != RCL_RET_OK) {
        rcl_error_string_t err = rcl_get_error_string();
        ESP_LOGW(TAG, "rcl_publish() failed: %d (%s)", (int)pub_rc, err.str);
        rcl_reset_error();
    }
    if (metrics_rcl_mutex != NULL) {
        xSemaphoreGive(metrics_rcl_mutex);
    }
}

bool embedded_metrics_init(rcl_node_t *node,
                           rclc_executor_t *executor,
                           rclc_support_t *support,
                           SemaphoreHandle_t rcl_mutex)
{
    if (node == NULL || executor == NULL || support == NULL) {
        return false;
    }

    metrics_publisher = rcl_get_zero_initialized_publisher();
    metrics_timer = rcl_get_zero_initialized_timer();
    metrics_rcl_mutex = rcl_mutex;

    rmw_qos_profile_t metrics_qos = rmw_qos_profile_default;
    metrics_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    metrics_qos.depth = 5;
    metrics_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    metrics_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    uint32_t heap_free = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    uint32_t heap_min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "Init metrics publisher topic=%s heap_free=%" PRIu32 " heap_min_free=%" PRIu32,
             EMBEDDED_METRICS_TOPIC, heap_free, heap_min_free);

    rcl_ret_t rc = rclc_publisher_init(
        &metrics_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        EMBEDDED_METRICS_TOPIC,
        &metrics_qos);
    if (rc != RCL_RET_OK) {
        rcl_error_string_t err = rcl_get_error_string();
        ESP_LOGW(TAG, "rclc_publisher_init() failed: %d (%s)", (int)rc, err.str);
        rcl_reset_error();
        return false;
    }
    metrics_publisher_ready = true;

    std_msgs__msg__String__init(&metrics_msg);
    metrics_msg_ready = true;

    rc = rclc_timer_init_default2(
        &metrics_timer,
        support,
        RCL_MS_TO_NS(CONFIG_MICRO_ROS_METRICS_PERIOD_MS),
        embedded_metrics_timer_callback,
        true);
    if (rc != RCL_RET_OK) {
        rcl_error_string_t err = rcl_get_error_string();
        ESP_LOGW(TAG, "rclc_timer_init_default2() failed: %d (%s)", (int)rc, err.str);
        rcl_reset_error();
        rcl_ret_t fini_rc = rcl_publisher_fini(&metrics_publisher, node);
        if (fini_rc != RCL_RET_OK) {
            rcl_error_string_t fini_err = rcl_get_error_string();
            ESP_LOGW(TAG, "rcl_publisher_fini() failed: %d (%s)", (int)fini_rc, fini_err.str);
            rcl_reset_error();
        }
        metrics_publisher_ready = false;
        std_msgs__msg__String__fini(&metrics_msg);
        metrics_msg_ready = false;
        return false;
    }
    metrics_timer_ready = true;

    rc = rclc_executor_add_timer(executor, &metrics_timer);
    if (rc != RCL_RET_OK) {
        rcl_error_string_t err = rcl_get_error_string();
        ESP_LOGW(TAG, "rclc_executor_add_timer() failed: %d (%s)", (int)rc, err.str);
        rcl_reset_error();
        rcl_ret_t fini_rc = rcl_timer_fini(&metrics_timer);
        if (fini_rc != RCL_RET_OK) {
            rcl_error_string_t fini_err = rcl_get_error_string();
            ESP_LOGW(TAG, "rcl_timer_fini() failed: %d (%s)", (int)fini_rc, fini_err.str);
            rcl_reset_error();
        }
        metrics_timer_ready = false;
        fini_rc = rcl_publisher_fini(&metrics_publisher, node);
        if (fini_rc != RCL_RET_OK) {
            rcl_error_string_t fini_err = rcl_get_error_string();
            ESP_LOGW(TAG, "rcl_publisher_fini() failed: %d (%s)", (int)fini_rc, fini_err.str);
            rcl_reset_error();
        }
        metrics_publisher_ready = false;
        std_msgs__msg__String__fini(&metrics_msg);
        metrics_msg_ready = false;
        return false;
    }

    return true;
}

void embedded_metrics_deinit(rcl_node_t *node)
{
    if (metrics_timer_ready) {
        rcl_ret_t rc = rcl_timer_fini(&metrics_timer);
        if (rc != RCL_RET_OK) {
            rcl_error_string_t err = rcl_get_error_string();
            ESP_LOGW(TAG, "rcl_timer_fini() failed: %d (%s)", (int)rc, err.str);
            rcl_reset_error();
        }
        metrics_timer_ready = false;
    }
    if (metrics_publisher_ready && node != NULL) {
        rcl_ret_t rc = rcl_publisher_fini(&metrics_publisher, node);
        if (rc != RCL_RET_OK) {
            rcl_error_string_t err = rcl_get_error_string();
            ESP_LOGW(TAG, "rcl_publisher_fini() failed: %d (%s)", (int)rc, err.str);
            rcl_reset_error();
        }
        metrics_publisher_ready = false;
    }
    if (metrics_msg_ready) {
        std_msgs__msg__String__fini(&metrics_msg);
        metrics_msg_ready = false;
    }
}

void embedded_metrics_set_loop_dt_us(uint32_t loop_dt_us)
{
    portENTER_CRITICAL(&metrics_lock);
    metrics_loop_dt_us = loop_dt_us;
    portEXIT_CRITICAL(&metrics_lock);
}

void embedded_metrics_set_xrce_reconnect_count(uint32_t count)
{
    portENTER_CRITICAL(&metrics_lock);
    metrics_xrce_reconnect_count = count;
    portEXIT_CRITICAL(&metrics_lock);
}

void embedded_metrics_set_pub_fail_count(uint32_t count)
{
    portENTER_CRITICAL(&metrics_lock);
    metrics_pub_fail_count = count;
    portEXIT_CRITICAL(&metrics_lock);
}

#else

bool embedded_metrics_init(rcl_node_t *node,
                           rclc_executor_t *executor,
                           rclc_support_t *support,
                           SemaphoreHandle_t rcl_mutex)
{
    (void)node;
    (void)executor;
    (void)support;
    (void)rcl_mutex;
    return true;
}

void embedded_metrics_deinit(rcl_node_t *node)
{
    (void)node;
}

void embedded_metrics_set_loop_dt_us(uint32_t loop_dt_us)
{
    (void)loop_dt_us;
}

void embedded_metrics_set_xrce_reconnect_count(uint32_t count)
{
    (void)count;
}

void embedded_metrics_set_pub_fail_count(uint32_t count)
{
    (void)count;
}

#endif
