#include "embedded_log.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/portmacro.h"
#include "rcl/error_handling.h"
#include "rmw_microros/rmw_microros.h"
#include "rclc/rclc.h"
#include "rmw/qos_profiles.h"
#include "std_msgs/msg/string.h"

#define EMBEDDED_LOG_TOPIC "/embedded/log"
#define EMBEDDED_LOG_TAG_MAX 24
#define EMBEDDED_LOG_MSG_MAX 160
#define EMBEDDED_LOG_LINE_MAX 256
#define EMBEDDED_LOG_BUFFER_SIZE 8

#if CONFIG_MICRO_ROS_LOG_ENABLE
static const char *TAG = "EMBEDDED_LOG";

#if CONFIG_MICRO_ROS_LOG_LEVEL_DEBUG
#define EMBEDDED_LOG_LEVEL_MIN EMBEDDED_LOG_LEVEL_DEBUG
#elif CONFIG_MICRO_ROS_LOG_LEVEL_INFO
#define EMBEDDED_LOG_LEVEL_MIN EMBEDDED_LOG_LEVEL_INFO
#elif CONFIG_MICRO_ROS_LOG_LEVEL_WARN
#define EMBEDDED_LOG_LEVEL_MIN EMBEDDED_LOG_LEVEL_WARN
#elif CONFIG_MICRO_ROS_LOG_LEVEL_ERROR
#define EMBEDDED_LOG_LEVEL_MIN EMBEDDED_LOG_LEVEL_ERROR
#else
#define EMBEDDED_LOG_LEVEL_MIN EMBEDDED_LOG_LEVEL_FATAL
#endif

typedef struct {
    uint64_t stamp_ns;
    embedded_log_level_t level;
    char tag[EMBEDDED_LOG_TAG_MAX];
    char msg[EMBEDDED_LOG_MSG_MAX];
} embedded_log_entry_t;

static rcl_publisher_t log_publisher;
static bool log_publisher_ready = false;
static bool log_msg_ready = false;
static SemaphoreHandle_t log_rcl_mutex = NULL;
static std_msgs__msg__String log_msg;
static char log_line_buffer[EMBEDDED_LOG_LINE_MAX];

static embedded_log_entry_t log_buffer[EMBEDDED_LOG_BUFFER_SIZE];
static size_t log_buffer_head = 0;
static size_t log_buffer_count = 0;
static portMUX_TYPE log_buffer_lock = portMUX_INITIALIZER_UNLOCKED;

static uint64_t last_throttle_second = 0;
static uint32_t throttle_count = 0;

static const char *embedded_log_level_str(embedded_log_level_t level)
{
    switch (level) {
        case EMBEDDED_LOG_LEVEL_DEBUG:
            return "DEBUG";
        case EMBEDDED_LOG_LEVEL_INFO:
            return "INFO";
        case EMBEDDED_LOG_LEVEL_WARN:
            return "WARN";
        case EMBEDDED_LOG_LEVEL_ERROR:
            return "ERROR";
        case EMBEDDED_LOG_LEVEL_FATAL:
            return "FATAL";
        default:
            return "UNKNOWN";
    }
}

static uint64_t embedded_log_stamp_ns(void)
{
    uint64_t stamp_ns = (uint64_t)rmw_uros_epoch_nanos();
    if (stamp_ns == 0) {
        stamp_ns = (uint64_t)esp_timer_get_time() * 1000ULL;
    }
    return stamp_ns;
}

static bool embedded_log_should_emit(embedded_log_level_t level)
{
    if (level < EMBEDDED_LOG_LEVEL_MIN) {
        return false;
    }
    if (CONFIG_MICRO_ROS_LOG_THROTTLE_PER_SEC <= 0) {
        return true;
    }
    uint64_t now_us = (uint64_t)esp_timer_get_time();
    uint64_t now_sec = now_us / 1000000ULL;
    if (now_sec != last_throttle_second) {
        last_throttle_second = now_sec;
        throttle_count = 0;
    }
    if (throttle_count >= (uint32_t)CONFIG_MICRO_ROS_LOG_THROTTLE_PER_SEC) {
        return false;
    }
    throttle_count++;
    return true;
}

static void embedded_log_buffer_push(const embedded_log_entry_t *entry)
{
    portENTER_CRITICAL(&log_buffer_lock);
    log_buffer[log_buffer_head] = *entry;
    log_buffer_head = (log_buffer_head + 1) % EMBEDDED_LOG_BUFFER_SIZE;
    if (log_buffer_count < EMBEDDED_LOG_BUFFER_SIZE) {
        log_buffer_count++;
    }
    portEXIT_CRITICAL(&log_buffer_lock);
}

static bool embedded_log_buffer_pop(embedded_log_entry_t *entry)
{
    bool has_entry = false;
    portENTER_CRITICAL(&log_buffer_lock);
    if (log_buffer_count > 0) {
        size_t tail = (log_buffer_head + EMBEDDED_LOG_BUFFER_SIZE - log_buffer_count) % EMBEDDED_LOG_BUFFER_SIZE;
        *entry = log_buffer[tail];
        log_buffer_count--;
        has_entry = true;
    }
    portEXIT_CRITICAL(&log_buffer_lock);
    return has_entry;
}

static void embedded_log_publish_entry(const embedded_log_entry_t *entry)
{
    uint32_t sec = (uint32_t)(entry->stamp_ns / 1000000000ULL);
    uint32_t nsec = (uint32_t)(entry->stamp_ns % 1000000000ULL);
    int len = snprintf(log_line_buffer,
                       sizeof(log_line_buffer),
                       "stamp=%" PRIu32 ".%09" PRIu32 " level=%u(%s) tag=%s msg=%s",
                       sec,
                       nsec,
                       (unsigned)entry->level,
                       embedded_log_level_str(entry->level),
                       entry->tag,
                       entry->msg);
    if (len < 0) {
        return;
    }
    if ((size_t)len >= sizeof(log_line_buffer)) {
        len = (int)(sizeof(log_line_buffer) - 1);
        log_line_buffer[len] = '\0';
    }

    log_msg.data.data = log_line_buffer;
    log_msg.data.size = (size_t)len;
    log_msg.data.capacity = sizeof(log_line_buffer);

    if (log_rcl_mutex != NULL) {
        xSemaphoreTake(log_rcl_mutex, portMAX_DELAY);
    }
    rcl_ret_t rc = rcl_publish(&log_publisher, &log_msg, NULL);
    if (rc != RCL_RET_OK) {
        rcl_error_string_t err = rcl_get_error_string();
        ESP_LOGW(TAG, "rcl_publish() failed: %d (%s)", (int)rc, err.str);
        rcl_reset_error();
    }
    if (log_rcl_mutex != NULL) {
        xSemaphoreGive(log_rcl_mutex);
    }
}

bool embedded_log_init(rcl_node_t *node, SemaphoreHandle_t rcl_mutex)
{
    if (node == NULL) {
        return false;
    }

    log_publisher = rcl_get_zero_initialized_publisher();
    log_rcl_mutex = rcl_mutex;

    rmw_qos_profile_t log_qos = rmw_qos_profile_default;
    log_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    log_qos.depth = 5;
    log_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    log_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    uint32_t heap_free = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    uint32_t heap_min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "Init log publisher topic=%s heap_free=%" PRIu32 " heap_min_free=%" PRIu32,
             EMBEDDED_LOG_TOPIC, heap_free, heap_min_free);

    rcl_ret_t rc = rclc_publisher_init(
        &log_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        EMBEDDED_LOG_TOPIC,
        &log_qos);
    if (rc != RCL_RET_OK) {
        rcl_error_string_t err = rcl_get_error_string();
        ESP_LOGW(TAG, "rclc_publisher_init() failed: %d (%s)", (int)rc, err.str);
        rcl_reset_error();
        return false;
    }

    std_msgs__msg__String__init(&log_msg);
    log_msg_ready = true;
    log_publisher_ready = true;

    embedded_log_entry_t entry;
    while (embedded_log_buffer_pop(&entry)) {
        embedded_log_publish_entry(&entry);
    }

    return true;
}

void embedded_log_deinit(rcl_node_t *node)
{
    if (!log_publisher_ready || node == NULL) {
        return;
    }

    rcl_ret_t rc = rcl_publisher_fini(&log_publisher, node);
    if (rc != RCL_RET_OK) {
        rcl_error_string_t err = rcl_get_error_string();
        ESP_LOGW(TAG, "rcl_publisher_fini() failed: %d (%s)", (int)rc, err.str);
        rcl_reset_error();
    }
    if (log_msg_ready) {
        std_msgs__msg__String__fini(&log_msg);
        log_msg_ready = false;
    }
    log_publisher_ready = false;
}

void embedded_log_write(embedded_log_level_t level, const char *tag, const char *fmt, ...)
{
    if (!embedded_log_should_emit(level) || tag == NULL || fmt == NULL) {
        return;
    }

    embedded_log_entry_t entry = {0};
    entry.stamp_ns = embedded_log_stamp_ns();
    entry.level = level;
    snprintf(entry.tag, sizeof(entry.tag), "%s", tag);

    va_list args;
    va_start(args, fmt);
    vsnprintf(entry.msg, sizeof(entry.msg), fmt, args);
    va_end(args);

    if (!log_publisher_ready) {
        embedded_log_buffer_push(&entry);
        return;
    }

    embedded_log_publish_entry(&entry);
}

#else

bool embedded_log_init(rcl_node_t *node, SemaphoreHandle_t rcl_mutex)
{
    (void)node;
    (void)rcl_mutex;
    return true;
}

void embedded_log_deinit(rcl_node_t *node)
{
    (void)node;
}

void embedded_log_write(embedded_log_level_t level, const char *tag, const char *fmt, ...)
{
    (void)level;
    (void)tag;
    (void)fmt;
}

#endif
