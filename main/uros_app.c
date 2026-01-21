#include "uros_app.h"

#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>

#include "esp_log.h"
#if CONFIG_MICRO_ROS_SCAN_ALLOC_GUARD
#include "esp_heap_caps.h"
#endif
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <rmw/qos_profiles.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/laser_scan.h>

#include "tof_config.h"
#include "tof_provider.h"
#include "scan_builder.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#if defined(CONFIG_MICRO_ROS_QOS_RELIABLE)
#define MICRO_ROS_QOS_RELIABLE_ENABLED 1
#else
#define MICRO_ROS_QOS_RELIABLE_ENABLED 0
#endif

#define TIMER_PERIOD_MS CONFIG_MICRO_ROS_TIMER_PERIOD_MS
#define NUMBER_OF_HANDLES 1

static const char *TAG_TASK = "MICRO_ROS";
static const char *TAG_CB   = "TIMER_CB";

#define N_BINS CONFIG_MICRO_ROS_SCAN_BINS
#define ANGLE_MIN (-(float)M_PI)
#define ANGLE_INC ((2.0f * (float)M_PI) / (float)N_BINS)

static const char *SCAN_FRAME = CONFIG_MICRO_ROS_SCAN_FRAME_ID;

#include "led_status.h"

#define RCCHECK_GOTO(fn, label) {                                                                     \
    rcl_ret_t temp_rc = fn;                                                                           \
    if (temp_rc != RCL_RET_OK) {                                                                      \
        ESP_LOGE("RCCHECK", "Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc);      \
        led_status_set_state(LED_STATUS_ERROR);                                                       \
        goto label;                                                                                   \
    }                                                                                                 \
}

#define RCCHECK_FAIL_GOTO(fn, label, flag) {                                                          \
    rcl_ret_t temp_rc = fn;                                                                           \
    if (temp_rc != RCL_RET_OK) {                                                                      \
        ESP_LOGE("RCCHECK", "Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc);      \
        led_status_set_state(LED_STATUS_ERROR);                                                       \
        flag = true;                                                                                  \
        goto label;                                                                                   \
    }                                                                                                 \
}
#define RCSOFTCHECK(fn) {                                                                             \
    rcl_ret_t temp_rc = fn;                                                                           \
    if (temp_rc != RCL_RET_OK) {                                                                      \
        ESP_LOGW("RCSOFTCHECK", "Failed status on line %d: %d. Continuing.", __LINE__, (int)temp_rc); \
    }                                                                                                 \
}

#define TIME_SYNC_TIMEOUT_MS 1000U
#define TIME_SYNC_MAX_ATTEMPTS 5
#define ENTITY_DESTROY_TIMEOUT_MS 200U

#if CONFIG_MICRO_ROS_DEBUG_LOGS
#define MICROROS_LOG_PUBLISH_EVERY_N(divider, tag, fmt, ...)                           \
    do {                                                                               \
        static uint32_t microros_log_divider = 0;                                      \
        if ((divider) > 0 && ((microros_log_divider++ % (divider)) == 0)) {            \
            ESP_LOGD(tag, fmt, ##__VA_ARGS__);                                         \
        }                                                                              \
    } while (0)
#else
#define MICROROS_LOG_PUBLISH_EVERY_N(divider, tag, fmt, ...) \
    do {                                                    \
        (void)(divider);                                   \
        (void)(tag);                                       \
        (void)(fmt);                                       \
    } while (0)
#endif

__attribute__((weak)) rmw_ret_t rmw_uros_set_entity_destroy_session_timeout(int64_t timeout_ms);

static rcl_publisher_t publisher;
static sensor_msgs__msg__LaserScan scan_msg;
static scan_config_t scan_cfg;
static volatile uint32_t publish_failures = 0;
static volatile uint32_t agent_missed_pings = 0;
static volatile uint32_t congestion_detected = 0;
static volatile bool publish_error_burst = false;
static uint32_t publish_backoff_cycles = 0;
static TaskHandle_t scan_pub_task_handle = NULL;
static volatile bool scan_pub_task_enabled = false;
static volatile bool scan_pub_task_stop = false;

#if CONFIG_MICRO_ROS_SCAN_ALLOC_GUARD
typedef struct {
    size_t free_bytes;
    size_t largest_block;
} heap_guard_t;

static heap_guard_t heap_guard_begin(void)
{
    heap_guard_t guard = {
        .free_bytes = heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
        .largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT),
    };
    return guard;
}

static void heap_guard_end(heap_guard_t before, const char *label)
{
    const size_t free_after = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    const size_t largest_after = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    if (free_after != before.free_bytes || largest_after != before.largest_block) {
        ESP_LOGE(TAG_TASK,
                 "Heap changed during %s (free=%zu->%zu, largest=%zu->%zu)",
                 label, before.free_bytes, free_after, before.largest_block, largest_after);
    }
}
#endif

#define PUBLISH_BACKOFF_MAX_CYCLES 5U

static bool validate_tof_bin_map(const scan_config_t *cfg)
{
    const tof_hw_config_t *hw_cfg = tof_get_hw_config();
    ESP_LOGI(TAG_TASK, "TOF bin mapping (tof -> bin):");
    for (int i = 0; i < TOF_COUNT; i++) {
        ESP_LOGI(TAG_TASK, "  tof[%d] -> bin[%u]", i, (unsigned)hw_cfg[i].bin_idx);
    }

#if !CONFIG_TOF_BIN_ALLOW_DUPLICATES
    bool used[N_BINS] = {0};
#endif

    bool valid = true;
    for (int i = 0; i < TOF_COUNT; i++) {
        uint8_t idx = hw_cfg[i].bin_idx;
        if ((int)idx >= cfg->bins) {
            ESP_LOGE(TAG_TASK,
                     "TOF bin index out of range (tof=%d idx=%u bins=%d)",
                     i, (unsigned)idx, cfg->bins);
            valid = false;
            continue;
        }
#if !CONFIG_TOF_BIN_ALLOW_DUPLICATES
        if (used[idx]) {
            ESP_LOGE(TAG_TASK,
                     "Duplicate TOF bin index detected (tof=%d idx=%u)",
                     i, (unsigned)idx);
            valid = false;
        } else {
            used[idx] = true;
        }
#endif
    }

    return valid;
}

static void sync_time_with_agent(void)
{
    rmw_ret_t sync_ret = RMW_RET_ERROR;
    for (int attempt = 1; attempt <= TIME_SYNC_MAX_ATTEMPTS; attempt++) {
        sync_ret = rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
        if (sync_ret == RMW_RET_OK) {
            ESP_LOGI(TAG_TASK, "Time sync completed");
            return;
        }
        ESP_LOGW(TAG_TASK, "Time sync attempt %d/%d failed: %d",
                 attempt, TIME_SYNC_MAX_ATTEMPTS, (int)sync_ret);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGW(TAG_TASK, "Time sync failed; timestamps may be unset");
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

static void log_rcl_failure(const char *tag, const char *label, rcl_ret_t rc)
{
    rcl_error_string_t err = rcl_get_error_string();
    if (err.str[0] != '\0') {
        ESP_LOGW(tag, "%s failed: %d (%s)", label, (int)rc, err.str);
    } else {
        ESP_LOGW(tag, "%s failed: %d", label, (int)rc);
    }
    rcl_reset_error();
}

static void configure_entity_destroy_timeout(void)
{
    if (rmw_uros_set_entity_destroy_session_timeout != NULL) {
        rmw_ret_t ret = rmw_uros_set_entity_destroy_session_timeout(ENTITY_DESTROY_TIMEOUT_MS);
        if (ret != RMW_RET_OK) {
            ESP_LOGW(TAG_TASK,
                     "rmw_uros_set_entity_destroy_session_timeout() failed: %d",
                     (int)ret);
        }
    }
}

static void scan_pub_task(void *arg)
{
    (void)arg;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (scan_pub_task_stop) {
            break;
        }
        if (!scan_pub_task_enabled) {
            continue;
        }

        if (publish_backoff_cycles > 0) {
            publish_backoff_cycles--;
            continue;
        }

        tof_sample_t snap[TOF_COUNT];
        tof_provider_snapshot(snap);

#if CONFIG_MICRO_ROS_SCAN_ALLOC_GUARD
        heap_guard_t guard = heap_guard_begin();
#endif
        scan_builder_fill(&scan_msg, &scan_cfg, snap, tof_get_hw_config());
        scan_msg_set_timestamp(&scan_msg);
        rcl_ret_t pub_rc = rcl_publish(&publisher, &scan_msg, NULL);
        bool publish_ok = (pub_rc == RCL_RET_OK);
        if (pub_rc != RCL_RET_OK) {
            publish_failures++;
            if (publish_backoff_cycles < PUBLISH_BACKOFF_MAX_CYCLES) {
                publish_backoff_cycles++;
            }
            if (CONFIG_MICRO_ROS_PUBLISH_ERROR_LOG_DIVIDER > 0 &&
                (publish_failures % CONFIG_MICRO_ROS_PUBLISH_ERROR_LOG_DIVIDER) == 0) {
                rcl_error_string_t err = rcl_get_error_string();
                ESP_LOGW("RCSOFTCHECK", "rcl_publish() failed %u times (last=%d, reason=%s)",
                         (unsigned)publish_failures, (int)pub_rc,
                         err.str ? err.str : "unknown");
                rcl_reset_error();
            }
            if (publish_failures >= 50) {
                publish_error_burst = true;
            }
        } else if (publish_failures > 0) {
            publish_failures = 0;
            publish_error_burst = false;
            publish_backoff_cycles = 0;
        }

#if CONFIG_MICRO_ROS_SCAN_ALLOC_GUARD
        heap_guard_end(guard, "scan_publish_loop");
#endif
        if (publish_ok) {
            MICROROS_LOG_PUBLISH_EVERY_N(CONFIG_MICRO_ROS_PUBLISH_LOG_DIVIDER, TAG_CB,
                                         "Published %s (%d bins)", CONFIG_MICRO_ROS_TOPIC_NAME, N_BINS);
        }
    }

    scan_pub_task_handle = NULL;
    vTaskDelete(NULL);
}

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) return;

    if (scan_pub_task_handle != NULL) {
        xTaskNotifyGive(scan_pub_task_handle);
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
    scan_cfg.scan_time = (float)TIMER_PERIOD_MS / 1000.0f;
    scan_cfg.time_increment = 0.0f;
    scan_cfg.frame_id  = SCAN_FRAME;

    uint32_t consecutive_failures = 0;

    while (true) {
        rcl_ret_t rc = RCL_RET_OK;
        bool scan_ready = false;
        bool node_ready = false;
        bool publisher_ready = false;
        bool executor_ready = false;
        bool timer_ready = false;
        bool init_options_ready = false;
        bool context_ready = false;
        bool init_failed = false;

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

        RCCHECK_FAIL_GOTO(rcl_init_options_init(&init_options, allocator), cleanup, init_failed);
        init_options_ready = true;
        RCCHECK_FAIL_GOTO(rcl_init_options_set_domain_id(&init_options, CONFIG_MICRO_ROS_DOMAIN_ID), cleanup, init_failed);
        ESP_LOGI(TAG_TASK, "Initializing micro-ROS support...");
        RCCHECK_FAIL_GOTO(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator), cleanup, init_failed);
        context_ready = true;
        sync_time_with_agent();

        rcl_node_options_t node_ops = rcl_node_get_default_options();
        ESP_LOGI(TAG_TASK, "Creating node '%s'...", CONFIG_MICRO_ROS_NODE_NAME);
        RCCHECK_FAIL_GOTO(rclc_node_init_with_options(&node, CONFIG_MICRO_ROS_NODE_NAME, "", &support, &node_ops),
                          cleanup, init_failed);
        node_ready = true;

        ESP_LOGI(TAG_TASK, "Creating publisher '%s'...", CONFIG_MICRO_ROS_TOPIC_NAME);
        // LaserScan QoS: BEST_EFFORT by default (configurable to RELIABLE) + KEEP_LAST(depth=1)
        // + VOLATILE to avoid XRCE serial backpressure.
        rmw_qos_profile_t scan_qos = rmw_qos_profile_default;
        scan_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        scan_qos.depth = 1;
        scan_qos.reliability = MICRO_ROS_QOS_RELIABLE_ENABLED
                                   ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
                                   : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        scan_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
        const int max_pub_attempts = 5;
        for (int attempt = 1; attempt <= max_pub_attempts; attempt++) {
            rc = rclc_publisher_init(
                &publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
                CONFIG_MICRO_ROS_TOPIC_NAME,
                &scan_qos);
            if (rc == RCL_RET_OK) {
                publisher_ready = true;
                break;
            }
            ESP_LOGW(TAG_TASK, "Publisher init attempt %d/%d failed: %d",
                     attempt, max_pub_attempts, (int)rc);
            if (attempt < max_pub_attempts) {
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }
        if (!publisher_ready) {
            ESP_LOGE("RCCHECK", "Failed status on line %d: %d. Aborting.", __LINE__, (int)rc);
            led_status_set_state(LED_STATUS_ERROR);
            init_failed = true;
            goto cleanup;
        }

        if (!validate_tof_bin_map(&scan_cfg)) {
            led_status_set_state(LED_STATUS_ERROR);
            goto cleanup;
        }

        if (!scan_builder_init(&scan_msg, &scan_cfg)) {
            ESP_LOGE(TAG_TASK, "scan_builder_init() failed (malloc?)");
            led_status_set_state(LED_STATUS_ERROR);
            goto cleanup;
        }
        scan_ready = true;

        scan_pub_task_enabled = false;
        scan_pub_task_stop = false;
        BaseType_t scan_task_ok = xTaskCreatePinnedToCore(
            scan_pub_task,
            "scan_pub_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            &scan_pub_task_handle,
            1
        );
        if (scan_task_ok != pdPASS || scan_pub_task_handle == NULL) {
            ESP_LOGE(TAG_TASK, "Failed to create scan publish task");
            led_status_set_state(LED_STATUS_ERROR);
            goto cleanup;
        }

        RCCHECK_FAIL_GOTO(rclc_timer_init_default2(
            &timer,
            &support,
            RCL_MS_TO_NS(TIMER_PERIOD_MS),
            timer_callback,
            true),
            cleanup,
            init_failed);
        timer_ready = true;

        RCCHECK_FAIL_GOTO(rclc_executor_init(&executor, &support.context, NUMBER_OF_HANDLES, &allocator),
                          cleanup, init_failed);
        executor_ready = true;
        RCCHECK_FAIL_GOTO(rclc_executor_add_timer(&executor, &timer), cleanup, init_failed);
        scan_pub_task_enabled = true;

        ESP_LOGI(TAG_TASK, "micro-ROS up: node=%s topic=%s period=%dms",
                 CONFIG_MICRO_ROS_NODE_NAME, CONFIG_MICRO_ROS_TOPIC_NAME, TIMER_PERIOD_MS);
        led_status_set_state(LED_STATUS_CONNECTED);

        uint32_t missed_pings = 0;
        while (true) {
            rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
            if (spin_ret != RCL_RET_OK) {
                ESP_LOGE(TAG_TASK, "rclc_executor_spin_some() failed");
                rcl_reset_error();
            }
            if (publish_error_burst) {
                if (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
                    missed_pings++;
                    agent_missed_pings++;
                    ESP_LOGW(TAG_TASK, "micro-ROS agent ping missed (%" PRIu32 "/3)", missed_pings);
                    if (missed_pings >= 3) {
                        ESP_LOGW(TAG_TASK, "micro-ROS agent lost, restarting session");
                        led_status_set_state(LED_STATUS_WAITING);
                        break;
                    }
                } else {
                    missed_pings = 0;
                    congestion_detected++;
                    publish_backoff_cycles = PUBLISH_BACKOFF_MAX_CYCLES;
                    ESP_LOGW(TAG_TASK,
                             "Publish failures with agent reachable (congestion=%" PRIu32
                             ", publish_failures=%" PRIu32 "). Slowing down without restart.",
                             congestion_detected, publish_failures);
                    publish_error_burst = false;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

cleanup:
        configure_entity_destroy_timeout();
        if (scan_pub_task_handle != NULL) {
            scan_pub_task_enabled = false;
            scan_pub_task_stop = true;
            xTaskNotifyGive(scan_pub_task_handle);
            for (int i = 0; i < 10 && scan_pub_task_handle != NULL; i++) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        if (executor_ready) {
            rc = rclc_executor_fini(&executor);
            if (rc != RCL_RET_OK) {
                log_rcl_failure(TAG_TASK, "rclc_executor_fini()", rc);
            }
        }
        if (timer_ready) {
            rc = rcl_timer_fini(&timer);
            if (rc != RCL_RET_OK) {
                log_rcl_failure(TAG_TASK, "rcl_timer_fini()", rc);
            }
        }
        if (publisher_ready) {
            rc = rcl_publisher_fini(&publisher, &node);
            if (rc != RCL_RET_OK) {
                log_rcl_failure(TAG_TASK, "rcl_publisher_fini()", rc);
            }
        }
        if (node_ready) {
            rc = rcl_node_fini(&node);
            if (rc != RCL_RET_OK) {
                log_rcl_failure(TAG_TASK, "rcl_node_fini()", rc);
            }
        }
        if (context_ready && rcl_context_is_valid(&support.context)) {
            rc = rcl_shutdown(&support.context);
            if (rc != RCL_RET_OK) {
                log_rcl_failure(TAG_TASK, "rcl_shutdown()", rc);
            }
        }
        if (context_ready) {
            rc = rclc_support_fini(&support);
            if (rc != RCL_RET_OK) {
                log_rcl_failure(TAG_TASK, "rclc_support_fini()", rc);
            }
        }
        if (init_options_ready) {
            rc = rcl_init_options_fini(&init_options);
            if (rc != RCL_RET_OK) {
                log_rcl_failure(TAG_TASK, "rcl_init_options_fini()", rc);
            }
        }
        if (scan_ready) {
            scan_builder_deinit(&scan_msg);
        }
        if (init_failed) {
            consecutive_failures++;
            ESP_LOGW(TAG_TASK, "Initialization failed (%" PRIu32 " consecutive)", consecutive_failures);
            if (consecutive_failures >= 3) {
                ESP_LOGE(TAG_TASK, "Too many init failures, restarting system");
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_restart();
            }
        } else {
            consecutive_failures = 0;
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
