#include "uros_core.h"

#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

static const char *TAG = "UROS_CORE";

// Time sync configuration
#define TIME_SYNC_TIMEOUT_MS 1000U
#define TIME_SYNC_MAX_ATTEMPTS 5
#define ENTITY_DESTROY_TIMEOUT_MS 200U

// Publish backoff configuration
#define PUBLISH_BACKOFF_MAX_CYCLES 5U

// Agent supervision configuration
#define AGENT_PING_INTERVAL_MS 1000U
#define AGENT_MAX_MISSED_PINGS 3

// RCL check macros
#define RCCHECK_GOTO(fn, label) {                                                                     \
    rcl_ret_t temp_rc = fn;                                                                           \
    if (temp_rc != RCL_RET_OK) {                                                                      \
        ESP_LOGE("RCCHECK", "Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc);      \
        if (ctx->config.on_state_change != NULL) {                                                    \
            ctx->config.on_state_change(UROS_LED_ERROR);                                            \
        }                                                                                             \
        goto label;                                                                                   \
    }                                                                                                 \
}

#define RCCHECK_FAIL_GOTO(fn, label, flag) {                                                          \
    rcl_ret_t temp_rc = fn;                                                                           \
    if (temp_rc != RCL_RET_OK) {                                                                      \
        ESP_LOGE("RCCHECK", "Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc);      \
        if (ctx->config.on_state_change != NULL) {                                                    \
            ctx->config.on_state_change(UROS_LED_ERROR);                                            \
        }                                                                                             \
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

// Weak symbol for entity destroy timeout (may not be available in all micro-ROS versions)
__attribute__((weak)) rmw_ret_t rmw_uros_set_entity_destroy_session_timeout(int64_t timeout_ms);

// Global context for timer callback (single instance supported)
static uros_core_context_t *g_current_ctx = NULL;

/**
 * @brief Internal context structure for micro-ROS core
 */
struct uros_core_context {
    // Configuration
    uros_core_config_t config;
    uros_app_interface_t app;

    // ROS message (allocated by core)
    void *ros_message;

    // RCL entities (shared between tasks)
    rcl_publisher_t *publisher;  // Pointer to publisher (owned by main_task)

    // FreeRTOS tasks
    TaskHandle_t main_task_handle;
    TaskHandle_t pub_task_handle;

    // Synchronization
    SemaphoreHandle_t rcl_mutex;

    // State flags
    volatile bool pub_task_enabled;
    volatile bool pub_task_stop;

    // Metrics
    volatile uint32_t publish_failures;
    volatile uint32_t app_step_failures;
    volatile uint32_t agent_missed_pings;
    volatile uint32_t congestion_detected;
    volatile bool publish_error_burst;
    uint32_t publish_backoff_cycles;
};

// ============================================================================
// Public utility functions
// ============================================================================

void uros_core_sync_time(void)
{
    rmw_ret_t sync_ret = RMW_RET_ERROR;
    for (int attempt = 1; attempt <= TIME_SYNC_MAX_ATTEMPTS; attempt++) {
        sync_ret = rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
        if (sync_ret == RMW_RET_OK) {
            ESP_LOGI(TAG, "Time sync completed");
            return;
        }
        ESP_LOGW(TAG, "Time sync attempt %d/%d failed: %d",
                 attempt, TIME_SYNC_MAX_ATTEMPTS, (int)sync_ret);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGW(TAG, "Time sync failed; timestamps may be unset");
}

void uros_core_log_rcl_failure(const char *tag, const char *label, rcl_ret_t rc)
{
    rcl_error_string_t err = rcl_get_error_string();
    if (err.str[0] != '\0') {
        ESP_LOGW(tag, "%s failed: %d (%s)", label, (int)rc, err.str);
    } else {
        ESP_LOGW(tag, "%s failed: %d", label, (int)rc);
    }
    rcl_reset_error();
}

void uros_core_configure_entity_timeout(void)
{
    if (rmw_uros_set_entity_destroy_session_timeout != NULL) {
        rmw_ret_t ret = rmw_uros_set_entity_destroy_session_timeout(ENTITY_DESTROY_TIMEOUT_MS);
        if (ret != RMW_RET_OK) {
            ESP_LOGW(TAG,
                     "rmw_uros_set_entity_destroy_session_timeout() failed: %d",
                     (int)ret);
        }
    }
}

// ============================================================================
// Private helper functions
// ============================================================================

/**
 * @brief Timer callback - notifies pub task to publish
 */
static void uros_core_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    // Use global context (single instance supported)
    if (g_current_ctx != NULL && g_current_ctx->pub_task_handle != NULL) {
        xTaskNotifyGive(g_current_ctx->pub_task_handle);
    }
}

/**
 * @brief Publisher task - executes app_step() and publishes messages
 */
static void uros_core_pub_task(void *arg)
{
    uros_core_context_t *ctx = (uros_core_context_t *)arg;

    ESP_LOGI(TAG, "Publisher task started");

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ctx->pub_task_stop) {
            break;
        }
        if (!ctx->pub_task_enabled) {
            continue;
        }

        // Apply backoff if needed
        if (ctx->publish_backoff_cycles > 0) {
            ctx->publish_backoff_cycles--;
            continue;
        }

        // Execute application step
        bool step_ok = ctx->app.app_step(ctx->app.app_context, ctx->ros_message);
        if (!step_ok) {
            ctx->app_step_failures++;
        }

        // Publish message
        rcl_ret_t pub_rc = RCL_RET_OK;
        if (step_ok && ctx->publisher != NULL) {
            if (ctx->rcl_mutex != NULL) {
                xSemaphoreTake(ctx->rcl_mutex, portMAX_DELAY);
            }
            pub_rc = rcl_publish(ctx->publisher, ctx->ros_message, NULL);
            if (ctx->rcl_mutex != NULL) {
                xSemaphoreGive(ctx->rcl_mutex);
            }
        }

        // Handle publish errors
        if (!step_ok || pub_rc != RCL_RET_OK) {
            ctx->publish_failures++;
            if (ctx->publish_backoff_cycles < PUBLISH_BACKOFF_MAX_CYCLES) {
                ctx->publish_backoff_cycles++;
            }
            if (ctx->publish_failures >= 50) {
                ctx->publish_error_burst = true;
            }
        } else if (ctx->publish_failures > 0) {
            ctx->publish_failures = 0;
            ctx->publish_error_burst = false;
            ctx->publish_backoff_cycles = 0;
        }
    }

    ESP_LOGI(TAG, "Publisher task stopped");
    ctx->pub_task_handle = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief Cleanup RCL session
 */
static void uros_core_cleanup_session(uros_core_context_t *ctx,
                                      rcl_publisher_t *publisher,
                                      rcl_node_t *node,
                                      rclc_executor_t *executor,
                                      rcl_timer_t *timer,
                                      rclc_support_t *support,
                                      rcl_init_options_t *init_options,
                                      bool executor_ready,
                                      bool timer_ready,
                                      bool publisher_ready,
                                      bool node_ready,
                                      bool context_ready,
                                      bool init_options_ready,
                                      bool app_ready)
{
    rcl_ret_t rc = RCL_RET_OK;

    uros_core_configure_entity_timeout();

    // Clear publisher pointer to prevent pub_task from using stale reference
    ctx->publisher = NULL;

    // Stop pub task
    if (ctx->pub_task_handle != NULL) {
        ctx->pub_task_enabled = false;
        ctx->pub_task_stop = true;
        xTaskNotifyGive(ctx->pub_task_handle);
        for (int i = 0; i < 10 && ctx->pub_task_handle != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (ctx->pub_task_handle != NULL) {
            TaskHandle_t handle = ctx->pub_task_handle;
            ESP_LOGE(TAG, "pub_task did not stop, forcing delete");
            vTaskDelete(handle);
            ctx->pub_task_handle = NULL;
        }
    }

    // Delete mutex
    if (ctx->rcl_mutex != NULL) {
        vSemaphoreDelete(ctx->rcl_mutex);
        ctx->rcl_mutex = NULL;
    }

    // Cleanup RCL entities
    if (executor_ready) {
        rc = rclc_executor_fini(executor);
        if (rc != RCL_RET_OK) {
            uros_core_log_rcl_failure(TAG, "rclc_executor_fini()", rc);
        }
    }
    if (timer_ready) {
        rc = rcl_timer_fini(timer);
        if (rc != RCL_RET_OK) {
            uros_core_log_rcl_failure(TAG, "rcl_timer_fini()", rc);
        }
    }
    if (publisher_ready) {
        rc = rcl_publisher_fini(publisher, node);
        if (rc != RCL_RET_OK) {
            uros_core_log_rcl_failure(TAG, "rcl_publisher_fini()", rc);
        }
    }
    if (node_ready) {
        rc = rcl_node_fini(node);
        if (rc != RCL_RET_OK) {
            uros_core_log_rcl_failure(TAG, "rcl_node_fini()", rc);
        }
    }
    if (context_ready && rcl_context_is_valid(&support->context)) {
        rc = rcl_shutdown(&support->context);
        if (rc != RCL_RET_OK) {
            uros_core_log_rcl_failure(TAG, "rcl_shutdown()", rc);
        }
    }
    if (context_ready) {
        rc = rclc_support_fini(support);
        if (rc != RCL_RET_OK) {
            uros_core_log_rcl_failure(TAG, "rclc_support_fini()", rc);
        }
    }
    if (init_options_ready) {
        rc = rcl_init_options_fini(init_options);
        if (rc != RCL_RET_OK) {
            uros_core_log_rcl_failure(TAG, "rcl_init_options_fini()", rc);
        }
    }

    // Cleanup app
    if (app_ready && ctx->app.app_fini != NULL) {
        ctx->app.app_fini(ctx->app.app_context);
    }
}

/**
 * @brief Main micro-ROS task - manages lifecycle and executor
 */
static void uros_core_main_task(void *arg)
{
    uros_core_context_t *ctx = (uros_core_context_t *)arg;
    uint32_t consecutive_failures = 0;

    ESP_LOGI(TAG, "Main task started");

    // Set global context for timer callback access
    g_current_ctx = ctx;

    while (true) {
        rcl_ret_t rc = RCL_RET_OK;
        bool app_ready = false;
        bool node_ready = false;
        bool publisher_ready = false;
        bool executor_ready = false;
        bool timer_ready = false;
        bool init_options_ready = false;
        bool context_ready = false;
        bool init_failed = false;

        rcl_allocator_t allocator = rcl_get_default_allocator();
        rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
        rclc_support_t support = {0};
        rcl_node_t node = rcl_get_zero_initialized_node();
        rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
        rcl_timer_t timer = rcl_get_zero_initialized_timer();
        rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

        ESP_LOGI(TAG, "Waiting for micro-ROS agent...");
        if (ctx->config.on_state_change != NULL) {
            ctx->config.on_state_change(UROS_LED_WAITING);
        }
        while (rmw_uros_ping_agent(100, 5) != RMW_RET_OK) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        ESP_LOGI(TAG, "Agent detected, starting RCL init");

        // Initialize RCL
        RCCHECK_FAIL_GOTO(rcl_init_options_init(&init_options, allocator), cleanup, init_failed);
        init_options_ready = true;
        RCCHECK_FAIL_GOTO(rcl_init_options_set_domain_id(&init_options, ctx->config.domain_id), cleanup, init_failed);
        ESP_LOGI(TAG, "Initializing micro-ROS support...");
        RCCHECK_FAIL_GOTO(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator), cleanup, init_failed);
        context_ready = true;
        uros_core_sync_time();

        // Create node
        rcl_node_options_t node_ops = rcl_node_get_default_options();
        ESP_LOGI(TAG, "Creating node '%s'...", ctx->config.node_name);
        RCCHECK_FAIL_GOTO(rclc_node_init_with_options(&node, ctx->config.node_name, "", &support, &node_ops),
                          cleanup, init_failed);
        node_ready = true;

        // Create publisher with configured QoS
        ESP_LOGI(TAG, "Creating publisher '%s'...", ctx->config.topic_name);
        rmw_qos_profile_t qos = rmw_qos_profile_default;
        qos.history = ctx->config.qos_history;
        qos.depth = ctx->config.qos_depth;
        qos.reliability = ctx->config.qos_reliability;
        qos.durability = ctx->config.qos_durability;

        const int max_pub_attempts = 5;
        for (int attempt = 1; attempt <= max_pub_attempts; attempt++) {
            rc = rclc_publisher_init(
                &publisher,
                &node,
                ctx->app.type_support,
                ctx->config.topic_name,
                &qos);
            if (rc == RCL_RET_OK) {
                publisher_ready = true;
                break;
            }
            ESP_LOGW(TAG, "Publisher init attempt %d/%d failed: %d",
                     attempt, max_pub_attempts, (int)rc);
            if (attempt < max_pub_attempts) {
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }
        if (!publisher_ready) {
            ESP_LOGE("RCCHECK", "Failed status on line %d: %d. Aborting.", __LINE__, (int)rc);
            if (ctx->config.on_state_change != NULL) {
                ctx->config.on_state_change(UROS_LED_ERROR);
            }
            init_failed = true;
            goto cleanup;
        }

        // Initialize application
        if (ctx->app.app_init != NULL) {
            if (!ctx->app.app_init(ctx->app.app_context)) {
                ESP_LOGE(TAG, "app_init() failed");
                if (ctx->config.on_state_change != NULL) {
                    ctx->config.on_state_change(UROS_LED_ERROR);
                }
                goto cleanup;
            }
        }
        app_ready = true;

        // Make publisher accessible to pub_task
        ctx->publisher = &publisher;

        // Create publisher task
        ctx->pub_task_enabled = false;
        ctx->pub_task_stop = false;
        ctx->rcl_mutex = xSemaphoreCreateMutex();
        if (ctx->rcl_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create RCL mutex");
            if (ctx->config.on_state_change != NULL) {
                ctx->config.on_state_change(UROS_LED_ERROR);
            }
            goto cleanup;
        }

        BaseType_t pub_task_ok;
        if (ctx->config.core_affinity >= 0) {
            pub_task_ok = xTaskCreatePinnedToCore(
                uros_core_pub_task,
                "uros_pub_task",
                ctx->config.stack_size,
                ctx,  // Pass context as argument
                ctx->config.task_priority,
                &ctx->pub_task_handle,
                ctx->config.core_affinity
            );
        } else {
            pub_task_ok = xTaskCreate(
                uros_core_pub_task,
                "uros_pub_task",
                ctx->config.stack_size,
                ctx,
                ctx->config.task_priority,
                &ctx->pub_task_handle
            );
        }
        if (pub_task_ok != pdPASS || ctx->pub_task_handle == NULL) {
            ESP_LOGE(TAG, "Failed to create publisher task");
            if (ctx->config.on_state_change != NULL) {
                ctx->config.on_state_change(UROS_LED_ERROR);
            }
            goto cleanup;
        }

        // We need to make publisher accessible to pub_task
        // Store it in a global (hack) or extend context (better)
        // For now, let's extend the timer callback to pass publisher
        // Actually, let's use a different approach: store publisher in context
        // But context is const... Let's use a workaround with timer.impl

        // Initialize timer and executor
        RCCHECK_FAIL_GOTO(rclc_timer_init_default2(
            &timer,
            &support,
            RCL_MS_TO_NS(ctx->config.timer_period_ms),
            uros_core_timer_callback,
            true),
            cleanup,
            init_failed);
        timer_ready = true;

        RCCHECK_FAIL_GOTO(rclc_executor_init(&executor, &support.context, 1, &allocator),
                          cleanup, init_failed);
        executor_ready = true;
        RCCHECK_FAIL_GOTO(rclc_executor_add_timer(&executor, &timer), cleanup, init_failed);
        ctx->pub_task_enabled = true;

        ESP_LOGI(TAG, "micro-ROS up: node=%s topic=%s period=%ums",
                 ctx->config.node_name, ctx->config.topic_name, ctx->config.timer_period_ms);
        if (ctx->config.on_state_change != NULL) {
            ctx->config.on_state_change(UROS_LED_CONNECTED);
        }

        // Main spin loop with agent supervision
        const TickType_t ping_interval = pdMS_TO_TICKS(AGENT_PING_INTERVAL_MS);
        uint32_t missed_pings = 0;
        uint32_t spin_failures = 0;
        TickType_t last_ping_tick = xTaskGetTickCount();

        while (true) {
            if (ctx->rcl_mutex != NULL) {
                xSemaphoreTake(ctx->rcl_mutex, portMAX_DELAY);
            }
            rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
            if (ctx->rcl_mutex != NULL) {
                xSemaphoreGive(ctx->rcl_mutex);
            }
            if (spin_ret != RCL_RET_OK) {
                spin_failures++;
                rcl_reset_error();
            }

            TickType_t now_tick = xTaskGetTickCount();
            if ((now_tick - last_ping_tick) >= ping_interval) {
                last_ping_tick = now_tick;
                if (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
                    missed_pings++;
                    ctx->agent_missed_pings++;
                    ESP_LOGW(TAG,
                             "micro-ROS agent ping missed (%" PRIu32 "/%" PRIu32 ")",
                             missed_pings, (uint32_t)AGENT_MAX_MISSED_PINGS);
                    if (missed_pings >= AGENT_MAX_MISSED_PINGS) {
                        ESP_LOGW(TAG, "micro-ROS agent lost, restarting session");
                        if (ctx->config.on_state_change != NULL) {
                            ctx->config.on_state_change(UROS_LED_WAITING);
                        }
                        break;
                    }
                } else {
                    if (missed_pings != 0) {
                        missed_pings = 0;
                    }
                    if (ctx->publish_error_burst) {
                        ctx->congestion_detected++;
                        ctx->publish_backoff_cycles = PUBLISH_BACKOFF_MAX_CYCLES;
                        ESP_LOGW(TAG,
                                 "Publish failures with agent reachable (congestion=%" PRIu32
                                 ", publish_failures=%" PRIu32 "). Slowing down without restart.",
                                 ctx->congestion_detected, ctx->publish_failures);
                        ctx->publish_error_burst = false;
                    }
                }
            }

            // Report metrics periodically if callback provided
            if (ctx->config.on_metrics != NULL) {
                // TODO: Add metrics reporting interval
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }

cleanup:
        uros_core_cleanup_session(ctx, &publisher, &node, &executor, &timer, &support, &init_options,
                                  executor_ready, timer_ready, publisher_ready, node_ready,
                                  context_ready, init_options_ready, app_ready);

        if (init_failed) {
            consecutive_failures++;
            ESP_LOGW(TAG, "Initialization failed (%" PRIu32 " consecutive)", consecutive_failures);
            if (consecutive_failures >= 3) {
                ESP_LOGE(TAG, "Too many init failures, restarting system");
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_restart();
            }
        } else {
            consecutive_failures = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ============================================================================
// Core lifecycle functions
// ============================================================================

uros_core_context_t *uros_core_create(const uros_core_config_t *config,
                                      const uros_app_interface_t *app)
{
    if (config == NULL || app == NULL) {
        ESP_LOGE(TAG, "Invalid arguments to uros_core_create()");
        return NULL;
    }

    // Validate app interface
    if (app->type_support == NULL || app->message_size == 0 || app->app_step == NULL) {
        ESP_LOGE(TAG, "Invalid app interface (missing type_support, message_size, or app_step)");
        return NULL;
    }

    // Allocate context
    uros_core_context_t *ctx = (uros_core_context_t *)malloc(sizeof(uros_core_context_t));
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Failed to allocate core context");
        return NULL;
    }
    memset(ctx, 0, sizeof(uros_core_context_t));

    // Copy configuration
    memcpy(&ctx->config, config, sizeof(uros_core_config_t));
    memcpy(&ctx->app, app, sizeof(uros_app_interface_t));

    // Allocate ROS message
    ctx->ros_message = malloc(app->message_size);
    if (ctx->ros_message == NULL) {
        ESP_LOGE(TAG, "Failed to allocate ROS message (%zu bytes)", app->message_size);
        free(ctx);
        return NULL;
    }
    memset(ctx->ros_message, 0, app->message_size);

    ESP_LOGI(TAG, "Core context created (msg_size=%zu)", app->message_size);
    return ctx;
}

void uros_core_destroy(uros_core_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    // Free ROS message
    if (ctx->ros_message != NULL) {
        free(ctx->ros_message);
    }

    // Free context
    free(ctx);
    ESP_LOGI(TAG, "Core context destroyed");
}

bool uros_core_start(uros_core_context_t *ctx)
{
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Invalid context");
        return false;
    }

    ESP_LOGI(TAG, "Starting micro-ROS core...");

    BaseType_t ok;
    if (ctx->config.core_affinity >= 0) {
        ok = xTaskCreatePinnedToCore(
            uros_core_main_task,
            "uros_main_task",
            ctx->config.stack_size,
            ctx,
            ctx->config.task_priority,
            &ctx->main_task_handle,
            ctx->config.core_affinity
        );
    } else {
        ok = xTaskCreate(
            uros_core_main_task,
            "uros_main_task",
            ctx->config.stack_size,
            ctx,
            ctx->config.task_priority,
            &ctx->main_task_handle
        );
    }

    if (ok != pdPASS || ctx->main_task_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create main task");
        return false;
    }

    return true;
}
