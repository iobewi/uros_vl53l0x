#include "uros_app_scan.h"

#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "esp_log.h"
#include "sdkconfig.h"

#include <sensor_msgs/msg/laser_scan.h>
#include <rmw_microros/rmw_microros.h>

#include "uros_core.h"
#include "scan_builder.h"
#include "scan_engine.h"
#include "tof_config.h"
#include "led_status.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "UROS_APP_SCAN";

// Configuration from Kconfig
#define N_BINS CONFIG_MICRO_ROS_SCAN_BINS
#define ANGLE_MIN (-(float)M_PI)
#define ANGLE_INC ((2.0f * (float)M_PI) / (float)N_BINS)

/**
 * @brief Application context for LaserScan publisher
 */
typedef struct {
    // ROS message
    sensor_msgs__msg__LaserScan scan_msg;

    // Scan engine and builder
    scan_config_t scan_cfg;
    scan_engine_t scan_engine;

    // Storage buffers (statically allocated)
    float ranges_buffer[N_BINS];
    char frame_id_buffer[64];  // Large enough for CONFIG_MICRO_ROS_SCAN_FRAME_ID
    scan_builder_storage_t scan_storage;

    // Metrics
    uint32_t step_failures;
} uros_app_scan_context_t;

// Global context (single instance)
static uros_app_scan_context_t g_scan_context;

// ============================================================================
// Application callbacks
// ============================================================================

/**
 * @brief Initialize scan application resources
 */
static bool scan_app_init(void *app_context)
{
    uros_app_scan_context_t *ctx = (uros_app_scan_context_t *)app_context;

    ESP_LOGI(TAG, "Initializing scan application...");

    // Configure scan parameters
    ctx->scan_cfg.angle_min = ANGLE_MIN;
    ctx->scan_cfg.angle_inc = ANGLE_INC;
    ctx->scan_cfg.bins = N_BINS;
    ctx->scan_cfg.range_min = 0.03f;
    ctx->scan_cfg.range_max = 2.00f;
    ctx->scan_cfg.scan_time = (float)CONFIG_MICRO_ROS_TIMER_PERIOD_MS / 1000.0f;
    ctx->scan_cfg.time_increment = 0.0f;
    ctx->scan_cfg.frame_id = CONFIG_MICRO_ROS_SCAN_FRAME_ID;

    // Initialize scan engine
    if (!scan_engine_init(&ctx->scan_engine, &ctx->scan_cfg, tof_get_hw_config())) {
        ESP_LOGE(TAG, "scan_engine_init() failed");
        return false;
    }
    scan_engine_set_time_provider(&ctx->scan_engine, rmw_uros_epoch_nanos);

    // Configure storage
    ctx->scan_storage.ranges_buffer = ctx->ranges_buffer;
    ctx->scan_storage.ranges_capacity = N_BINS;
    ctx->scan_storage.frame_id_buffer = ctx->frame_id_buffer;
    ctx->scan_storage.frame_id_capacity = sizeof(ctx->frame_id_buffer);
    ctx->scan_storage.owns_ranges_buffer = false;
    ctx->scan_storage.owns_frame_id_buffer = false;

    // Initialize scan builder
    if (!scan_builder_init(&ctx->scan_msg, &ctx->scan_cfg, &ctx->scan_storage)) {
        ESP_LOGE(TAG, "scan_builder_init() failed");
        scan_engine_deinit(&ctx->scan_engine);
        return false;
    }

    ctx->step_failures = 0;

    ESP_LOGI(TAG, "Scan application initialized (bins=%d, period=%ums)",
             N_BINS, CONFIG_MICRO_ROS_TIMER_PERIOD_MS);
    return true;
}

/**
 * @brief Execute one scan step and fill the ROS message
 */
static bool scan_app_step(void *app_context, void *ros_message)
{
    uros_app_scan_context_t *ctx = (uros_app_scan_context_t *)app_context;
    sensor_msgs__msg__LaserScan *msg = (sensor_msgs__msg__LaserScan *)ros_message;

    // Execute scan engine step
    bool ok = scan_engine_step(&ctx->scan_engine, msg);
    if (!ok) {
        ctx->step_failures++;
#if CONFIG_MICRO_ROS_SCAN_STEP_ERROR_LOG_DIVIDER > 0
        if ((ctx->step_failures % CONFIG_MICRO_ROS_SCAN_STEP_ERROR_LOG_DIVIDER) == 0) {
            ESP_LOGE(TAG, "scan_engine_step() failed (total failures: %" PRIu32 ")",
                     ctx->step_failures);
        }
#endif
    }

    return ok;
}

/**
 * @brief Cleanup scan application resources
 */
static void scan_app_fini(void *app_context)
{
    uros_app_scan_context_t *ctx = (uros_app_scan_context_t *)app_context;

    ESP_LOGI(TAG, "Cleaning up scan application...");

    scan_builder_deinit(&ctx->scan_msg, &ctx->scan_storage);
    scan_engine_deinit(&ctx->scan_engine);

    ESP_LOGI(TAG, "Scan application cleaned up");
}

// ============================================================================
// LED status callback wrapper
// ============================================================================

static void scan_led_status_callback(uros_led_state_t state)
{
    // Map core LED states to led_status.h states
    switch (state) {
        case UROS_LED_WAITING:
            led_status_set_state(LED_STATUS_WAITING);
            break;
        case UROS_LED_CONNECTED:
            led_status_set_state(LED_STATUS_CONNECTED);
            break;
        case UROS_LED_ERROR:
            led_status_set_state(LED_STATUS_ERROR);
            break;
    }
}

// ============================================================================
// Public API
// ============================================================================

bool uros_app_scan_start(void)
{
    ESP_LOGI(TAG, "Starting LaserScan publisher application...");

    // Initialize LED status
#if !CONFIG_MICRO_ROS_STATUS_LED_ENABLE
    ESP_LOGW(TAG, "Status LED disabled (CONFIG_MICRO_ROS_STATUS_LED_ENABLE=n)");
#endif
    if (!led_status_start()) {
        ESP_LOGE(TAG, "Failed to start status LED task");
        return false;
    }
    led_status_set_state(LED_STATUS_WAITING);

    // Configure core
    uros_core_config_t config = {
        // ROS configuration
        .node_name = CONFIG_MICRO_ROS_NODE_NAME,
        .topic_name = CONFIG_MICRO_ROS_TOPIC_NAME,
        .domain_id = CONFIG_MICRO_ROS_DOMAIN_ID,
        .timer_period_ms = CONFIG_MICRO_ROS_TIMER_PERIOD_MS,

        // QoS configuration
        .qos_history = RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        .qos_depth = CONFIG_MICRO_ROS_QOS_DEPTH,
#if defined(CONFIG_MICRO_ROS_QOS_RELIABLE)
        .qos_reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE,
#else
        .qos_reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
#endif
        .qos_durability = RMW_QOS_POLICY_DURABILITY_VOLATILE,

        // Task configuration
        .stack_size = CONFIG_MICRO_ROS_APP_STACK,
        .task_priority = CONFIG_MICRO_ROS_APP_TASK_PRIO,
        .core_affinity = 1,  // Pin to core 1

        // Observability hooks
        .on_state_change = scan_led_status_callback,
        .on_metrics = NULL,  // No metrics callback for now
    };

    // Configure application interface
    uros_app_interface_t app = {
        .type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        .message_size = sizeof(sensor_msgs__msg__LaserScan),
        .app_init = scan_app_init,
        .app_step = scan_app_step,
        .app_fini = scan_app_fini,
        .app_context = &g_scan_context,
        .app_context_size = sizeof(uros_app_scan_context_t),
    };

    // Initialize context
    memset(&g_scan_context, 0, sizeof(uros_app_scan_context_t));

    // Create core context
    uros_core_context_t *core_ctx = uros_core_create(&config, &app);
    if (core_ctx == NULL) {
        ESP_LOGE(TAG, "Failed to create micro-ROS core context");
        return false;
    }

    // Start core
    if (!uros_core_start(core_ctx)) {
        ESP_LOGE(TAG, "Failed to start micro-ROS core");
        uros_core_destroy(core_ctx);
        return false;
    }

    ESP_LOGI(TAG, "LaserScan publisher started successfully");
    return true;
}
