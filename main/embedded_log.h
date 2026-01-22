#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "rcl/rcl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EMBEDDED_LOG_LEVEL_DEBUG = 0,
    EMBEDDED_LOG_LEVEL_INFO = 1,
    EMBEDDED_LOG_LEVEL_WARN = 2,
    EMBEDDED_LOG_LEVEL_ERROR = 3,
    EMBEDDED_LOG_LEVEL_FATAL = 4
} embedded_log_level_t;

bool embedded_log_init(rcl_node_t *node, SemaphoreHandle_t rcl_mutex);
void embedded_log_deinit(rcl_node_t *node);

void embedded_log_write(embedded_log_level_t level, const char *tag, const char *fmt, ...);

#ifdef __cplusplus
}
#endif
