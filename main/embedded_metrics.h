#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "rcl/rcl.h"
#include "rclc/executor.h"
#include "rclc/types.h"

#ifdef __cplusplus
extern "C" {
#endif

bool embedded_metrics_init(rcl_node_t *node,
                           rclc_executor_t *executor,
                           rclc_support_t *support,
                           SemaphoreHandle_t rcl_mutex);
void embedded_metrics_deinit(rcl_node_t *node);

void embedded_metrics_set_loop_dt_us(uint32_t loop_dt_us);
void embedded_metrics_set_xrce_reconnect_count(uint32_t count);
void embedded_metrics_set_pub_fail_count(uint32_t count);

#ifdef __cplusplus
}
#endif
