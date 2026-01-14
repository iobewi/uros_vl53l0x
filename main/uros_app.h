#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the micro-ROS node, publisher and timer (runs in its own FreeRTOS task).
 * @return true on success, false otherwise
 */
bool uros_app_start(void);

#ifdef __cplusplus
}
#endif
