#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the micro-ROS LaserScan publisher application
 *
 * This function configures and starts a micro-ROS publisher for LaserScan messages.
 * Configuration is read from Kconfig settings (CONFIG_MICRO_ROS_*).
 *
 * @return true on success, false on failure
 */
bool uros_app_scan_start(void);

#ifdef __cplusplus
}
#endif
