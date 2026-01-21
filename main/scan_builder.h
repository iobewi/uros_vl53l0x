#pragma once
#include <stdbool.h>

#include <sensor_msgs/msg/laser_scan.h>
#include "tof_provider.h"
#include "tof_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float angle_min;
    float angle_inc;
    int bins;

    float range_min;
    float range_max;

    float scan_time;
    float time_increment;

    const char *frame_id;
} scan_config_t;

/**
 * @brief Initialize a LaserScan message according to the provided scan configuration.
 *
 * This function initializes the LaserScan structure, sets all static fields
 * (frame_id, angle_min, angle_increment, angle_max, range limits, timing),
 * allocates the ranges buffer once, preallocates the (unused) intensities buffer,
 * and initializes all range values to NAN.
 *
 * Intensities are not used and are left empty.
 *
 * This function must be called once before repeated calls to scan_builder_fill().
 *
 * @param msg   Pointer to the LaserScan message to initialize.
 * @param cfg   Scan configuration (bin count, angular layout, range limits,
 *              scan timing, frame identifier).
 *
 * @return true on success, false on invalid parameters or allocation failure.
 */
bool scan_builder_init(sensor_msgs__msg__LaserScan *msg, const scan_config_t *cfg);

/**
 * @brief Deinitialize a LaserScan message previously initialized by scan_builder_init().
 *
 * This function releases the ranges buffer and resets ranges/intensities fields
 * to a safe empty state. Call it when the scan builder is no longer needed or
 * before reinitializing.
 *
 * @param msg   Pointer to the LaserScan message to deinitialize.
 */
void scan_builder_deinit(sensor_msgs__msg__LaserScan *msg);

/**
 * @brief Populate a LaserScan message from a snapshot of ToF samples,
 *        using a fixed bin index mapping.
 *
 * The scan is first cleared (all ranges set to NAN). Each ToF sample is then
 * written to the LaserScan bin specified by hw_cfg[sensor_index].bin_idx.
 *
 * - The mapping is index-based (no angle computation or rounding).
 * - Each ToF sensor is expected to map to a unique bin (no overlap).
 * - Bins not associated with any sensor remain NAN (unobserved space).
 * - Invalid or out-of-range samples are ignored, leaving the bin as NAN.
 *
 * @param msg       Pointer to the LaserScan message to fill.
 * @param cfg       Scan configuration (bin count, angle_min, angle_increment,
 *                  range_min, range_max, scan timing).
 * @param samples   Snapshot array of TOF_COUNT ToF measurements.
 * @param hw_cfg    Hardware configuration table (includes bin indices).
 */
void scan_builder_fill(sensor_msgs__msg__LaserScan *msg,
                       const scan_config_t *cfg,
                       const tof_sample_t samples[TOF_COUNT],
                       const tof_hw_config_t *hw_cfg);


#ifdef __cplusplus
}
#endif
