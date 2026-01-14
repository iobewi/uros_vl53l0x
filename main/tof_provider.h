#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TOF_COUNT 8

typedef struct {
    bool valid;
    uint8_t status;   // 0 = OK (ST style), otherwise invalid
    float range_m;    // meters, NAN if invalid
    uint32_t seq;
} tof_sample_t;

/**
 * @brief Initialize the ToF provider (mock for now).
 *        Must be called once at startup (before snapshot is used).
 */
void tof_provider_init(void);

/**
 * @brief Take an atomic snapshot of the latest samples.
 */
void tof_provider_snapshot(tof_sample_t out[TOF_COUNT]);

#ifdef __cplusplus
}
#endif
