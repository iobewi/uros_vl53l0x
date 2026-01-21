#pragma once

#include <stdint.h>

#include "driver/gpio.h"

#include "tof_provider.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t sda_gpio;
    gpio_num_t scl_gpio;
    uint32_t i2c_freq_hz;
    uint32_t timing_budget_us;
    uint32_t gpio_ready_timeout_ms;
} tof_bus_config_t;

typedef struct {
    gpio_num_t xshut_gpio;
    gpio_num_t int_gpio;
    uint8_t addr_7b;
    uint8_t bin_idx;
} tof_hw_config_t;

const tof_bus_config_t *tof_get_bus_config(void);
const tof_hw_config_t *tof_get_hw_config(void);

#ifdef __cplusplus
}
#endif
