#include "tof_config.h"

#include "sdkconfig.h"

static const tof_bus_config_t TOF_BUS_CONFIG = {
    .sda_gpio = CONFIG_TOF_I2C_SDA_GPIO,
    .scl_gpio = CONFIG_TOF_I2C_SCL_GPIO,
    .i2c_freq_hz = CONFIG_TOF_I2C_FREQ_HZ,
    .timing_budget_us = CONFIG_TOF_TIMING_BUDGET_US,
    .gpio_ready_timeout_ms = CONFIG_TOF_GPIO_READY_TIMEOUT_MS,
};

static const tof_hw_config_t TOF_HW_CONFIG[TOF_COUNT] = {
    {
        .xshut_gpio = CONFIG_TOF_0_XSHUT_GPIO,
        .int_gpio = CONFIG_TOF_0_INT_GPIO,
        .addr_7b = CONFIG_TOF_0_ADDR_7B,
        .bin_idx = CONFIG_TOF_0_BIN_IDX,
    },
#if CONFIG_TOF_COUNT > 1
    {
        .xshut_gpio = CONFIG_TOF_1_XSHUT_GPIO,
        .int_gpio = CONFIG_TOF_1_INT_GPIO,
        .addr_7b = CONFIG_TOF_1_ADDR_7B,
        .bin_idx = CONFIG_TOF_1_BIN_IDX,
    },
#endif
#if CONFIG_TOF_COUNT > 2
    {
        .xshut_gpio = CONFIG_TOF_2_XSHUT_GPIO,
        .int_gpio = CONFIG_TOF_2_INT_GPIO,
        .addr_7b = CONFIG_TOF_2_ADDR_7B,
        .bin_idx = CONFIG_TOF_2_BIN_IDX,
    },
#endif
#if CONFIG_TOF_COUNT > 3
    {
        .xshut_gpio = CONFIG_TOF_3_XSHUT_GPIO,
        .int_gpio = CONFIG_TOF_3_INT_GPIO,
        .addr_7b = CONFIG_TOF_3_ADDR_7B,
        .bin_idx = CONFIG_TOF_3_BIN_IDX,
    },
#endif
#if CONFIG_TOF_COUNT > 4
    {
        .xshut_gpio = CONFIG_TOF_4_XSHUT_GPIO,
        .int_gpio = CONFIG_TOF_4_INT_GPIO,
        .addr_7b = CONFIG_TOF_4_ADDR_7B,
        .bin_idx = CONFIG_TOF_4_BIN_IDX,
    },
#endif
#if CONFIG_TOF_COUNT > 5
    {
        .xshut_gpio = CONFIG_TOF_5_XSHUT_GPIO,
        .int_gpio = CONFIG_TOF_5_INT_GPIO,
        .addr_7b = CONFIG_TOF_5_ADDR_7B,
        .bin_idx = CONFIG_TOF_5_BIN_IDX,
    },
#endif
#if CONFIG_TOF_COUNT > 6
    {
        .xshut_gpio = CONFIG_TOF_6_XSHUT_GPIO,
        .int_gpio = CONFIG_TOF_6_INT_GPIO,
        .addr_7b = CONFIG_TOF_6_ADDR_7B,
        .bin_idx = CONFIG_TOF_6_BIN_IDX,
    },
#endif
#if CONFIG_TOF_COUNT > 7
    {
        .xshut_gpio = CONFIG_TOF_7_XSHUT_GPIO,
        .int_gpio = CONFIG_TOF_7_INT_GPIO,
        .addr_7b = CONFIG_TOF_7_ADDR_7B,
        .bin_idx = CONFIG_TOF_7_BIN_IDX,
    },
#endif
};

const tof_bus_config_t *tof_get_bus_config(void)
{
    return &TOF_BUS_CONFIG;
}

const tof_hw_config_t *tof_get_hw_config(void)
{
    return TOF_HW_CONFIG;
}
