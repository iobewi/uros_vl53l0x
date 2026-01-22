# micro-ROS + ToF (mock/VL53) — architecture, configuration, execution flow

[Version française](micro_ros_tof_architecture.fr.md)

This document provides the startup pipeline, ToF provider selection (mock/VL53),
critical Kconfig parameters, ROS QoS/timing, and common troubleshooting guidance.

## 1) Overview / Architecture

### Startup pipeline (main flow)

```
app_main
 ├─ init transport USB-CDC (micro-ROS custom transport)
 ├─ tof_provider_init()
 ├─ micro_ros_adapter_start()
 │   └─ micro_ros_task
 │       ├─ wait for micro-ROS agent
 │       ├─ init RCL, node, publisher, timer, executor
 │       ├─ scan_engine_init + scan_builder_init
 │       ├─ loop: timer → scan_pub_task → scan_engine_step → rcl_publish
 │       └─ agent monitoring + restart
 └─ FreeRTOS loop (tasks)
```

**Key locations**
- Global startup and USB-CDC transport in `app_main()`.【F:main/main.c†L1-L61】
- The micro-ROS app is started by `uros_app_start()` → `micro_ros_adapter_start()`.【F:main/uros_app.c†L1-L9】【F:main/micro_ros_adapter.c†L570-L605】
- The scan pipeline is `scan_engine_step()` → `tof_provider_snapshot()` → `scan_builder_fill()`.
  【F:main/scan_engine.c†L120-L129】【F:main/scan_builder.c†L156-L186】【F:main/tof_provider.h†L24-L29】

### Module responsibilities

- **tof_provider_(mock|vl53).c**
  - Provides ToF samples (range + status + valid) via `tof_provider_snapshot()`.
  - The mock provider generates synthetic signals (sine, “cliff”, invalid bursts).
    【F:main/tof_provider_mock.c†L108-L172】
  - The VL53L0X provider initializes I2C, assigns addresses via XSHUT, and handles data-ready IRQs.
    【F:main/tof_provider_vl53.c†L220-L321】

- **scan_engine.c**
  - Scan orchestration: ToF snapshot → LaserScan build → timestamp.
  - Validates sensor→bin mapping and applies the time source.
    【F:main/scan_engine.c†L29-L108】【F:main/scan_engine.c†L120-L139】

- **scan_builder.c**
  - Builds `sensor_msgs/LaserScan` (ranges, frame_id, angles, limits).
  - Ignores invalid samples and only fills valid bins.
    【F:main/scan_builder.c†L43-L123】【F:main/scan_builder.c†L156-L186】

- **micro_ros_adapter.c**
  - Starts the micro-ROS session (node, publisher, timer, executor) and publishes LaserScan.
  - Handles agent detection, QoS, time sync, and error backoff.
    【F:main/micro_ros_adapter.c†L286-L520】

- **led_status.c**
  - Status indicator (waiting, connected, error) via configurable LED.
    【F:main/led_status.c†L43-L133】

## 2) Critical Kconfig configuration

### micro-ROS (publishing & scheduling)
- `CONFIG_MICRO_ROS_DOMAIN_ID`: micro-ROS domain for discovery.【F:main/Kconfig.projbuild†L15-L19】
- `CONFIG_MICRO_ROS_NODE_NAME`: node name for LaserScan publishing.【F:main/Kconfig.projbuild†L21-L25】
- `CONFIG_MICRO_ROS_TOPIC_NAME`: LaserScan topic name.【F:main/Kconfig.projbuild†L27-L31】
- `CONFIG_MICRO_ROS_TIMER_PERIOD_MS`: publish period (ms).【F:main/Kconfig.projbuild†L33-L37】
- `CONFIG_MICRO_ROS_QOS_RELIABLE`: toggle RELIABLE vs BEST_EFFORT QoS.【F:main/Kconfig.projbuild†L39-L45】
- `CONFIG_MICRO_ROS_QOS_DEPTH`: QoS depth (KEEP_LAST) for LaserScan.【F:main/Kconfig.projbuild†L47-L52】
- `CONFIG_MICRO_ROS_SCAN_BINS`: LaserScan bin count (angular resolution).【F:main/Kconfig.projbuild†L54-L58】
- `CONFIG_MICRO_ROS_SCAN_FRAME_ID`: LaserScan frame_id.【F:main/Kconfig.projbuild†L60-L64】
- Logs/diagnostics:
  - `CONFIG_MICRO_ROS_PUBLISH_LOG_DIVIDER`, `CONFIG_MICRO_ROS_PUBLISH_ERROR_LOG_DIVIDER`,
    `CONFIG_MICRO_ROS_SCAN_STEP_ERROR_LOG_DIVIDER`, `CONFIG_MICRO_ROS_EXECUTOR_ERROR_LOG_DIVIDER`.
    【F:main/Kconfig.projbuild†L72-L98】
- Scan memory behavior:
  - `CONFIG_MICRO_ROS_SCAN_ALLOC_GUARD`, `CONFIG_MICRO_ROS_SCAN_BUILDER_ALLOC_MALLOC`.
    【F:main/Kconfig.projbuild†L100-L113】

### ToF (I2C, GPIOs, mapping)
- `CONFIG_TOF_COUNT`: number of sensors, indexed 0..N-1.【F:main/Kconfig.projbuild†L140-L146】
- `CONFIG_TOF_BIN_ALLOW_DUPLICATES`: allow duplicate bin indices
  (disabled = duplicate detection in `scan_engine`).【F:main/Kconfig.projbuild†L148-L152】【F:main/scan_engine.c†L37-L63】
- I2C bus:
  - `CONFIG_TOF_I2C_SDA_GPIO`, `CONFIG_TOF_I2C_SCL_GPIO`, `CONFIG_TOF_I2C_FREQ_HZ`.
    【F:main/Kconfig.projbuild†L154-L170】
- Timing/IRQ:
  - `CONFIG_TOF_TIMING_BUDGET_US`, `CONFIG_TOF_GPIO_READY_TIMEOUT_MS`.
    【F:main/Kconfig.projbuild†L172-L182】
- Per-sensor (`TOF_0_*`, `TOF_1_*`, …):
  - `*_XSHUT_GPIO`, `*_INT_GPIO`, `*_ADDR_7B`, `*_BIN_IDX`.
    【F:main/Kconfig.projbuild†L184-L336】

### micro-ROS status LED
- `CONFIG_MICRO_ROS_STATUS_LED_ENABLE`: enable the status LED.【F:main/Kconfig.projbuild†L115-L119】
- `CONFIG_MICRO_ROS_STATUS_LED_GPIO`: LED GPIO.【F:main/Kconfig.projbuild†L121-L126】
- `CONFIG_MICRO_ROS_STATUS_LED_BRIGHTNESS`: brightness scaling (0-255).【F:main/Kconfig.projbuild†L128-L134】

### ToF provider selection
- `CONFIG_TOF_PROVIDER_MOCK`: selects the mock provider (otherwise VL53L0X).
  Used by CMake to pick the implementation at build time.
  【F:main/Kconfig.projbuild†L342-L349】【F:main/CMakeLists.txt†L10-L16】

## 3) ROS QoS & timing

- **LaserScan QoS**
  - KEEP_LAST depth=`CONFIG_MICRO_ROS_QOS_DEPTH`.
  - RELIABLE or BEST_EFFORT based on `CONFIG_MICRO_ROS_QOS_RELIABLE`.
  - DURABILITY = VOLATILE to avoid XRCE backpressure.
  【F:main/micro_ros_adapter.c†L342-L367】

- **Publish period**
  - `CONFIG_MICRO_ROS_TIMER_PERIOD_MS` drives the `scan_pub_task` timer.
  【F:main/micro_ros_adapter.c†L333-L414】

- **Derived LaserScan parameters**
  - `scan_time = period_ms / 1000.0`, `bins = CONFIG_MICRO_ROS_SCAN_BINS`.
  - `angle_min = -π`, `angle_increment = 2π / bins`.
  - `range_min = 0.03m`, `range_max = 2.0m`.
  【F:main/micro_ros_adapter.c†L47-L58】【F:main/micro_ros_adapter.c†L286-L294】

- **Timestamping**
  - `rmw_uros_epoch_nanos()` when time sync succeeds.
  - Fallback to `esp_timer_get_time()` (warning logged) if sync unavailable.
  【F:main/micro_ros_adapter.c†L383-L383】【F:main/scan_engine.c†L66-L90】

## 4) ToF provider behavior

### Mock (simulation)
- Generates sinusoidal ranges and a moving “cliff” (large distance).
- Injects invalid bursts (status=4, range=NaN) on a rotating sensor.
- Updates at 50 Hz (20 ms delay).
  【F:main/tof_provider_mock.c†L117-L173】

### VL53L0X (hardware)
- Initializes I2C + timing budget via `vl53l0x_i2c_master_init()`.
- Assigns addresses via XSHUT (`vl53l0x_multi_assign_addresses`).
- Data-ready IRQ on GPIO, read protected by I2C mutex, then clear interrupt.
- Dedicated task per sensor with exponential backoff on GPIO/I2C errors.
  【F:main/tof_provider_vl53.c†L220-L361】【F:main/tof_provider_vl53.c†L138-L209】

### ToF status interpretation
- `status = 0`: valid measurement (VL53 RangeStatus OK).
- `status = 250`: I2C mutex timeout (blocked read).
- `status = 251`: GPIO data-ready wait timeout.
- `status = 252`: snapshot timeout (lock-free read exceeded time/spin).
- `status = 255`: generic invalid state (init/uncategorized error).
  【F:main/tof_provider_vl53.c†L138-L209】【F:main/tof_provider_mock.c†L9-L16】【F:main/tof_provider_vl53.c†L9-L16】

## 5) Troubleshooting / common errors

### micro-ROS agent lost
- The task waits for the agent then initializes RCL.
- Repeated `rmw_uros_ping_agent()` failures trigger a session restart.
  【F:main/micro_ros_adapter.c†L316-L517】

**Symptoms**
- Logs: “micro-ROS agent lost, restarting session”.
- LED blinking in waiting mode.

**Actions**
- Check the USB-CDC transport and ensure the agent is running.
- Verify `CONFIG_MICRO_ROS_DOMAIN_ID` on both agent and device.
  【F:main/main.c†L17-L41】【F:main/Kconfig.projbuild†L12-L18】

### Time sync unavailable
- Time sync attempts up to 5 times, then warns and falls back to `esp_timer_get_time()`.
  【F:main/micro_ros_adapter.c†L169-L184】【F:main/scan_engine.c†L66-L90】

**Symptoms**
- Logs: “Time sync failed; timestamps may be unset” / fallback timestamp.

**Actions**
- Verify the micro-ROS agent (time sync) and XRCE transport.

### ToF snapshot timeout
- Lock-free snapshots can time out if a sensor is mid-update; returns
  `status=252` and `range=NaN`.
  【F:main/tof_provider_mock.c†L81-L129】【F:main/tof_provider_vl53.c†L81-L129】

**Symptoms**
- Logs: “Snapshot timeout (idx=…)”.

**Actions**
- Check I2C/IRQ delays (e.g., `CONFIG_TOF_GPIO_READY_TIMEOUT_MS`).
- Reduce CPU load or increase publish period.
  【F:main/Kconfig.projbuild†L154-L164】【F:main/Kconfig.projbuild†L36-L42】

### Congestion / publish failures
- Repeated publish errors trigger backoff (`publish_backoff_cycles`).
- Congestion is detected when the agent is reachable but publishes fail.
  【F:main/micro_ros_adapter.c†L232-L260】【F:main/micro_ros_adapter.c†L480-L505】

**Actions**
- Switch to BEST_EFFORT if RELIABLE causes XRCE backpressure.
- Lower publish frequency (increase `CONFIG_MICRO_ROS_TIMER_PERIOD_MS`).
  【F:main/micro_ros_adapter.c†L342-L367】【F:main/Kconfig.projbuild†L36-L42】

### Common VL53 errors
- `status=250/251`: I2C/GPIO timeouts (check wiring, IRQ, I2C pull-ups).
- `status=255`: invalid init/measurement (sensor not detected or init failed).
  【F:main/tof_provider_vl53.c†L138-L209】【F:main/tof_provider_vl53.c†L220-L321】

## 6) Integration notes

- The ToF provider is selected **at build time** via `CONFIG_TOF_PROVIDER_MOCK`.
  No runtime toggle is implemented; adjust via menuconfig.
  【F:main/Kconfig.projbuild†L313-L321】【F:main/CMakeLists.txt†L10-L16】
- LaserScan bins are mapped per sensor via `TOF_*_BIN_IDX`. Duplicates are
  rejected when `CONFIG_TOF_BIN_ALLOW_DUPLICATES=n`.
  【F:main/Kconfig.projbuild†L166-L311】【F:main/scan_engine.c†L37-L63】

---

> ✅ This document covers architecture, configuration, ToF providers,
> ROS QoS/timing, and troubleshooting procedures for integration and field debug.
