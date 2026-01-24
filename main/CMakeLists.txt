set(PRIV_REQUIRES_LIST
    micro_ros_espidf_component
    esp_driver_uart
    esp_driver_rmt
    esp_timer
    esp_usbcdc
    vl53l0x
)

if(CONFIG_MICRO_ROS_STATUS_LED_ENABLE)
    list(APPEND PRIV_REQUIRES_LIST led_strip)
endif()

set(TOF_PROVIDER_SRC "tof_provider_vl53.c")
if(CONFIG_TOF_PROVIDER_MOCK)
    set(TOF_PROVIDER_SRC "tof_provider_mock.c")
endif()

idf_component_register(
    SRCS
        "main.c"
        "uros_core.c"
        "uros_app_scan.c"
        "led_status.c"
        "scan_builder.c"
        "scan_engine.c"
        "tof_config.c"
        "tof_provider.c"
        "tof_snapshot.c"
        ${TOF_PROVIDER_SRC}
    INCLUDE_DIRS
        "."
    REQUIRES
        ${PRIV_REQUIRES_LIST}
)
