set(PRIV_REQUIRES_LIST
    micro_ros_espidf_component
    esp_driver_uart
    esp_driver_rmt
    esp_timer
    esp_usbcdc_logging
    esp_usbcdc_transport
    vl53l0x
)

if(CONFIG_MICRO_ROS_STATUS_LED_ENABLE)
    list(APPEND PRIV_REQUIRES_LIST esp_driver_led_strip)
endif()

idf_component_register(
    SRCS
        "main.c"
        "uros_app.c"
        "led_status.c"
        "scan_builder.c"
        "tof_provider.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        ${PRIV_REQUIRES_LIST}
)
