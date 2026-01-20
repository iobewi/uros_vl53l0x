idf_component_register(SRCS
    "esp_usbcdc_common.c"
    "esp_usbcdc_logging.c"
    "esp_usbcdc_transport.c"
    INCLUDE_DIRS "."
    REQUIRES esp_tinyusb micro_ros_espidf_component # include <uxr/client/transport.h>
)
