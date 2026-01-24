#include <stdio.h>

#include "esp_log.h"
#include "esp_usbcdc_logging.h"
#include "esp_usbcdc_transport.h"

#include <rmw_microros/rmw_microros.h>

#include "tof_provider.h"
#include "uros_app_scan.h"

static const char *TAG_MAIN = "MAIN";

// USB-CDC interface
static tinyusb_cdcacm_itf_t cdc_port = TINYUSB_CDC_ACM_0;

void app_main(void)
{
#if (CONFIG_TINYUSB_CDC_COUNT >= 2)
    if (esp_usbcdc_logging_init() == ESP_OK) {
        ESP_LOGI(TAG_MAIN, "USB-CDC Logging initialized");
    }
#endif

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_ret_t ret = rmw_uros_set_custom_transport(
        true,
        (void *)&cdc_port,
        esp_usbcdc_open,
        esp_usbcdc_close,
        esp_usbcdc_write,
        esp_usbcdc_read);

    if (ret != RMW_RET_OK) {
        ESP_LOGE(TAG_MAIN, "Failed to set micro-ROS custom transport layer");
        return;
    }
#else
#error micro-ROS transports misconfigured (RMW_UXRCE_TRANSPORT_CUSTOM expected)
#endif

    // Initialize ToF provider selected by build config (mock or VL53)
    tof_provider_init();

    // Start micro-ROS app (LaserScan publisher)
    if (!uros_app_scan_start()) {
        ESP_LOGE(TAG_MAIN, "Failed to start micro-ROS app task");
        return;
    }

    ESP_LOGI(TAG_MAIN, "Application started");
}
