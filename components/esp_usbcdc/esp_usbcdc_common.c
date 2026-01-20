#include "esp_usbcdc_common.h"

esp_err_t esp_usbcdc_tinyusb_init_once(const tinyusb_config_t *tinyusb_config)
{
    static bool tinyusb_initialized = false;

    if (tinyusb_initialized) {
        return ESP_OK;
    }

    if (tinyusb_config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = tinyusb_driver_install(tinyusb_config);

    if (ret == ESP_OK) {
        tinyusb_initialized = true;
    }

    return ret;
}
