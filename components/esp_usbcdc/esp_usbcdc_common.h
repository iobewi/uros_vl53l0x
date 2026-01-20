#ifndef ESP_USBCDC_COMMON_H
#define ESP_USBCDC_COMMON_H

#include "esp_err.h"
#include "tinyusb.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t esp_usbcdc_tinyusb_init_once(const tinyusb_config_t *tinyusb_config);

#ifdef __cplusplus
}
#endif

#endif // ESP_USBCDC_COMMON_H
