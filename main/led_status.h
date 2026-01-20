#ifndef LED_STATUS_H
#define LED_STATUS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LED_STATUS_OFF = 0,
    LED_STATUS_WAITING,
    LED_STATUS_CONNECTED,
    LED_STATUS_ERROR
} led_status_state_t;

static const uint8_t LED_STATUS_COLOR_WAITING[3] = {0, 0, 255};
static const uint8_t LED_STATUS_COLOR_CONNECTED[3] = {0, 255, 0};
static const uint8_t LED_STATUS_COLOR_ERROR[3] = {255, 0, 0};

bool led_status_start(void);
void led_status_set_state(led_status_state_t state);

#endif
