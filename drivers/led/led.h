#ifndef LED_H
#define LED_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "hal_gpio.h"

typedef struct {
    gpio_init gpio;
    gpio_state active;
} led_obj;

int led_init(const led_obj *leds, size_t count);
void led_set(uint8_t id, bool on);
void led_toggle(uint8_t id);
void led_reset(void);

#endif // LED_H
