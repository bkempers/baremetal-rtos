#ifndef BUTTON_H
#define BUTTON_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "hal.h"
#include "hal_gpio.h"

#define BUTTON_DEBOUNCE_MS 20u

typedef struct {
    gpio_init gpio;
    uint32_t pressed_time;
    bool state;
} button_obj;

int button_init(const button_obj *button, size_t count);

/* blocking */
bool button_pressed(uint8_t id);

#endif // BUTTON_H
