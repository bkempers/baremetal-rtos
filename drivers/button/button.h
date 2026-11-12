#ifndef BUTTON_H
#define BUTTON_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "hal_gpio.h"

typedef struct {
    gpio_init gpio;
} button_obj;

int button_init(const button_obj *button, size_t count);

/* blocking */
bool button_read(uint8_t id);

#endif // BUTTON_H
