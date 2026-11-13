#include "button.h"

#define BUTTON_MAX 4u

static struct {
    uint32_t pressed_time;
    bool state;
} button_state[BUTTON_MAX];

static const button_obj *button_table;
static size_t button_count;

static const button_obj *button_get(uint8_t id)
{
    if (button_table == NULL || (size_t)id >= button_count || button_table[id].gpio.gpio.port == GPIO_PORT_NONE) {
        return NULL;
    }
    return &button_table[id];
}

int button_init(const button_obj *buttons, size_t count)
{
    if (buttons == NULL || count == 0) {
        return -1;
    }

    button_table = buttons;
    button_count = count;

    for (size_t i = 0; i < count; i++) {
        if (buttons[i].gpio.gpio.port != GPIO_PORT_NONE) {
            hal_gpio_init(&buttons[i].gpio);
        }
    }

    return 0;
}

bool button_pressed(uint8_t id)
{
    const button_obj *button = button_get(id);
    if (button == NULL) {
        return false;
    }

    gpio obj = button->gpio.gpio;
    uint32_t now = hal_time_ms();

    // debounce check
    if ((uint32_t) (now - button_state[id].pressed_time) < BUTTON_DEBOUNCE_MS) {
        return false;
    }

    bool pressed = (hal_gpio_read_input(&obj) == GPIO_PIN_RESET);
    if (pressed == button_state[id].state) {
        return false;
    }

    button_state[id].pressed_time = now;
    button_state[id].state = pressed;

    return pressed;
}
