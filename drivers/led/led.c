#include "led.h"

static const led_obj *led_table;
static size_t led_count;

static const led_obj *led_get(uint8_t id)
{
    if (led_table == NULL || (size_t)id >= led_count || led_table[id].gpio.gpio.port == GPIO_PORT_NONE) {
        return NULL;   /* this board does not have this LED */
    }
    return &led_table[id];
}

int led_init(const led_obj *leds, size_t count)
{
    if (leds == NULL || count == 0) {
        return -1;
    }

    led_table = leds;
    led_count = count;

    for (size_t i = 0; i < count; i++) {
        if (leds[i].gpio.gpio.port != GPIO_PORT_NONE) {
            hal_gpio_init(&leds[i].gpio);
        }
    }

    led_reset();
    return 0;
}

void led_set(uint8_t id, bool on)
{
    const led_obj *led = led_get(id);
    if (led == NULL) {
        return;
    }
    gpio_state state = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
    gpio obj = led->gpio.gpio;
    hal_gpio_write(&obj, state);
}

void led_toggle(uint8_t id)
{
    const led_obj *led = led_get(id);
    if (led != NULL) {
        gpio obj = led->gpio.gpio;
        hal_gpio_toggle(&obj);
    }
}

void led_reset(void)
{
    for (size_t i = 0; i < led_count; i++) {
        led_set(i, false);
    }
}
