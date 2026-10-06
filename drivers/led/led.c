#include "led.h"

/* Named led_table, not leds: led_init()'s parameter is called leds, and a
 * same-named static here silently shadowed it. */
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

// void led_toggle(uint8_t led_num)
// {
//     switch (led_num) {
//     case 1:
//         HAL_GPIO_Toggle(GPIOD, LED1_PIN);
//         break; // Green
//     case 2:
//         HAL_GPIO_Toggle(GPIOD, LED2_PIN);
//         break; // Yellow
//     case 3:
//         HAL_GPIO_Toggle(GPIOB, LED3_PIN);
//         break; // Red
//     }
// }
//
// void led_reset(void)
// {
//     HAL_GPIO_Write(GPIOD, LED1_PIN, GPIO_PIN_RESET);
//     HAL_GPIO_Write(GPIOD, LED2_PIN, GPIO_PIN_RESET);
//     HAL_GPIO_Write(GPIOB, LED3_PIN, GPIO_PIN_RESET);
// }
//

