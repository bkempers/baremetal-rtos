#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdint.h>

/* A gpio_init.mode packs two fields so callers get one "what is this pin"
 * knob: the 2-bit direction (MODE) plus the 1-bit output driver (TYPE).
 * The chip layer must mask them apart before touching registers. */
#define GPIO_MODE_Pos 0u
#define GPIO_MODE     (0x3uL << GPIO_MODE_Pos)
#define MODE_INPUT    0x0uL
#define MODE_OUTPUT   0x1uL
#define MODE_ALT_FUNC 0x2uL
#define MODE_ANALOG   0x3uL

#define GPIO_TYPE_Pos 4u
#define TYPE_PP       0x0uL
#define TYPE_OD       0x1uL

#define GPIO_MODE_INPUT       (MODE_INPUT << GPIO_MODE_Pos)
#define GPIO_MODE_OUTPUT_PP   ((MODE_OUTPUT << GPIO_MODE_Pos) | (TYPE_PP << GPIO_TYPE_Pos))
#define GPIO_MODE_OUTPUT_OD   ((MODE_OUTPUT << GPIO_MODE_Pos) | (TYPE_OD << GPIO_TYPE_Pos))
#define GPIO_MODE_ALT_FUNC_PP ((MODE_ALT_FUNC << GPIO_MODE_Pos) | (TYPE_PP << GPIO_TYPE_Pos))
#define GPIO_MODE_ALT_FUNC_OD ((MODE_ALT_FUNC << GPIO_MODE_Pos) | (TYPE_OD << GPIO_TYPE_Pos))
#define GPIO_MODE_ANALOG      (MODE_ANALOG << GPIO_MODE_Pos)

#define GPIO_SPEED_FREQ_LOW       0x00u
#define GPIO_SPEED_FREQ_MED       0x01u
#define GPIO_SPEED_FREQ_HIGH      0x02u
#define GPIO_SPEED_FREQ_VERY_HIGH 0x03u

#define GPIO_NOPULL_UP 0x00u
#define GPIO_PULL_UP   0x01u
#define GPIO_PULL_DOWN 0x02u

typedef enum {
    GPIO_PORT_NONE = 0,
    GPIO_PORT_A,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_F,
    GPIO_PORT_G,
    GPIO_PORT_H,
    GPIO_PORT_COUNT
} gpio_port;

/* Ordinal, not a bitmask: GPIO_PIN_10 == 10. One gpio value addresses exactly
 * one pin; the chip layer shifts it into whatever register layout it needs. */
typedef enum {
    GPIO_PIN_0 = 0,
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_5,
    GPIO_PIN_6,
    GPIO_PIN_7,
    GPIO_PIN_8,
    GPIO_PIN_9,
    GPIO_PIN_10,
    GPIO_PIN_11,
    GPIO_PIN_12,
    GPIO_PIN_13,
    GPIO_PIN_14,
    GPIO_PIN_15,
    GPIO_PIN_COUNT
} gpio_pin;

typedef enum {
    GPIO_PIN_RESET = 0U,
    GPIO_PIN_SET
} gpio_state;

typedef struct {
    gpio_port port;
    gpio_pin pin;
} gpio;

typedef struct {
    gpio gpio;
    uint32_t mode;
    uint32_t speed;
    uint32_t pull;
    uint32_t alternate;
} gpio_init;

void hal_gpio_init(const gpio_init *gpio);
void hal_gpio_deinit(gpio *obj);
gpio_state hal_gpio_read(gpio *obj);
void hal_gpio_write(gpio *obj, gpio_state state);
void hal_gpio_toggle(gpio *obj);

#endif // HAL_GPIO_H
