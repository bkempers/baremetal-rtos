#include "hal_gpio.h"

#include "stm32_assert.h"
#include "stm32h7rsxx_ll_bus.h"
#include "stm32h7rsxx_ll_gpio.h"

#define ARRAY_COUNT(a) (sizeof(a) / sizeof((a)[0]))

static GPIO_TypeDef *const gpio_base[GPIO_PORT_COUNT] = {
    [GPIO_PORT_A] = GPIOA,
    [GPIO_PORT_B] = GPIOB,
    [GPIO_PORT_C] = GPIOC,
    [GPIO_PORT_D] = GPIOD,
    [GPIO_PORT_E] = GPIOE,
    [GPIO_PORT_F] = GPIOF,
    [GPIO_PORT_G] = GPIOG,
    [GPIO_PORT_H] = GPIOH
};

/* Looked up, not computed: the AHB4ENR gate bits are only contiguous for
 * A..H (bits 0..7) — M..P sit at bits 12..15 on this family. */
static const uint32_t gpio_ahb4_periph[GPIO_PORT_COUNT] = {
    [GPIO_PORT_A] = LL_AHB4_GRP1_PERIPH_GPIOA,
    [GPIO_PORT_B] = LL_AHB4_GRP1_PERIPH_GPIOB,
    [GPIO_PORT_C] = LL_AHB4_GRP1_PERIPH_GPIOC,
    [GPIO_PORT_D] = LL_AHB4_GRP1_PERIPH_GPIOD,
    [GPIO_PORT_E] = LL_AHB4_GRP1_PERIPH_GPIOE,
    [GPIO_PORT_F] = LL_AHB4_GRP1_PERIPH_GPIOF,
    [GPIO_PORT_G] = LL_AHB4_GRP1_PERIPH_GPIOG,
    [GPIO_PORT_H] = LL_AHB4_GRP1_PERIPH_GPIOH
};

/* hal_gpio.h's speed/pull values already match the register encodings LL uses,
 * but map them so the contract is not coupled to that coincidence. */
static const uint32_t gpio_ll_speed[] = {
    [GPIO_SPEED_FREQ_LOW]       = LL_GPIO_SPEED_FREQ_LOW,
    [GPIO_SPEED_FREQ_MED]       = LL_GPIO_SPEED_FREQ_MEDIUM,
    [GPIO_SPEED_FREQ_HIGH]      = LL_GPIO_SPEED_FREQ_HIGH,
    [GPIO_SPEED_FREQ_VERY_HIGH] = LL_GPIO_SPEED_FREQ_VERY_HIGH
};

static const uint32_t gpio_ll_pull[] = {
    [GPIO_NOPULL_UP] = LL_GPIO_PULL_NO,
    [GPIO_PULL_UP]   = LL_GPIO_PULL_UP,
    [GPIO_PULL_DOWN] = LL_GPIO_PULL_DOWN
};

/* Ordinal pin -> the LL_GPIO_PIN_n bitmask LL expects. */
#define GPIO_LL_PIN(pin) (1uL << (pin))

/* gpio_init.mode packs direction (bits 0..1) with the output driver (bit 4);
 * LL takes the two as separate fields. */
static uint32_t gpio_ll_mode(uint32_t mode)
{
    switch (mode & GPIO_MODE) {
    case MODE_INPUT:
        return LL_GPIO_MODE_INPUT;
    case MODE_OUTPUT:
        return LL_GPIO_MODE_OUTPUT;
    case MODE_ALT_FUNC:
        return LL_GPIO_MODE_ALTERNATE;
    default:
        return LL_GPIO_MODE_ANALOG;
    }
}

static uint32_t gpio_ll_output_type(uint32_t mode)
{
    return (((mode >> GPIO_TYPE_Pos) & 0x1uL) == TYPE_OD) ? LL_GPIO_OUTPUT_OPENDRAIN : LL_GPIO_OUTPUT_PUSHPULL;
}

void hal_gpio_init(const gpio_init *obj)
{
    /* speed/pull index the tables above, so bad values would read past them. */
    assert_param(obj->gpio.port < GPIO_PORT_COUNT);
    assert_param(obj->speed < ARRAY_COUNT(gpio_ll_speed));
    assert_param(obj->pull < ARRAY_COUNT(gpio_ll_pull));

    /* Writes to a gated port are silently dropped; LL_AHB4_GRP1_EnableClock
     * reads the gate back itself, so the clock is up before MODER. */
    LL_AHB4_GRP1_EnableClock(gpio_ahb4_periph[obj->gpio.port]);

    LL_GPIO_InitTypeDef init = {
        .Pin        = GPIO_LL_PIN(obj->gpio.pin),
        .Mode       = gpio_ll_mode(obj->mode),
        .Speed      = gpio_ll_speed[obj->speed],
        .OutputType = gpio_ll_output_type(obj->mode),
        .Pull       = gpio_ll_pull[obj->pull],
        .Alternate  = obj->alternate, /* LL_GPIO_AF_n == n */
    };

    (void) LL_GPIO_Init(gpio_base[obj->gpio.port], &init);
}

// TODO: ADD LATER
void hal_gpio_deinit(gpio *obj)
{
    (void) obj;
    return;
}

gpio_state hal_gpio_read_output(gpio *obj)
{
    return LL_GPIO_IsOutputPinSet(gpio_base[obj->port], GPIO_LL_PIN(obj->pin)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

gpio_state hal_gpio_read_input(gpio *obj)
{
    return LL_GPIO_IsInputPinSet(gpio_base[obj->port], GPIO_LL_PIN(obj->pin)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void hal_gpio_write(gpio *obj, gpio_state state)
{
    if (state != GPIO_PIN_RESET) {
        LL_GPIO_SetOutputPin(gpio_base[obj->port], GPIO_LL_PIN(obj->pin));
    } else {
        LL_GPIO_ResetOutputPin(gpio_base[obj->port], GPIO_LL_PIN(obj->pin));
    }
}

void hal_gpio_toggle(gpio *obj)
{
    LL_GPIO_TogglePin(gpio_base[obj->port], GPIO_LL_PIN(obj->pin));
}
