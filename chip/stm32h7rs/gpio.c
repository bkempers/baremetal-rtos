#include "hal_gpio.h"
#include "cmsis_device.h"

static GPIO_TypeDef *const GPIOx[GPIO_PORT_COUNT] = {
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
static const uint32_t gpio_ahb4_en[GPIO_PORT_COUNT] = {
    [GPIO_PORT_A] = RCC_AHB4ENR_GPIOAEN,
    [GPIO_PORT_B] = RCC_AHB4ENR_GPIOBEN,
    [GPIO_PORT_C] = RCC_AHB4ENR_GPIOCEN,
    [GPIO_PORT_D] = RCC_AHB4ENR_GPIODEN,
    [GPIO_PORT_E] = RCC_AHB4ENR_GPIOEEN,
    [GPIO_PORT_F] = RCC_AHB4ENR_GPIOFEN,
    [GPIO_PORT_G] = RCC_AHB4ENR_GPIOGEN,
    [GPIO_PORT_H] = RCC_AHB4ENR_GPIOHEN
};

/* Writes to a gated port are silently dropped, so init has to ungate first. */
static void gpio_clk_enable(gpio_port port)
{
    RCC->AHB4ENR |= gpio_ahb4_en[port];
    (void) RCC->AHB4ENR; /* read back so the clock is up before MODER */
}

void hal_gpio_init(const gpio_init *obj)
{
    uint32_t position = obj->gpio.pin; // ordinal 0..15, already the bit index
    uint32_t temp     = 0;

    gpio_clk_enable(obj->gpio.port);

    // Configure mode
    temp = GPIOx[obj->gpio.port]->MODER;
    temp &= ~(3U << (position * 2)); // Clear 2 bits
    temp |= ((obj->mode & GPIO_MODE) << (position * 2));
    GPIOx[obj->gpio.port]->MODER = temp;

    // Configure output type (push-pull or open-drain)
    if (obj->mode == GPIO_MODE_OUTPUT_PP || obj->mode == GPIO_MODE_ALT_FUNC_PP) {
        GPIOx[obj->gpio.port]->OTYPER &= ~(1U << position); // Push-pull
    } else if (obj->mode == GPIO_MODE_OUTPUT_OD || obj->mode == GPIO_MODE_ALT_FUNC_OD) {
        GPIOx[obj->gpio.port]->OTYPER |= (1U << position); // Open-drain
    }

    // Configure speed
    temp = GPIOx[obj->gpio.port]->OSPEEDR;
    temp &= ~(3U << (position * 2));
    temp |= (obj->speed << (position * 2));
    GPIOx[obj->gpio.port]->OSPEEDR = temp;

    // Configure pull-up/pull-down
    temp = GPIOx[obj->gpio.port]->PUPDR;
    temp &= ~(3U << (position * 2));
    temp |= (obj->pull << (position * 2));
    GPIOx[obj->gpio.port]->PUPDR = temp;

    // Configure alternate function
    if (obj->mode == GPIO_MODE_ALT_FUNC_PP || obj->mode == GPIO_MODE_ALT_FUNC_OD) {
        temp = (position < 8) ? GPIOx[obj->gpio.port]->AFR[0] : GPIOx[obj->gpio.port]->AFR[1];
        uint32_t af_position = (position < 8) ? position : (position - 8);
        temp &= ~(0xFU << (af_position * 4));
        temp |= (obj->alternate << (af_position * 4));
        if (position < 8) {
            GPIOx[obj->gpio.port]->AFR[0] = temp;
        } else {
            GPIOx[obj->gpio.port]->AFR[1] = temp;
        }
    }
}

// TODO: ADD LATER
void hal_gpio_deinit(gpio *obj)
{
    (void) obj;
    return;
}

gpio_state hal_gpio_read(gpio *obj)
{
    gpio_state ret = (GPIOx[obj->port]->ODR & (1U << obj->pin)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    return ret;
}

void hal_gpio_write(gpio *obj, gpio_state state)
{
    if (state != GPIO_PIN_RESET) {
        GPIOx[obj->port]->BSRR = (1U << obj->pin);
    } else {
        GPIOx[obj->port]->BRR = (1U << obj->pin);
    }
}

void hal_gpio_toggle(gpio *obj)
{
    GPIOx[obj->port]->ODR ^= (1U << obj->pin);
}
