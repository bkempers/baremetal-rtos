#include "hal_gpio.h"
#include "stm32h7rsxx.h"

static GPIO_TypeDef *const GPIOx[GPIO_PORT_COUNT] = {
    [GPIO_PORT_A] = GPIOA,
    [GPIO_PORT_B] = GPIOB,
    [GPIO_PORT_C] = GPIOC,
    [GPIO_PORT_D] = GPIOD,
    [GPIO_PORT_E] = GPIOE,
    [GPIO_PORT_F] = GPIOF,
    [GPIO_PORT_G] = GPIOG,
    [GPIO_PORT_H] = GPIOH
}

void hal_gpio_init(const GPIO_Init *gpio)
{
    uint32_t position = 0;
    uint32_t temp     = 0;

    while ((gpio->pin >> position) != 0) {
        if ((gpio->pin & (1U << position)) != 0) {
            // Configure mode
            temp = GPIOx->MODER;
            temp &= ~(3U << (position * 2)); // Clear 2 bits
            temp |= (gpio->mode << (position * 2));
            GPIOx->MODER = temp;

            // Configure output type (push-pull or open-drain)
            if (gpio->mode == GPIO_MODE_OUTPUT_PP || gpio->mode == GPIO_MODE_ALT_FUNC_PP) {
                GPIOx->OTYPER &= ~(1U << position); // Push-pull
            } else if (gpio->mode == GPIO_MODE_OUTPUT_OD || gpio->mode == GPIO_MODE_ALT_FUNC_OD) {
                GPIOx->OTYPER |= (1U << position); // Open-drain
            }

            // Configure speed
            temp = GPIOx->OSPEEDR;
            temp &= ~(3U << (position * 2));
            temp |= (gpio->speed << (position * 2));
            GPIOx->OSPEEDR = temp;

            // Configure pull-up/pull-down
            temp = GPIOx->PUPDR;
            temp &= ~(3U << (position * 2));
            temp |= (gpio->pull << (position * 2));
            GPIOx->PUPDR = temp;

            // Configure alternate function
            if (gpio->mode == GPIO_MODE_ALT_FUNC_PP || gpio->mode == GPIO_MODE_ALT_FUNC_OD) {
                temp = (position < 8) ? GPIOx->AFR[0] : GPIOx->AFR[1];
                uint32_t af_position = (position < 8) ? position : (position - 8);
                temp &= ~(0xFU << (af_position * 4));
                temp |= (gpio->alternate << (af_position * 4));
                if (position < 8) {
                    GPIOx->AFR[0] = temp;
                } else {
                    GPIOx->AFR[1] = temp;
                }
            }
        }
        position++;
    }
}

// TODO: ADD LATER
void hal_gpio_deinit(uint16_t pin)
{
    return;
}

gpio_state hal_gpio_read(uint16_t pin)
{
    GPIO_PinState ret = (GPIOx->ODR & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    return ret;
}

void hal_gpio_write(uint16_t pin, gpio_state state)
{
    if (state != GPIO_PIN_RESET) {
        GPIOx->BSRR = (uint32_t) pin;
    } else {
        GPIOx->BRR = (uint32_t) pin;
    }
}

void hal_gpio_toggle(uint16_t pin)
{
    GPIOx->ODR ^= pin;
}
