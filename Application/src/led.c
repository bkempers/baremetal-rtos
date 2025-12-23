#include <stdio.h>

#include "shell_command.h"
#include "console.h"
#include "led.h"
#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_gpio.h"
#include "stm32h7rsxx.h"

static int led_handler(int argc, char **argv) {
    PRINT_INFO("LED command called with %d args", argc);
    return 0;
}
SHELL_COMMAND_REGISTER(led, led_handler, "Control LEDs");

void Led_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // Configure LED pins as outputs
    // LED1 (PD10) // Green
    GPIO_Init gpio_led_1;
    gpio_led_1.Pin   = LED1_PIN;
    gpio_led_1.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_led_1.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_led_1.Pull  = GPIO_NOPULL_UP;
    HAL_GPIO_Init(GPIOD, &gpio_led_1);

    // LED2 (PD13) // Yellow
    GPIO_Init gpio_led_2;
    gpio_led_2.Pin   = LED2_PIN;
    gpio_led_2.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_led_2.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_led_2.Pull  = GPIO_NOPULL_UP;
    HAL_GPIO_Init(GPIOD, &gpio_led_2);

    // LED3 (PB7) // Red
    GPIO_Init gpio_led_3;
    gpio_led_3.Pin   = LED3_PIN;
    gpio_led_3.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_led_3.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_led_3.Pull  = GPIO_NOPULL_UP;
    HAL_GPIO_Init(GPIOB, &gpio_led_3);
}

void Led_Toggle(uint8_t led_num)
{
    switch (led_num) {
    case 1:
        HAL_GPIO_Toggle(GPIOD, LED1_PIN);
        break; // Green
    case 2:
        HAL_GPIO_Toggle(GPIOD, LED2_PIN);
        break; // Yellow
    case 3:
        HAL_GPIO_Toggle(GPIOB, LED3_PIN);
        break; // Red
    }
}

void Led_Reset(void)
{
    HAL_GPIO_Write(GPIOD, LED1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_Write(GPIOD, LED2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_Write(GPIOB, LED3_PIN, GPIO_PIN_RESET);
}

void Led_Cycle(void)
{
    Led_Toggle(1);
    HAL_DelayMS(50);

    Led_Toggle(2);
    HAL_DelayMS(50);

    Led_Toggle(3);
    HAL_DelayMS(50);
}

void Led_Error(void)
{
    Led_Toggle(3);
    HAL_DelayMS(75);
}
