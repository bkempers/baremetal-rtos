#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_rcc.h"
#include <system.h>

#define PIN5                  (1U<<5)
#define PIN7                  (1U<<7)
#define PIN10                 (1U<<10)
#define PIN13                 (1U<<13)

/* LED DEFINES */

#define LED1_PIN              (PIN10) //PD10
#define LED2_PIN              (PIN13) //PD13
#define LED3_PIN              (PIN7) //PB7


void led_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // Configure LED pins as outputs
    // LED1 (PD10) and LED2 (PD13)
    GPIOD->MODER |= (1U<<20) | (1U<<26);  // Set bits for output mode
    GPIOD->MODER &= ~((1U<<21) | (1U<<27)); // Clear upper bits

    // LED3 (PB7)
    GPIOB->MODER |= (1U<<14);   // Set bit for output mode
    GPIOB->MODER &= ~(1U<<15);  // Clear upper bit
}

void led_toggle(int led_num)
{
    switch(led_num) {
        case 1: GPIOD->ODR ^= LED1_PIN; break;  // Green
        case 2: GPIOD->ODR ^= LED2_PIN; break;  // Yellow
        case 3: GPIOB->ODR ^= LED3_PIN; break;  // Red
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    SysTick_Config(64000000);
    __enable_irq();

    led_init();
    while(1)
    {
        led_toggle(1);
        HAL_DelayMS(500);
        // for(int i =0; i<500000; i++) {}

        led_toggle(2);
        HAL_DelayMS(500);
        // for(int i =0; i<500000; i++) {}

        led_toggle(3);
        HAL_DelayMS(500);
        // for(int i =0; i<500000; i++) {}

    }

	return 1;
}
