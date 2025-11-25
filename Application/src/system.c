#include "system.h"

void SystemClock_Config()
{
    RCC_OscInit oscillator_def;
    oscillator_def.OscillatorType = RCC_OSCILLATOR_TYPE_HSI;
    oscillator_def.HSEState       = RCC_HSE_OFF;
    oscillator_def.HSIState       = RCC_HSI_ON;
    oscillator_def.HSICalibration = 32;
    if (HAL_RCC_OscConfig(&oscillator_def) != HAL_OK) {
        Error_Handler();
    }

    // TODO: disregard PLL for now...
    RCC_PLLInit pll_def;
    pll_def.PLLState  = RCC_PLL_OFF;
    pll_def.PLLSource = RCC_OSCILLATOR_TYPE_HSI;
    pll_def.PLLM      = 0;
    pll_def.PLLN      = 0;
    pll_def.PLLP      = 0;
    pll_def.PLLQ      = 0;
    pll_def.PLLR      = 0;
    pll_def.PLLFRACN  = 0;
    if (HAL_RCC_PLLConfig(&pll_def) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInit clkinit_def;
    clkinit_def.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    clkinit_def.AHBCLKDivider  = RCC_HCLK_DIV1;
    clkinit_def.APB1CLKDivider = RCC_APB1_DIV1;
    clkinit_def.APB2CLKDivider = RCC_APB2_DIV1;
    clkinit_def.APB4CLKDivider = RCC_APB4_DIV1;
    clkinit_def.APB5CLKDivider = RCC_APB5_DIV1;

    uint32_t flash_latency = FLASH_ACR_LATENCY_3;
    if (HAL_RCC_ClockConfig(&clkinit_def, flash_latency) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler()
{
    __disable_irq();

    // Re-enable ONLY SysTick
    __set_BASEPRI(0);             // Clear BASEPRI
    NVIC_EnableIRQ(SysTick_IRQn); // Won't work - SysTick is different

    // Better: Don't use NVIC, just enable SysTick directly
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // Enable SysTick interrupt
    __enable_irq();                            // Re-enable interrupts globally

    while (1) {
        // flash red LED
        Led_Error();
    }
}
