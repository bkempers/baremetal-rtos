#include "board.h"
#include "led.h"
#include "stm32h7rs_hal_rcc.h"

void board_clock_init(void) { SystemClock_Config(); }

void SystemClock_Config(void) {
    RCC_OscInit oscillator_def;
    oscillator_def.OscillatorType = RCC_OSCILLATOR_TYPE_HSI;
    oscillator_def.HSEState       = RCC_HSE_OFF;
    oscillator_def.HSIState       = RCC_HSI_ON;
    oscillator_def.HSICalibration = 0;
    if (HAL_RCC_OscConfig(&oscillator_def) != HAL_OK) {
        Error_Handler();
    }

    RCC_PLLInit pll_def;
    pll_def.PLLState  = RCC_PLL_ON;
    pll_def.PLLSource = RCC_OSCILLATOR_TYPE_HSI;
    pll_def.PLLM      = 4;
    pll_def.PLLN      = 12;
    pll_def.PLLP      = 2;
    pll_def.PLLQ      = 2;
    pll_def.PLLR      = 2;
    pll_def.PLLFRACN  = 4096;
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
    if (HAL_RCC_ClockConfig(&clkinit_def, FLASH_ACR_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void) {
    __disable_irq();
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    __enable_irq();
    while (1) { 
        Led_Error();
    }
}
