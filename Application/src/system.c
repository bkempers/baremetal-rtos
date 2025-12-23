#include <stdio.h>
#include <string.h>
#include "console.h"

#include "system.h"

static int system_handler(int argc, char **argv) {
    if (strcmp(argv[1], "info") == 0) {
        PRINT_INFO("STM32H7RS Serial Console");
    } else if (strcmp(argv[1], "ver") == 0) {
        PRINT_INFO("VERSION: %u.%u.%u", MAJOR_VER, MINOR_VER, PATCH_VER);
    } else if (strcmp(argv[1], "clock") == 0) {
        PRINT_INFO("CLOCK: %.1f MHz", (SystemCoreClock / 1e6));
    } else if (strcmp(argv[1], "git") == 0) {
        PRINT_INFO("GIT BRANCH: %s & HASH: %s", GIT_BRANCH, GIT_COMMIT_SHORT);
    } else if (strlen(*argv) > 1) {
        PRINT_INFO("Unknown system argument %s", argv[1]);
        return 1;
    }

    return 0;
}
SHELL_COMMAND_REGISTER(system, system_handler, "Access system information");

void SystemClock_Config()
{
    RCC_OscInit oscillator_def;
    oscillator_def.OscillatorType = RCC_OSCILLATOR_TYPE_HSI;
    oscillator_def.HSEState       = RCC_HSE_OFF;
    oscillator_def.HSIState       = RCC_HSI_ON;
    // oscillator_def.HSICalibration = 32;
    oscillator_def.HSICalibration = 0;
    if (HAL_RCC_OscConfig(&oscillator_def) != HAL_OK) {
        Error_Handler();
    }

    // TODO: disregard PLL for now...
    RCC_PLLInit pll_def;
    pll_def.PLLState  = RCC_PLL_OFF;
    pll_def.PLLSource = RCC_OSCILLATOR_TYPE_HSI;
    pll_def.PLLM      = 4;
    pll_def.PLLN      = 50;
    pll_def.PLLP      = 2;
    pll_def.PLLQ      = 2;
    pll_def.PLLR      = 2;
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
