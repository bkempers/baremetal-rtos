#include "stm32h7rsxx.h"
#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_rcc.h"
#include "stm32h7rs_hal_gpio.h"
#include "stm32h7rs_hal_xspi.h"

// Application base address in external flash
#define APP_ADDRESS 0x70000000

// Forward declarations
void boot_system_clock_config(void);
void boot_xspi_gpio_init(void);
int boot_xspi_init(void);
void boot_jump_to_application(void);
void Error_Handler(void);

// XSPI handle
XSPI_Handle hxspi2;

int main(void)
{
    // 1. HAL initialization
    HAL_Init();
    
    // 2. Configure system clock to 600 MHz using your RCC HAL
    boot_system_clock_config();
    
    // 3. Initialize XSPI GPIOs using your GPIO HAL
    boot_xspi_gpio_init();
    
    // 4. Initialize XSPI peripheral and flash
    if (boot_xspi_init() != 0) {
        // Error - could blink LED here
        while (1) {
            __NOP();
        }
    }
    
    // 5. Enable CPU caches
    SCB_EnableICache();
    SCB_EnableDCache();
    
    // 6. Jump to application
    boot_jump_to_application();
    
    // Should never reach here
    while (1);
}

void boot_system_clock_config(void)
{
    // Conservative 200 MHz configuration for boot
    // Application can reconfigure clocks if needed
    
    RCC_OscInit oscillator_def;
    oscillator_def.OscillatorType = RCC_OSCILLATOR_TYPE_HSI;
    oscillator_def.HSEState       = RCC_HSE_OFF;
    oscillator_def.HSIState       = RCC_HSI_ON;
    oscillator_def.HSICalibration = 0;
    
    if (HAL_RCC_OscConfig(&oscillator_def) != HAL_OK) {
        Error_Handler();
    }
    
    // PLL for 200 MHz: 64 / 4 * 50 / 2 / 2 = 200 MHz
    RCC_PLLInit pll_def;
    pll_def.PLLState  = RCC_PLL_ON;
    pll_def.PLLSource = RCC_OSCILLATOR_TYPE_HSI;
    pll_def.PLLM      = 4;   // 64 / 4 = 16 MHz
    pll_def.PLLN      = 50;  // 16 * 50 = 800 MHz
    pll_def.PLLP      = 2;   // 800 / 2 = 400 MHz
    pll_def.PLLQ      = 2;
    pll_def.PLLR      = 2;
    pll_def.PLLFRACN  = 0;
    
    if (HAL_RCC_PLLConfig(&pll_def) != HAL_OK) {
        Error_Handler();
    }
    
    RCC_ClkInit clkinit_def;
    clkinit_def.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clkinit_def.AHBCLKDivider  = RCC_HCLK_DIV2;   // 400 / 2 = 200 MHz
    clkinit_def.APB1CLKDivider = RCC_APB1_DIV1;   // 200 MHz
    clkinit_def.APB2CLKDivider = RCC_APB2_DIV1;
    clkinit_def.APB4CLKDivider = RCC_APB4_DIV1;
    clkinit_def.APB5CLKDivider = RCC_APB5_DIV1;
    
    uint32_t flash_latency = FLASH_ACR_LATENCY_3;  // 3 wait states for 200 MHz
    
    if (HAL_RCC_ClockConfig(&clkinit_def, flash_latency) != HAL_OK) {
        Error_Handler();
    }
}

void boot_xspi_gpio_init(void)
{
    GPIO_Init GPIO_InitStruct = {0};
    
    // Enable GPIO clocks using your HAL
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    
    // CRITICAL: Enable HSLV for XSPI2 Port 2
    __HAL_RCC_SBS_CLK_ENABLE();
    SBS->CCCSR |= SBS_CCCSR_XSPI2_IOHSLV;
    
    // Configure XSPI2 pins
    // All pins: Alternate Function, Very High Speed, No Pull
    
    GPIO_InitStruct.Mode = GPIO_MODE_ALT_FUNC_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL_UP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    
    // CLK: PF10 (AF9)
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Alternate = GPIO_AF9_XSPI2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    
    // NCS: PG6 (AF9)
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF9_XSPI2;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
    
    // DQS: PF8 (AF10)
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Alternate = GPIO_AF10_XSPI2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    
    // IO0: PD11 (AF9)
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Alternate = GPIO_AF9_XSPI2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    // IO1: PD12 (AF9)
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Alternate = GPIO_AF9_XSPI2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    // IO2: PE2 (AF9)
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Alternate = GPIO_AF9_XSPI2;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    
    // IO3: PD13 (AF9)
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Alternate = GPIO_AF9_XSPI2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    // IO4: PF12 (AF9)
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Alternate = GPIO_AF9_XSPI2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    
    // IO5: PD4 (AF9)
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Alternate = GPIO_AF9_XSPI2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    // IO6: PD5 (AF9)
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Alternate = GPIO_AF9_XSPI2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    // IO7: PD6 (AF9)
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF9_XSPI2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

int boot_xspi_init(void)
{
    // Enable XSPI2 clock
    __HAL_RCC_XSPI2_CLK_ENABLE();
    
    // Configure XSPI handle
    hxspi2.Instance = XSPI2;
    hxspi2.Init.ClockPrescaler = 2;                    // 600MHz / 3 = 200MHz
    hxspi2.Init.FifoThreshold = 4;
    hxspi2.Init.MemorySize = HAL_XSPI_SIZE_256MB;      // 256MB
    hxspi2.Init.ChipSelectHighTime = 5000;
    hxspi2.Init.MemoryType = HAL_XSPI_MEMTYPE_MACRONIX;
    
    // Initialize XSPI peripheral
    if (HAL_XSPI_Init(&hxspi2) != 0) {
        return -1;
    }
    
    // Initialize MX25UM flash (reset, ID check, enable Octal DTR, memory-mapped)
    if (HAL_XSPI_MX25UM_Init(&hxspi2) != 0) {
        return -1;
    }
    
    return 0;
}

void boot_jump_to_application(void)
{
    // Get stack pointer and reset handler from application's vector table
    uint32_t app_stack_pointer = *((volatile uint32_t *)APP_ADDRESS);
    uint32_t app_reset_handler = *((volatile uint32_t *)(APP_ADDRESS + 4));
    
    // Type for the reset handler function
    typedef void (*app_reset_t)(void);
    app_reset_t app_reset = (app_reset_t)app_reset_handler;
    
    // Disable interrupts
    __disable_irq();
    
    // Disable SysTick
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    
    // Set vector table offset to application
    SCB->VTOR = APP_ADDRESS;
    
    // Set stack pointer
    __set_MSP(app_stack_pointer);
    
    // Jump to application reset handler
    app_reset();
}

// Error handler for boot
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        // Could toggle LED here for debugging
    }
}
