#ifndef STM32H7RS_HAL_RCC_H
#define STM32H7RS_HAL_RCC_H

#include "stm32h7rs_hal.h"
#include "stm32h7rsxx.h"

/*============================================================================
 * Clock Source Definitions
 *===========================================================================*/

#define RCC_OSCILLATOR_TYPE_HSI 0
#define RCC_OSCILLATOR_TYPE_HSE 1
#define RCC_OSCILLATOR_TYPE_CSI 2
#define RCC_OSCILLATOR_TYPE_PLL 3

// HSI (High-Speed Internal) - 8, 16, 32, 64 MHz RC oscillator
#define RCC_HSI_OFF 0
#define RCC_HSI_ON  1
#define HSI_VALUE   64000000U

// HSE (High-Speed External) - 4-50 MHz Crystal/oscillator
#define RCC_HSE_OFF    0
#define RCC_HSE_ON     1
#define RCC_HSE_BYPASS 2 // External clock source
#define HSE_VALUE      24000000U

// CSI (Low-Power Internal) - ~4 MHz
#define RCC_CSI_OFF 0
#define RCC_CSI_ON  1
#define CSI_VALUE   4000000U

// PLL
#define RCC_PLL_OFF 0
#define RCC_PLL_ON  1

#define RCC_PLLSOURCE_HSI 0U                     /*!< HSI clock selected as PLL entry clock source */
#define RCC_PLLSOURCE_CSI RCC_PLLCKSELR_PLLSRC_0 /*!< CSI clock selected as PLL entry clock source */
#define RCC_PLLSOURCE_HSE RCC_PLLCKSELR_PLLSRC_1 /*!< HSE clock selected as PLL entry clock source */
#define RCC_PLLSOURCE_NONE                                                                                                                           \
    RCC_PLLCKSELR_PLLSRC /*!< No clock selected as PLL entry clock source                                                                            \
                          */

// System clock source
#define RCC_SYSCLKSOURCE_HSI    0x00000000U
#define RCC_SYSCLKSOURCE_CSI    0x00000001U
#define RCC_SYSCLKSOURCE_HSE    0x00000002U
#define RCC_SYSCLKSOURCE_PLLCLK 0x00000003U

/*============================================================================
 * Clock Prescaler Definitions
 *===========================================================================*/

// AHB prescale (divides SYSCLK to get HCLK)
#define RCC_HCLK_DIV1   (0x00000000U)
#define RCC_HCLK_DIV2   (RCC_BMCFGR_BMPRE_3)
#define RCC_HCLK_DIV4   (RCC_BMCFGR_BMPRE_3 | RCC_BMCFGR_BMPRE_0)
#define RCC_HCLK_DIV8   (RCC_BMCFGR_BMPRE_3 | RCC_BMCFGR_BMPRE_1)
#define RCC_HCLK_DIV16  (RCC_BMCFGR_BMPRE_3 | RCC_BMCFGR_BMPRE_1 | RCC_BMCFGR_BMPRE_0)
#define RCC_HCLK_DIV64  (RCC_BMCFGR_BMPRE_3 | RCC_BMCFGR_BMPRE_2)
#define RCC_HCLK_DIV128 (RCC_BMCFGR_BMPRE_3 | RCC_BMCFGR_BMPRE_2 | RCC_BMCFGR_BMPRE_0)
#define RCC_HCLK_DIV256 (RCC_BMCFGR_BMPRE_3 | RCC_BMCFGR_BMPRE_2 | RCC_BMCFGR_BMPRE_1)
#define RCC_HCLK_DIV512 (RCC_BMCFGR_BMPRE_3 | RCC_BMCFGR_BMPRE_2 | RCC_BMCFGR_BMPRE_1 | RCC_BMCFGR_BMPRE_0)

// APB1 prescaler (PPRE1 bits in APBCFGR)
#define RCC_APB1_DIV1  (0x00000000U)
#define RCC_APB1_DIV2  (RCC_APBCFGR_PPRE1_2)
#define RCC_APB1_DIV4  (RCC_APBCFGR_PPRE1_2 | RCC_APBCFGR_PPRE1_0)
#define RCC_APB1_DIV8  (RCC_APBCFGR_PPRE1_2 | RCC_APBCFGR_PPRE1_1)
#define RCC_APB1_DIV16 (RCC_APBCFGR_PPRE1_2 | RCC_APBCFGR_PPRE1_1 | RCC_APBCFGR_PPRE1_0)

// APB2 prescaler (PPRE2 bits in APBCFGR)
#define RCC_APB2_DIV1  (0x00000000U)
#define RCC_APB2_DIV2  (RCC_APBCFGR_PPRE2_2)
#define RCC_APB2_DIV4  (RCC_APBCFGR_PPRE2_2 | RCC_APBCFGR_PPRE2_0)
#define RCC_APB2_DIV8  (RCC_APBCFGR_PPRE2_2 | RCC_APBCFGR_PPRE2_1)
#define RCC_APB2_DIV16 (RCC_APBCFGR_PPRE2_2 | RCC_APBCFGR_PPRE2_1 | RCC_APBCFGR_PPRE2_0)

// APB4 prescaler (PPRE4 bits in APBCFGR)
#define RCC_APB4_DIV1  (0x00000000U)
#define RCC_APB4_DIV2  (RCC_APBCFGR_PPRE4_2)
#define RCC_APB4_DIV4  (RCC_APBCFGR_PPRE4_2 | RCC_APBCFGR_PPRE4_0)
#define RCC_APB4_DIV8  (RCC_APBCFGR_PPRE4_2 | RCC_APBCFGR_PPRE4_1)
#define RCC_APB4_DIV16 (RCC_APBCFGR_PPRE4_2 | RCC_APBCFGR_PPRE4_1 | RCC_APBCFGR_PPRE4_0)

// APB5 prescaler (PPRE5 bits in APBCFGR)
#define RCC_APB5_DIV1  (0x00000000U)
#define RCC_APB5_DIV2  (RCC_APBCFGR_PPRE5_2)
#define RCC_APB5_DIV4  (RCC_APBCFGR_PPRE5_2 | RCC_APBCFGR_PPRE5_0)
#define RCC_APB5_DIV8  (RCC_APBCFGR_PPRE5_2 | RCC_APBCFGR_PPRE5_1)
#define RCC_APB5_DIV16 (RCC_APBCFGR_PPRE5_2 | RCC_APBCFGR_PPRE5_1 | RCC_APBCFGR_PPRE5_0)

/*============================================================================
 * Oscillator Configuration Structure
 *===========================================================================*/

typedef struct {
    uint32_t OscillatorType; // HSI, HSE, LSI, LSE
    uint32_t HSEState;       // ON, OFF, BYPASS
    uint32_t HSIState;       // ON, OFF
    uint32_t HSICalibration; // Calibration value 0-63 (default 32)
} RCC_OscInit;

/*============================================================================
 * PLL Configuration Structure
 *===========================================================================*/

typedef struct {
    uint32_t PLLState;  // ON or OFF
    uint32_t PLLSource; // HSI, HSE, CSI
    uint32_t PLLM;      // Input divider: 1-63
    uint32_t PLLN;      // Multiplier: 4-512
    uint32_t PLLP;      // Output divider for PLLP (system clock)
    uint32_t PLLQ;      // Output divider for PLLQ (USB, etc)
    uint32_t PLLR;      // Output divider for PLLR
    uint32_t PLLFRACN;  // Fractional part
} RCC_PLLInit;

/*============================================================================
 * Clock Configuration Structure
 *===========================================================================*/

typedef struct {
    uint32_t SYSCLKSource;   // HSI, HSE, or PLL
    uint32_t AHBCLKDivider;  // HCLK divider
    uint32_t APB1CLKDivider; // APB1 divider
    uint32_t APB2CLKDivider; // APB2 divider
    uint32_t APB4CLKDivider; // APB4 divider
    uint32_t APB5CLKDivider; // APB5 divider
} RCC_ClkInit;

/*============================================================================
 * API Functions
 *===========================================================================*/

// Oscillator configuration
HAL_Status HAL_RCC_OscConfig(RCC_OscInit *RCC_OscInitStruct);

// PLL configuration
HAL_Status HAL_RCC_PLLConfig(RCC_PLLInit *RCC_PLLInitStruct);

// Clock tree configuration
HAL_Status HAL_RCC_ClockConfig(RCC_ClkInit *RCC_ClkInitStruct, uint32_t FLatency);

// Get clock frequencies
uint32_t HAL_RCC_GetSysClockFreq(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);
uint32_t HAL_RCC_GetPCLK3Freq(void);

/*============================================================================
 * Peripheral Clock Enable/Disable Macros
 *===========================================================================*/

// GPIO clocks (AHB4 bus)
#define __HAL_RCC_GPIOA_CLK_ENABLE() (RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN)
#define __HAL_RCC_GPIOB_CLK_ENABLE() (RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN)
#define __HAL_RCC_GPIOC_CLK_ENABLE() (RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN)
#define __HAL_RCC_GPIOD_CLK_ENABLE() (RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN)
#define __HAL_RCC_GPIOE_CLK_ENABLE() (RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN)
#define __HAL_RCC_GPIOF_CLK_ENABLE() (RCC->AHB4ENR |= RCC_AHB4ENR_GPIOFEN)
#define __HAL_RCC_GPIOG_CLK_ENABLE() (RCC->AHB4ENR |= RCC_AHB4ENR_GPIOGEN)
#define __HAL_RCC_GPIOH_CLK_ENABLE() (RCC->AHB4ENR |= RCC_AHB4ENR_GPIOHEN)

#define __HAL_RCC_GPIOA_CLK_DISABLE() (RCC->AHB4ENR &= ~RCC_AHB4ENR_GPIOAEN)
#define __HAL_RCC_GPIOB_CLK_DISABLE() (RCC->AHB4ENR &= ~RCC_AHB4ENR_GPIOBEN)
// ... etc for other GPIO ports

// UART clocks (APB1/APB2 bus)
#define __HAL_RCC_USART1_CLK_ENABLE() (RCC->APB2ENR |= RCC_APB2ENR_USART1EN)
#define __HAL_RCC_USART2_CLK_ENABLE() (RCC->APB1LENR |= RCC_APB1LENR_USART2EN)
#define __HAL_RCC_USART3_CLK_ENABLE() (RCC->APB1LENR |= RCC_APB1LENR_USART3EN)

#define __HAL_RCC_USART1_CLK_DISABLE() (RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN)
#define __HAL_RCC_USART2_CLK_DISABLE() (RCC->APB1LENR &= ~RCC_APB1LENR_USART2EN)
#define __HAL_RCC_USART3_CLK_DISABLE() (RCC->APB1LENR &= ~RCC_APB1LENR_USART3EN)

// Timer clocks (APB1/APB2 bus)
#define __HAL_RCC_TIM1_CLK_ENABLE() (RCC->APB2ENR |= RCC_APB2ENR_TIM1EN)
#define __HAL_RCC_TIM2_CLK_ENABLE() (RCC->APB1LENR |= RCC_APB1LENR_TIM2EN)
#define __HAL_RCC_TIM3_CLK_ENABLE() (RCC->APB1LENR |= RCC_APB1LENR_TIM3EN)
#define __HAL_RCC_TIM4_CLK_ENABLE() (RCC->APB1LENR |= RCC_APB1LENR_TIM4EN)
#define __HAL_RCC_TIM5_CLK_ENABLE() (RCC->APB1LENR |= RCC_APB1LENR_TIM5EN)

// DMA clocks (AHB1 bus)
#define __HAL_RCC_DMA1_CLK_ENABLE() (RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN)
#define __HAL_RCC_DMA2_CLK_ENABLE() (RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN)

// SPI clocks
#define __HAL_RCC_SPI1_CLK_ENABLE() (RCC->APB2ENR |= RCC_APB2ENR_SPI1EN)
#define __HAL_RCC_SPI2_CLK_ENABLE() (RCC->APB1LENR |= RCC_APB1LENR_SPI2EN)
#define __HAL_RCC_SPI3_CLK_ENABLE() (RCC->APB1LENR |= RCC_APB1LENR_SPI3EN)

// I2C clocks
#define __HAL_RCC_I2C1_CLK_ENABLE() (RCC->APB1LENR |= RCC_APB1LENR_I2C1EN)
#define __HAL_RCC_I2C2_CLK_ENABLE() (RCC->APB1LENR |= RCC_APB1LENR_I2C2EN)
#define __HAL_RCC_I2C3_CLK_ENABLE() (RCC->APB1LENR |= RCC_APB1LENR_I2C3EN)

#endif // STM32H7RS_HAL_RCC_H
