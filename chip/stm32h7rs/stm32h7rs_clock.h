#ifndef STM32H7RS_CLOCK_H
#define STM32H7RS_CLOCK_H

#include <stdint.h>

#include "hal_clock.h"
#include "stm32h7rsxx_ll_rcc.h"

/* HSI/HSE/CSI_VALUE come from stm32h7rsxx_ll_rcc.h (64/24/4 MHz). */

/* Only HSI is implemented. PLL1 additionally needs a legal VCO — >=400MHz on
 * this part, see UTILS_PLLVCO_WIDE_OUTPUT_MIN — plus a PWR voltage-scaling
 * step that LL does not do for you.
 * TODO: add CSI/HSE/PLL1 when something needs them. */
typedef enum {
    RCC_SYSCLKSOURCE_HSI = 0,
    RCC_SYSCLKSOURCE_CSI,
    RCC_SYSCLKSOURCE_HSE,
    RCC_SYSCLKSOURCE_PLLCLK
} clock_source;

typedef enum {
    RCC_CLKSOURCE_SYS = 0, /* CPUCLK: SYSCLK after the CPRE prescaler */
    RCC_CLKSOURCE_HCLK,
    RCC_CLKSOURCE_APB1,
    RCC_CLKSOURCE_APB2,
    RCC_CLKSOURCE_APB4,
    RCC_CLKSOURCE_APB5,
    RCC_CLKSOURCE_COUNT
} subsys_clock_source;

struct clock_subsys {
    subsys_clock_source source;
    uint32_t divider; /* LL_RCC_{SYSCLK,AHB,APB1,APB2,APB4,APB5}_DIV_* */
};

struct clock_cfg {
    clock_source sysclk_source;
    struct clock_subsys subsys[RCC_CLKSOURCE_COUNT];
};

#endif // STM32H7RS_CLOCK_H
