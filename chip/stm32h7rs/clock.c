#include "hal.h"
#include "hal_clock.h"
#include "stm32h7rs_clock.h"

#include "stm32h7rsxx_ll_rcc.h"
#include "stm32h7rsxx_ll_system.h"
#include "stm32h7rsxx_ll_utils.h"

static uint32_t clock_hclk_freq(void)
{
    LL_RCC_ClocksTypeDef clocks;
    LL_RCC_GetSystemClocksFreq(&clocks);
    return clocks.HCLK_Frequency;
}

void clock_init(const struct clock_cfg *cfg)
{
    if (cfg == NULL || cfg->sysclk_source != RCC_SYSCLKSOURCE_HSI) {
        system_error_handle(); /* see the TODO in stm32h7rs_clock.h */
    }

    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() != 1U) {
    }

    /* A prescaler change can raise HCLK, and flash with too few wait states
     * faults. Park at the maximum, then drop to what HCLK really needs. */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_7);

    LL_RCC_SetSysPrescaler(cfg->subsys[RCC_CLKSOURCE_SYS].divider);
    LL_RCC_SetAHBPrescaler(cfg->subsys[RCC_CLKSOURCE_HCLK].divider);
    LL_RCC_SetAPB1Prescaler(cfg->subsys[RCC_CLKSOURCE_APB1].divider);
    LL_RCC_SetAPB2Prescaler(cfg->subsys[RCC_CLKSOURCE_APB2].divider);
    LL_RCC_SetAPB4Prescaler(cfg->subsys[RCC_CLKSOURCE_APB4].divider);
    LL_RCC_SetAPB5Prescaler(cfg->subsys[RCC_CLKSOURCE_APB5].divider);

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
    }

    if (LL_SetFlashLatency(clock_hclk_freq()) != SUCCESS) {
        system_error_handle();
    }

    LL_SetSystemCoreClock(clock_get_sysclk());
}

/* SysTick runs off CPUCLK (sys_cpu_ck), which is SYSCLK after the CPRE
 * prescaler — not SYSCLK itself. kernel.c feeds this to arch_systick_start. */
uint32_t clock_get_sysclk(void)
{
    LL_RCC_ClocksTypeDef clocks;
    LL_RCC_GetSystemClocksFreq(&clocks);
    return clocks.CPUCLK_Frequency;
}

uint32_t clock_get_subsysclk(struct clock_subsys *clk)
{
    LL_RCC_ClocksTypeDef clocks;

    if (clk == NULL) {
        return 0;
    }

    LL_RCC_GetSystemClocksFreq(&clocks);

    switch (clk->source) {
    case RCC_CLKSOURCE_SYS:
        return clocks.CPUCLK_Frequency;
    case RCC_CLKSOURCE_HCLK:
        return clocks.HCLK_Frequency;
    case RCC_CLKSOURCE_APB1:
        return clocks.PCLK1_Frequency;
    case RCC_CLKSOURCE_APB2:
        return clocks.PCLK2_Frequency;
    case RCC_CLKSOURCE_APB4:
        return clocks.PCLK4_Frequency;
    case RCC_CLKSOURCE_APB5:
        return clocks.PCLK5_Frequency;
    default:
        return 0;
    }
}
