#include "hal.h"
#include "hal_clock.h"
#include "stm32h7rs_clock.h"
#include "cmsis_device.h"

/* At reset the CPU runs on HSI. Each loop takes at least one cycle,
 * so HSI_VALUE / 10 loops take at least 100 ms. */
#define RCC_WAIT_LOOPS (HSI_VALUE / 10U)

extern uint32_t SystemCoreClock;

static int rcc_wait(volatile const uint32_t *reg, uint32_t mask, uint32_t expect)
{
    for (uint32_t i = 0; i < RCC_WAIT_LOOPS; i++) {
        if ((*reg & mask) == expect) {
            return 0;
        }
    }
    return -1;
}

/**
 * @brief Calculate PLL output frequency
 */
static uint32_t get_pll_output_freq(uint32_t pll_source, uint32_t pllm, uint32_t plln, uint32_t pllp)
{
    uint32_t pll_input_freq;

    // Get PLL input frequency based on source
    switch (pll_source) {
    case RCC_PLLSOURCE_HSI:
        pll_input_freq = HSI_VALUE;
        break;
    case RCC_PLLSOURCE_HSE:
        pll_input_freq = HSE_VALUE;
        break;
    case RCC_PLLSOURCE_CSI:
        pll_input_freq = CSI_VALUE;
        break;
    default:
        return 0;
    }

    // PLL formula: VCO = (Input / PLLM) * PLLN
    // Output = VCO / PLLP
    uint32_t vco_freq = (pll_input_freq / pllm) * plln;
    return vco_freq / pllp;
}

void clock_init(const struct clock_cfg* cfg)
{
    // Initialize oscillator with system source clock
    if (rcc_osc_config(cfg) != 0) {
        system_error_handle();
    }

    // Setup PLL (phase-locked loop) with system clock
    if (rcc_pll_config(cfg) != 0) {
        system_error_handle();
    }

    // Configure STM32 clock tree
    if (rcc_clock_config(cfg) != 0) {
        system_error_handle();
    }

    return;
}

uint32_t clock_get_sysclk(void)
{
    uint32_t sysclk_source;
    uint32_t pll_source, pllm, plln, pllp;

    // Check which source is being used for SYSCLK
    sysclk_source = (RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos;

    switch (sysclk_source) {
    case 0x00: // HSI
        return HSI_VALUE;

    case 0x01: // CSI
        return CSI_VALUE;

    case 0x02:            // HSE
        return HSE_VALUE; // Defined in system file

    case 0x03: // PLL
        // Get PLL configuration
        pll_source = RCC->PLLCKSELR & RCC_PLLCKSELR_PLLSRC;
        pllm       = (RCC->PLLCKSELR & RCC_PLLCKSELR_DIVM1) >> RCC_PLLCKSELR_DIVM1_Pos;
        plln       = ((RCC->PLL1DIVR1 & RCC_PLL1DIVR1_DIVN) >> RCC_PLL1DIVR1_DIVN_Pos) + 1U;
        pllp       = ((RCC->PLL1DIVR1 & RCC_PLL1DIVR1_DIVP) >> RCC_PLL1DIVR1_DIVP_Pos) + 1U;

        return get_pll_output_freq(pll_source, pllm, plln, pllp);

    default:
        return HSI_VALUE; // Default to HSI
    }
}

uint32_t clock_get_subsysclk(struct clock_subsys* clk)
{
    switch (clk->source)
    {
    case RCC_CLKSOURCE_HCLK:
        return rcc_get_hclk_freq();
    case RCC_CLKSOURCE_APB1:
        return rcc_get_pclk1_freq();
    case RCC_CLKSOURCE_APB2:
        return rcc_get_pclk2_freq();
    case RCC_CLKSOURCE_APB4:
        return rcc_get_pclk4_freq();
    case RCC_CLKSOURCE_APB5:
        return rcc_get_pclk5_freq();
    default:
        break;
    }

    return 0;
}

int rcc_osc_config(const struct clock_cfg *config)
{
    if (config == NULL) {
        return -1;
    }

    /*--- HSE Configuration ---*/
    if (config->hse_state != RCC_HSE_OFF) {
        // Enable HSE
        if (config->hse_state == RCC_HSE_BYPASS) {
            // External clock source (bypass crystal)
            SET_BIT(RCC->CR, RCC_CR_HSEBYP);
        } else {
            // Crystal oscillator
            CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);
        }

        // Turn on HSE
        SET_BIT(RCC->CR, RCC_CR_HSEON);
        if (rcc_wait(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY) != 0) {
            return -1;
        }
    } else {
        // Disable HSE
        CLEAR_BIT(RCC->CR, RCC_CR_HSEON);
        if (rcc_wait(&RCC->CR, RCC_CR_HSERDY, 0U) != 0) {
            return -1;
        }
    }

    /*--- HSI Configuration ---*/
    if (config->hsi_state != RCC_HSI_OFF) {
        // Enable HSI
        SET_BIT(RCC->CR, RCC_CR_HSION);
        if (rcc_wait(&RCC->CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY) != 0) {
            return -1;
        }

        // Adjust HSI calibration if specified
        if (config->calibration != 0) {
            MODIFY_REG(RCC->HSICFGR, RCC_HSICFGR_HSITRIM, config->calibration << RCC_HSICFGR_HSITRIM_Pos);
        }
    } else {
        /* HSI is the reset-default system clock, so refuse to gate it while
         * it is still feeding SYSCLK — that would stop the core dead. */
        if (((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos) == RCC_SYSCLKSOURCE_HSI) {
            return -1;
        }
        CLEAR_BIT(RCC->CR, RCC_CR_HSION);
        if (rcc_wait(&RCC->CR, RCC_CR_HSIRDY, 0U) != 0) {
            return -1;
        }
    }

    return 0;
}

int rcc_pll_config(const struct clock_cfg *config)
{
    if (config == NULL) {
        return -1;
    }

    if (config->pll_state == RCC_PLL_ON) {
        // Disable PLL first
        CLEAR_BIT(RCC->CR, RCC_CR_PLL1ON);
        if (rcc_wait(&RCC->CR, RCC_CR_PLL1RDY, 0U) != 0) {
            return -1;
        }

        // Configure PLL source — RCC_PLLSOURCE_*, not the SYSCLK selector
        MODIFY_REG(RCC->PLLCKSELR, RCC_PLLCKSELR_PLLSRC, config->pll_source);

        // Configure PLLM (input divider)
        MODIFY_REG(RCC->PLLCKSELR, RCC_PLLCKSELR_DIVM1, config->pllm << RCC_PLLCKSELR_DIVM1_Pos);

        // Configure PLLN (multiplier)
        MODIFY_REG(RCC->PLL1DIVR1, RCC_PLL1DIVR1_DIVN, (config->plln - 1U) << RCC_PLL1DIVR1_DIVN_Pos);

        // Configure PLLP (divider for system clock output)
        MODIFY_REG(RCC->PLL1DIVR1, RCC_PLL1DIVR1_DIVP, (config->pllp - 1U) << RCC_PLL1DIVR1_DIVP_Pos);

        // Configure PLLQ (divider for peripherals)
        MODIFY_REG(RCC->PLL1DIVR1, RCC_PLL1DIVR1_DIVQ, (config->pllq - 1U) << RCC_PLL1DIVR1_DIVQ_Pos);

        // Configure PLLR (divider)
        MODIFY_REG(RCC->PLL1DIVR1, RCC_PLL1DIVR1_DIVR, (config->pllr - 1U) << RCC_PLL1DIVR1_DIVR_Pos);

        // Enable PLL outputs
        SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1PEN | RCC_PLLCFGR_PLL1QEN | RCC_PLLCFGR_PLL1REN);

        // Enable PLL
        SET_BIT(RCC->CR, RCC_CR_PLL1ON);
        if (rcc_wait(&RCC->CR, RCC_CR_PLL1RDY, RCC_CR_PLL1RDY) != 0) {
            return -1;
        }
    } else {
        /* Refuse to stop PLL1 while SYSCLK is still running off it. */
        if (((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos) == RCC_SYSCLKSOURCE_PLLCLK) {
            return -1;
        }
        CLEAR_BIT(RCC->CR, RCC_CR_PLL1ON);
        if (rcc_wait(&RCC->CR, RCC_CR_PLL1RDY, 0U) != 0) {
            return -1;
        }
    }

    return 0;
}

int rcc_clock_config(const struct clock_cfg *config)
{
    if (config == NULL) {
        return -1;
    }

    // Configure flash latency before increasing frequency
    //TODO: this probably shouldn't be hardcoded?
    uint32_t flatency = FLASH_ACR_LATENCY_3;
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, flatency);

    // Check that flash latency was set correctly
    if (READ_BIT(FLASH->ACR, FLASH_ACR_LATENCY) != flatency) {
        return -1;
    }

    /*--- Configure AHB prescaler (HCLK) ---*/
    MODIFY_REG(RCC->BMCFGR, RCC_BMCFGR_BMPRE, config->subsys[RCC_CLKSOURCE_HCLK].divider);

    /*--- Configure APB1 prescaler ---*/
    MODIFY_REG(RCC->APBCFGR, RCC_APBCFGR_PPRE1, config->subsys[RCC_CLKSOURCE_APB1].divider);

    /*--- Configure APB2 prescaler ---*/
    MODIFY_REG(RCC->APBCFGR, RCC_APBCFGR_PPRE2, config->subsys[RCC_CLKSOURCE_APB2].divider);

    /*--- Configure APB4 prescaler ---*/
    MODIFY_REG(RCC->APBCFGR, RCC_APBCFGR_PPRE4, config->subsys[RCC_CLKSOURCE_APB4].divider);

    /*--- Configure APB5 prescaler ---*/
    MODIFY_REG(RCC->APBCFGR, RCC_APBCFGR_PPRE5, config->subsys[RCC_CLKSOURCE_APB5].divider);

    /*--- Configure system clock source ---*/
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, config->sysclk_source);

    // Wait until system clock source is switched
    while ((RCC->CFGR & RCC_CFGR_SWS) != ((uint32_t) config->sysclk_source << RCC_CFGR_SWS_Pos)) {
        // Wait for switch to complete
    }

    // Update SystemCoreClock variable
    SystemCoreClock = clock_get_sysclk();

    return 0;
}

/**
 * @brief Get AHB clock frequency (HCLK)
 */
uint32_t rcc_get_hclk_freq(void)
{
    uint32_t sysclk = clock_get_sysclk();
    uint32_t hpre   = (RCC->BMCFGR & RCC_BMCFGR_BMPRE) >> RCC_BMCFGR_BMPRE_Pos;

    // AHB prescaler table
    const uint8_t ahb_prescaler_table[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

    return sysclk >> ahb_prescaler_table[hpre];
}

/**
 * @brief Get APB1 clock frequency (PCLK1)
 */
uint32_t rcc_get_pclk1_freq(void)
{
    uint32_t hclk  = rcc_get_hclk_freq();
    uint32_t ppre1 = (RCC->APBCFGR & RCC_APBCFGR_PPRE1) >> RCC_APBCFGR_PPRE1_Pos;

    // APB prescaler table
    const uint8_t apb_prescaler_table[8] = {0, 0, 0, 0, 1, 2, 3, 4};

    return hclk >> apb_prescaler_table[ppre1];
}

/**
 * @brief Get APB2 clock frequency (PCLK2)
 */
uint32_t rcc_get_pclk2_freq(void)
{
    uint32_t hclk  = rcc_get_hclk_freq();
    uint32_t ppre2 = (RCC->APBCFGR & RCC_APBCFGR_PPRE2) >> RCC_APBCFGR_PPRE2_Pos;

    const uint8_t apb_prescaler_table[8] = {0, 0, 0, 0, 1, 2, 3, 4};

    return hclk >> apb_prescaler_table[ppre2];
}

/**
 * @brief Get APB4 clock frequency (PCLK4)
 */
uint32_t rcc_get_pclk4_freq(void)
{
    uint32_t hclk  = rcc_get_hclk_freq();
    uint32_t ppre4 = (RCC->APBCFGR & RCC_APBCFGR_PPRE4) >> RCC_APBCFGR_PPRE4_Pos;

    const uint8_t apb_prescaler_table[8] = {0, 0, 0, 0, 1, 2, 3, 4};

    return hclk >> apb_prescaler_table[ppre4];
}

/**
 * @brief Get APB5 clock frequency (PCLK5)
 */
uint32_t rcc_get_pclk5_freq(void)
{
    uint32_t hclk  = rcc_get_hclk_freq();
    uint32_t ppre5 = (RCC->APBCFGR & RCC_APBCFGR_PPRE5) >> RCC_APBCFGR_PPRE5_Pos;

    const uint8_t apb_prescaler_table[8] = {0, 0, 0, 0, 1, 2, 3, 4};

    return hclk >> apb_prescaler_table[ppre5];
}
