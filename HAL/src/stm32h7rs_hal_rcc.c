#include "stm32h7rs_hal_rcc.h"

// Timeout values (in milliseconds)
#define HSE_STARTUP_TIMEOUT     100U
#define HSI_STARTUP_TIMEOUT     100U
#define PLL_TIMEOUT             100U

// Global variable to store system clock frequency
extern uint32_t SystemCoreClock;

/*============================================================================
 * Private Helper Functions
 *===========================================================================*/

/**
 * @brief Get tick count (assumes HAL_GetTick() is implemented via SysTick)
 */
extern uint32_t HAL_GetTick(void);

/**
 * @brief Calculate PLL output frequency
 */
static uint32_t RCC_GetPLLOutputFreq(uint32_t pll_source, uint32_t pllm, uint32_t plln, uint32_t pllp) {
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

/*============================================================================
 * Public API Implementation
 *===========================================================================*/

/**
 * @brief Configure oscillators (HSI, HSE)
 * @param RCC_OscInitStruct: Pointer to oscillator configuration
 * @return HAL_OK if success, HAL_ERROR or HAL_TIMEOUT on failure
 */
HAL_Status HAL_RCC_OscConfig(RCC_OscInit *RCC_OscInitStruct) {
    uint32_t tickstart;
    
    if (RCC_OscInitStruct == NULL) {
        return HAL_ERROR;
    }
    
    /*--- HSE Configuration ---*/
    if (RCC_OscInitStruct->HSEState != RCC_HSE_OFF) {
        // Enable HSE
        if (RCC_OscInitStruct->HSEState == RCC_HSE_BYPASS) {
            // External clock source (bypass crystal)
            SET_BIT(RCC->CR, RCC_CR_HSEBYP);
        } else {
            // Crystal oscillator
            CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);
        }
        
        // Turn on HSE
        SET_BIT(RCC->CR, RCC_CR_HSEON);
        
        // Wait for HSE to be ready
        tickstart = HAL_GetTick();
        while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == 0) {
            if ((HAL_GetTick() - tickstart) > HSE_STARTUP_TIMEOUT) {
                return HAL_TIMEOUT;
            }
        }
    } else {
        // Disable HSE
        CLEAR_BIT(RCC->CR, RCC_CR_HSEON);
        
        // Wait for HSE to be disabled
        tickstart = HAL_GetTick();
        while (READ_BIT(RCC->CR, RCC_CR_HSERDY) != 0) {
            if ((HAL_GetTick() - tickstart) > HSE_STARTUP_TIMEOUT) {
                return HAL_TIMEOUT;
            }
        }
    }
    
    /*--- HSI Configuration ---*/
    if (RCC_OscInitStruct->HSIState != RCC_HSI_OFF) {
        // Enable HSI
        SET_BIT(RCC->CR, RCC_CR_HSION);
        
        // Wait for HSI to be ready
        tickstart = HAL_GetTick();
        while (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == 0) {
            if ((HAL_GetTick() - tickstart) > HSI_STARTUP_TIMEOUT) {
                return HAL_TIMEOUT;
            }
        }
        
        // Adjust HSI calibration if specified
        if (RCC_OscInitStruct->HSICalibration != 0) {
            MODIFY_REG(RCC->HSICFGR, RCC_HSICFGR_HSITRIM, 
                       RCC_OscInitStruct->HSICalibration << RCC_HSICFGR_HSITRIM_Pos);
        }
    } else {
        // Disable HSI (only if not used as system clock)
        CLEAR_BIT(RCC->CR, RCC_CR_HSION);
        
        // Wait for HSI to be disabled
        tickstart = HAL_GetTick();
        while (READ_BIT(RCC->CR, RCC_CR_HSIRDY) != 0) {
            if ((HAL_GetTick() - tickstart) > HSI_STARTUP_TIMEOUT) {
                return HAL_TIMEOUT;
            }
        }
    }
    
    return HAL_OK;
}

/**
 * @brief Configure main PLL
 * @param RCC_PLLInitStruct: Pointer to PLL configuration
 * @return HAL_OK if success, HAL_ERROR or HAL_TIMEOUT on failure
 */
HAL_Status HAL_RCC_PLLConfig(RCC_PLLInit *RCC_PLLInitStruct) {
    uint32_t tickstart;
    
    if (RCC_PLLInitStruct == NULL) {
        return HAL_ERROR;
    }
    
    if (RCC_PLLInitStruct->PLLState == RCC_PLL_ON) {
        // Disable PLL first
        CLEAR_BIT(RCC->CR, RCC_CR_PLL1ON);
        
        // Wait until PLL is disabled
        tickstart = HAL_GetTick();
        while (READ_BIT(RCC->CR, RCC_CR_PLL1RDY) != 0) {
            if ((HAL_GetTick() - tickstart) > PLL_TIMEOUT) {
                return HAL_TIMEOUT;
            }
        }
        
        // Configure PLL source
        MODIFY_REG(RCC->PLLCKSELR, RCC_PLLCKSELR_PLLSRC, RCC_PLLInitStruct->PLLSource);
        
        // Configure PLLM (input divider)
        MODIFY_REG(RCC->PLLCKSELR, RCC_PLLCKSELR_DIVM1, 
                   RCC_PLLInitStruct->PLLM << RCC_PLLCKSELR_DIVM1_Pos);
        
        // Configure PLLN (multiplier)
        MODIFY_REG(RCC->PLL1DIVR1, RCC_PLL1DIVR1_DIVN, 
                   (RCC_PLLInitStruct->PLLN - 1U) << RCC_PLL1DIVR1_DIVN_Pos);
        
        // Configure PLLP (divider for system clock output)
        MODIFY_REG(RCC->PLL1DIVR1, RCC_PLL1DIVR1_DIVP, 
                   (RCC_PLLInitStruct->PLLP - 1U) << RCC_PLL1DIVR1_DIVP_Pos);
        
        // Configure PLLQ (divider for peripherals)
        MODIFY_REG(RCC->PLL1DIVR1, RCC_PLL1DIVR1_DIVQ, 
                   (RCC_PLLInitStruct->PLLQ - 1U) << RCC_PLL1DIVR1_DIVQ_Pos);
        
        // Configure PLLR (divider)
        MODIFY_REG(RCC->PLL1DIVR1, RCC_PLL1DIVR1_DIVR, 
                   (RCC_PLLInitStruct->PLLR - 1U) << RCC_PLL1DIVR1_DIVR_Pos);
        
        // Enable PLL outputs
        SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1PEN | RCC_PLLCFGR_PLL1QEN | RCC_PLLCFGR_PLL1REN);
        
        // Enable PLL
        SET_BIT(RCC->CR, RCC_CR_PLL1ON);
        
        // Wait for PLL to be ready
        tickstart = HAL_GetTick();
        while (READ_BIT(RCC->CR, RCC_CR_PLL1RDY) == 0) {
            if ((HAL_GetTick() - tickstart) > PLL_TIMEOUT) {
                return HAL_TIMEOUT;
            }
        }
    } else {
        // Disable PLL
        CLEAR_BIT(RCC->CR, RCC_CR_PLL1ON);
        
        // Wait until PLL is disabled
        tickstart = HAL_GetTick();
        while (READ_BIT(RCC->CR, RCC_CR_PLL1RDY) != 0) {
            if ((HAL_GetTick() - tickstart) > PLL_TIMEOUT) {
                return HAL_TIMEOUT;
            }
        }
    }
    
    return HAL_OK;
}

/**
 * @brief Configure system clocks and prescalers
 * @param RCC_ClkInitStruct: Pointer to clock configuration
 * @param FLatency: Flash latency (wait states) for new frequency
 * @return HAL_OK if success, HAL_ERROR on failure
 */
HAL_Status HAL_RCC_ClockConfig(RCC_ClkInit *RCC_ClkInitStruct, uint32_t FLatency) {
    if (RCC_ClkInitStruct == NULL) {
        return HAL_ERROR;
    }
    
    // Configure flash latency before increasing frequency
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLatency);

    // Check that flash latency was set correctly
    if (READ_BIT(FLASH->ACR, FLASH_ACR_LATENCY) != FLatency) {
        return HAL_ERROR;
    }
    
    /*--- Configure AHB prescaler (HCLK) ---*/
    MODIFY_REG(RCC->BMCFGR, RCC_BMCFGR_BMPRE, RCC_ClkInitStruct->AHBCLKDivider);

    /*--- Configure APB1 prescaler ---*/
    MODIFY_REG(RCC->APBCFGR, RCC_APBCFGR_PPRE1, RCC_ClkInitStruct->APB1CLKDivider);

    /*--- Configure APB2 prescaler ---*/
    MODIFY_REG(RCC->APBCFGR, RCC_APBCFGR_PPRE2, RCC_ClkInitStruct->APB2CLKDivider);

    /*--- Configure APB4 prescaler ---*/
    MODIFY_REG(RCC->APBCFGR, RCC_APBCFGR_PPRE4, RCC_ClkInitStruct->APB4CLKDivider);

    /*--- Configure APB5 prescaler ---*/
    MODIFY_REG(RCC->APBCFGR, RCC_APBCFGR_PPRE5, RCC_ClkInitStruct->APB5CLKDivider);

    /*--- Configure system clock source ---*/
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_ClkInitStruct->SYSCLKSource);

    // Wait until system clock source is switched
    while ((RCC->CFGR & RCC_CFGR_SWS) != (RCC_ClkInitStruct->SYSCLKSource << RCC_CFGR_SWS_Pos)) {
        // Wait for switch to complete
    }

    // Update SystemCoreClock variable
    SystemCoreClock = HAL_RCC_GetSysClockFreq();
    
    return HAL_OK;
}

/**
 * @brief Get current system clock frequency
 * @return System clock frequency in Hz
 */
uint32_t HAL_RCC_GetSysClockFreq(void) {
    uint32_t sysclk_source;
    uint32_t pll_source, pllm, plln, pllp;
    
    // Check which source is being used for SYSCLK
    sysclk_source = (RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos;
    
    switch (sysclk_source) {
        case 0x00:  // HSI
            return HSI_VALUE;
            
        case 0x01:  // CSI
            return CSI_VALUE;
            
        case 0x02:  // HSE
            return HSE_VALUE;  // Defined in system file
            
        case 0x03:  // PLL
            // Get PLL configuration
            pll_source = RCC->PLLCKSELR & RCC_PLLCKSELR_PLLSRC;
            pllm = (RCC->PLLCKSELR & RCC_PLLCKSELR_DIVM1) >> RCC_PLLCKSELR_DIVM1_Pos;
            plln = ((RCC->PLL1DIVR1 & RCC_PLL1DIVR1_DIVN) >> RCC_PLL1DIVR1_DIVN_Pos) + 1U;
            pllp = ((RCC->PLL1DIVR1 & RCC_PLL1DIVR1_DIVP) >> RCC_PLL1DIVR1_DIVP_Pos) + 1U;
            
            return RCC_GetPLLOutputFreq(pll_source, pllm, plln, pllp);
            
        default:
            return HSI_VALUE;  // Default to HSI
    }
}

/**
 * @brief Get AHB clock frequency (HCLK)
 */
uint32_t HAL_RCC_GetHCLKFreq(void) {
    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    uint32_t hpre = (RCC->BMCFGR & RCC_BMCFGR_BMPRE) >> RCC_BMCFGR_BMPRE_Pos;
    
    // AHB prescaler table
    const uint8_t ahb_prescaler_table[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
    
    return sysclk >> ahb_prescaler_table[hpre];
}

/**
 * @brief Get APB1 clock frequency (PCLK1)
 */
uint32_t HAL_RCC_GetPCLK1Freq(void) {
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    uint32_t ppre1 = (RCC->APBCFGR & RCC_APBCFGR_PPRE1) >> RCC_APBCFGR_PPRE1_Pos;
    
    // APB prescaler table
    const uint8_t apb_prescaler_table[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    
    return hclk >> apb_prescaler_table[ppre1];
}

/**
 * @brief Get APB2 clock frequency (PCLK2)
 */
uint32_t HAL_RCC_GetPCLK2Freq(void) {
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    uint32_t ppre2 = (RCC->APBCFGR & RCC_APBCFGR_PPRE2) >> RCC_APBCFGR_PPRE2_Pos;
    
    const uint8_t apb_prescaler_table[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    
    return hclk >> apb_prescaler_table[ppre2];
}

/**
 * @brief Get APB4 clock frequency (PCLK4)
 */
uint32_t HAL_RCC_GetPCLK4Freq(void) {
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    uint32_t ppre4 = (RCC->APBCFGR & RCC_APBCFGR_PPRE4) >> RCC_APBCFGR_PPRE4_Pos;
    
    const uint8_t apb_prescaler_table[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    
    return hclk >> apb_prescaler_table[ppre4];
}

/**
 * @brief Get APB5 clock frequency (PCLK5)
 */
uint32_t HAL_RCC_GetPCLK5Freq(void) {
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    uint32_t ppre5 = (RCC->APBCFGR & RCC_APBCFGR_PPRE5) >> RCC_APBCFGR_PPRE5_Pos;
    
    const uint8_t apb_prescaler_table[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    
    return hclk >> apb_prescaler_table[ppre5];
}
