#include <stdint.h>

#define SRAM_START (0x20000000U)
#define SRAM_SIZE (32U * 1024U)
#define SRAM_END (SRAM_START + SRAM_SIZE)
#define STACK_POINTER_INIT_ADDRESS (SRAM_END)
#define ISR_VECTOR_SIZE_WORDS 172

void Reset_Handler(void);

// Default handler
void Default_Handler(void) {
    while(1);
}

// Macro for weak handlers
#define VECTOR_HANDLER(name) \
    __attribute__((weak)) void name(void) { \
        Default_Handler(); \
    }

// Cortex-M system exceptions
VECTOR_HANDLER(NMI_Handler)
VECTOR_HANDLER(HardFault_Handler)
VECTOR_HANDLER(MemManage_Handler)
VECTOR_HANDLER(BusFault_Handler)
VECTOR_HANDLER(UsageFault_Handler)
VECTOR_HANDLER(SVCall_Handler)
VECTOR_HANDLER(DebugMonitor_Handler)
VECTOR_HANDLER(PendSV_Handler)
VECTOR_HANDLER(SysTick_Handler)

// STM32H7S3L8HX interrupt handlers
VECTOR_HANDLER(PVD_AVD_IRHandler)
VECTOR_HANDLER(DTS_IRHandler)
VECTOR_HANDLER(IWDG_IRHandler)
VECTOR_HANDLER(WWDG_IRHandler)
VECTOR_HANDLER(RCC_IRHandler)
VECTOR_HANDLER(FLASH_IRHandler)
VECTOR_HANDLER(ECC_FPU_IRHandler)
VECTOR_HANDLER(FPU_IRHandler)
VECTOR_HANDLER(TAMP_IRHandler)
VECTOR_HANDLER(EXTI0_IRHandler)
VECTOR_HANDLER(EXTI1_IRHandler)
VECTOR_HANDLER(EXTI2_IRHandler)
VECTOR_HANDLER(EXTI3_IRHandler)
VECTOR_HANDLER(EXTI4_IRHandler)
VECTOR_HANDLER(EXTI5_IRHandler)
VECTOR_HANDLER(EXTI6_IRHandler)
VECTOR_HANDLER(EXTI7_IRHandler)
VECTOR_HANDLER(EXTI8_IRHandler)
VECTOR_HANDLER(EXTI9_IRHandler)
VECTOR_HANDLER(EXTI10_IRHandler)
VECTOR_HANDLER(EXTI11_IRHandler)
VECTOR_HANDLER(EXTI12_IRHandler)
VECTOR_HANDLER(EXTI13_IRHandler)
VECTOR_HANDLER(EXTI14_IRHandler)
VECTOR_HANDLER(EXTI15_IRHandler)
VECTOR_HANDLER(RTC_IRHandler)
VECTOR_HANDLER(SAES_IRHandler)
VECTOR_HANDLER(AES_IRHandler)
VECTOR_HANDLER(PKA_IRHandler)
VECTOR_HANDLER(HASH_IRHandler)
VECTOR_HANDLER(RNG_IRHandler)
VECTOR_HANDLER(ADC1_2_IRHandler)
VECTOR_HANDLER(GPDMA1_CH0_IRHandler)
VECTOR_HANDLER(GPDMA1_CH1_IRHandler)
VECTOR_HANDLER(GPDMA1_CH2_IRHandler)
VECTOR_HANDLER(GPDMA1_CH3_IRHandler)
VECTOR_HANDLER(GPDMA1_CH4_IRHandler)
VECTOR_HANDLER(GPDMA1_CH5_IRHandler)
VECTOR_HANDLER(GPDMA1_CH6_IRHandler)
VECTOR_HANDLER(GPDMA1_CH7_IRHandler)
VECTOR_HANDLER(TIM1_BRK_IRHandler)
VECTOR_HANDLER(TIM1_UP_IRHandler)
VECTOR_HANDLER(TIM1_TRG_COM_IRHandler)
VECTOR_HANDLER(TIM1_CC_IRHandler)
VECTOR_HANDLER(TIM2_IRHandler)
VECTOR_HANDLER(TIM3_IRHandler)
VECTOR_HANDLER(TIM4_IRHandler)
VECTOR_HANDLER(TIM5_IRHandler)
VECTOR_HANDLER(TIM6_IRHandler)
VECTOR_HANDLER(TIM7_IRHandler)
VECTOR_HANDLER(TIM9_IRHandler)
VECTOR_HANDLER(SPI1_IRHandler)
VECTOR_HANDLER(SPI2_IRHandler)
VECTOR_HANDLER(SPI3_IRHandler)
VECTOR_HANDLER(SPI4_IRHandler)
VECTOR_HANDLER(SPI5_IRHandler)
VECTOR_HANDLER(SPI6_IRHandler)
VECTOR_HANDLER(HPDMA1_CH0_IRHandler)
VECTOR_HANDLER(HPDMA1_CH1_IRHandler)
VECTOR_HANDLER(HPDMA1_CH2_IRHandler)
VECTOR_HANDLER(HPDMA1_CH3_IRHandler)
VECTOR_HANDLER(HPDMA1_CH4_IRHandler)
VECTOR_HANDLER(HPDMA1_CH5_IRHandler)
VECTOR_HANDLER(HPDMA1_CH6_IRHandler)
VECTOR_HANDLER(HPDMA1_CH7_IRHandler)
VECTOR_HANDLER(SAI1_A_IRHandler)
VECTOR_HANDLER(SAI1_B_IRHandler)
VECTOR_HANDLER(SAI2_A_IRHandler)
VECTOR_HANDLER(SAI2_B_IRHandler)
VECTOR_HANDLER(I2C1_EV_IRHandler)
VECTOR_HANDLER(I2C1_ER_IRHandler)
VECTOR_HANDLER(I2C2_EV_IRHandler)
VECTOR_HANDLER(I2C2_ER_IRHandler)
VECTOR_HANDLER(I2C3_EV_IRHandler)
VECTOR_HANDLER(I2C3_ER_IRHandler)
VECTOR_HANDLER(USART1_IRHandler)
VECTOR_HANDLER(USART2_IRHandler)
VECTOR_HANDLER(USART3_IRHandler)
VECTOR_HANDLER(UART4_IRHandler)
VECTOR_HANDLER(UART5_IRHandler)
VECTOR_HANDLER(UART6_IRHandler)
VECTOR_HANDLER(UART7_IRHandler)
VECTOR_HANDLER(UART8_IRHandler)
VECTOR_HANDLER(I3C1_EV_IRHandler)
VECTOR_HANDLER(I3C1_ER_IRHandler)
VECTOR_HANDLER(OTG_HS_IRHandler)
VECTOR_HANDLER(ETH_IRHandler)
VECTOR_HANDLER(CORDIC_IRHandler)
VECTOR_HANDLER(GFXTIM_IRHandler)
VECTOR_HANDLER(DCMIPP_IRHandler)
VECTOR_HANDLER(LTDC_IRHandler)
VECTOR_HANDLER(LTDC_ER_IRHandler)
VECTOR_HANDLER(DMA2D_IRHandler)
VECTOR_HANDLER(JPEG_IRHandler)
VECTOR_HANDLER(GFXMMU_IRHandler)
VECTOR_HANDLER(I3C1_WKUP_IRHandler)
VECTOR_HANDLER(MCE1_IRHandler)
VECTOR_HANDLER(MCE2_IRHandler)
VECTOR_HANDLER(MCE3_IRHandler)
VECTOR_HANDLER(OSPI1_IRHandler)
VECTOR_HANDLER(OSPI2_IRHandler)
VECTOR_HANDLER(FMC_IRHandler)
VECTOR_HANDLER(SDMMC1_IRHandler)
VECTOR_HANDLER(SDMMC2_IRHandler)
VECTOR_HANDLER(OTG_FS_IRHandler)
VECTOR_HANDLER(TIM12_IRHandler)
VECTOR_HANDLER(TIM13_IRHandler)
VECTOR_HANDLER(TIM14_IRHandler)
VECTOR_HANDLER(TIM15_IRHandler)
VECTOR_HANDLER(TIM16_IRHandler)
VECTOR_HANDLER(TIM17_IRHandler)
VECTOR_HANDLER(LPTIM1_IRHandler)
VECTOR_HANDLER(LPTIM2_IRHandler)
VECTOR_HANDLER(LPTIM3_IRHandler)
VECTOR_HANDLER(LPTIM4_IRHandler)
VECTOR_HANDLER(LPTIM5_IRHandler)
VECTOR_HANDLER(SPDIF_RX_IRHandler)
VECTOR_HANDLER(MDIOS_IRHandler)
VECTOR_HANDLER(ADF1_FLT0_IRHandler)
VECTOR_HANDLER(CRS_IRHandler)
VECTOR_HANDLER(UCPD1_IRHandler)
VECTOR_HANDLER(CEC_IRHandler)
VECTOR_HANDLER(PSSI_IRHandler)
VECTOR_HANDLER(LPUART1_IRHandler)
VECTOR_HANDLER(WAKEUP_PIN_IRHandler)
VECTOR_HANDLER(GPDMA1_CH8_IRHandler)
VECTOR_HANDLER(GPDMA1_CH9_IRHandler)
VECTOR_HANDLER(GPDMA1_CH10_IRHandler)
VECTOR_HANDLER(GPDMA1_CH11_IRHandler)
VECTOR_HANDLER(GPDMA1_CH12_IRHandler)
VECTOR_HANDLER(GPDMA1_CH13_IRHandler)
VECTOR_HANDLER(GPDMA1_CH14_IRHandler)
VECTOR_HANDLER(GPDMA1_CH15_IRHandler)
VECTOR_HANDLER(HPDMA1_CH8_IRHandler)
VECTOR_HANDLER(HPDMA1_CH9_IRHandler)
VECTOR_HANDLER(HPDMA1_CH10_IRHandler)
VECTOR_HANDLER(HPDMA1_CH11_IRHandler)
VECTOR_HANDLER(HPDMA1_CH12_IRHandler)
VECTOR_HANDLER(HPDMA1_CH13_IRHandler)
VECTOR_HANDLER(HPDMA1_CH14_IRHandler)
VECTOR_HANDLER(HPDMA1_CH15_IRHandler)
VECTOR_HANDLER(GPU2D_IRHandler)
VECTOR_HANDLER(GPU2D_ER_IRHandler)
VECTOR_HANDLER(TCACHE_IRHandler)
VECTOR_HANDLER(FDCAN1_IT0_IRHandler)
VECTOR_HANDLER(FDCAN1_IT1_IRHandler)
VECTOR_HANDLER(FDCAN2_IT0_IRHandler)
VECTOR_HANDLER(FDCAN2_IT1_IRHandler)

uint32_t g_pfnVectors[ISR_VECTOR_SIZE_WORDS] __attribute__((section(".isr_vector"))) = {
  STACK_POINTER_INIT_ADDRESS,
  // Cortex-M system exceptions
  (uint32_t)&Reset_Handler,
  (uint32_t)&NMI_Handler,
  (uint32_t)&HardFault_Handler,
  (uint32_t)&MemManage_Handler,
  (uint32_t)&BusFault_Handler,
  (uint32_t)&UsageFault_Handler,
  0,
  0,
  0,
  0,
  (uint32_t)&SVCall_Handler,
  (uint32_t)&DebugMonitor_Handler,
  0,
  (uint32_t)&PendSV_Handler,
  (uint32_t)&SysTick_Handler,

  // STM32H7S3L8HX interrupt handlers
  (uint32_t)&PVD_AVD_IRHandler,
  0,
  (uint32_t)&DTS_IRHandler,
  (uint32_t)&IWDG_IRHandler,
  (uint32_t)&WWDG_IRHandler,
  (uint32_t)&RCC_IRHandler,
  0,
  0,
  (uint32_t)&FLASH_IRHandler,
  (uint32_t)&ECC_FPU_IRHandler,
  (uint32_t)&FPU_IRHandler,
  0,
  0,
  (uint32_t)&TAMP_IRHandler,
  0,
  0,
  (uint32_t)&EXTI0_IRHandler,
  (uint32_t)&EXTI1_IRHandler,
  (uint32_t)&EXTI2_IRHandler,
  (uint32_t)&EXTI3_IRHandler,
  (uint32_t)&EXTI4_IRHandler,
  (uint32_t)&EXTI5_IRHandler,
  (uint32_t)&EXTI6_IRHandler,
  (uint32_t)&EXTI7_IRHandler,
  (uint32_t)&EXTI8_IRHandler,
  (uint32_t)&EXTI9_IRHandler,
  (uint32_t)&EXTI10_IRHandler,
  (uint32_t)&EXTI11_IRHandler,
  (uint32_t)&EXTI12_IRHandler,
  (uint32_t)&EXTI13_IRHandler,
  (uint32_t)&EXTI14_IRHandler,
  (uint32_t)&EXTI15_IRHandler,
  (uint32_t)&RTC_IRHandler,
  (uint32_t)&SAES_IRHandler,
  (uint32_t)&AES_IRHandler,
  (uint32_t)&PKA_IRHandler,
  (uint32_t)&HASH_IRHandler,
  (uint32_t)&RNG_IRHandler,
  (uint32_t)&ADC1_2_IRHandler,
  (uint32_t)&GPDMA1_CH0_IRHandler,
  (uint32_t)&GPDMA1_CH1_IRHandler,
  (uint32_t)&GPDMA1_CH2_IRHandler,
  (uint32_t)&GPDMA1_CH3_IRHandler,
  (uint32_t)&GPDMA1_CH4_IRHandler,
  (uint32_t)&GPDMA1_CH5_IRHandler,
  (uint32_t)&GPDMA1_CH6_IRHandler,
  (uint32_t)&TIM1_BRK_IRHandler,
  (uint32_t)&TIM1_UP_IRHandler,
  (uint32_t)&TIM1_TRG_COM_IRHandler,
  (uint32_t)&TIM1_CC_IRHandler,
  (uint32_t)&TIM2_IRHandler,
  (uint32_t)&TIM3_IRHandler,
  (uint32_t)&TIM4_IRHandler,
  (uint32_t)&TIM5_IRHandler,
  (uint32_t)&TIM6_IRHandler,
  (uint32_t)&TIM7_IRHandler,
  (uint32_t)&TIM9_IRHandler,
  (uint32_t)&SPI1_IRHandler,
  (uint32_t)&SPI2_IRHandler,
  (uint32_t)&SPI3_IRHandler,
  (uint32_t)&SPI4_IRHandler,
  (uint32_t)&SPI5_IRHandler,
  (uint32_t)&SPI6_IRHandler,
  (uint32_t)&HPDMA1_CH0_IRHandler,
  (uint32_t)&HPDMA1_CH1_IRHandler,
  (uint32_t)&HPDMA1_CH2_IRHandler,
  (uint32_t)&HPDMA1_CH3_IRHandler,
  (uint32_t)&HPDMA1_CH4_IRHandler,
  (uint32_t)&HPDMA1_CH5_IRHandler,
  (uint32_t)&HPDMA1_CH6_IRHandler,
  (uint32_t)&HPDMA1_CH7_IRHandler,
  (uint32_t)&SAI1_A_IRHandler,
  (uint32_t)&SAI1_B_IRHandler,
  (uint32_t)&SAI2_A_IRHandler,
  (uint32_t)&SAI2_B_IRHandler,
  (uint32_t)&I2C1_EV_IRHandler,
  (uint32_t)&I2C1_ER_IRHandler,
  (uint32_t)&I2C2_EV_IRHandler,
  (uint32_t)&I2C2_ER_IRHandler,
  (uint32_t)&I2C3_EV_IRHandler,
  (uint32_t)&I2C3_ER_IRHandler,
  (uint32_t)&USART1_IRHandler,
  (uint32_t)&USART2_IRHandler,
  (uint32_t)&USART3_IRHandler,
  (uint32_t)&UART4_IRHandler,
  (uint32_t)&UART5_IRHandler,
  (uint32_t)&UART6_IRHandler,
  (uint32_t)&UART7_IRHandler,
  (uint32_t)&UART8_IRHandler,
  (uint32_t)&I3C1_EV_IRHandler,
  (uint32_t)&I3C1_ER_IRHandler,
  (uint32_t)&OTG_HS_IRHandler,
  (uint32_t)&ETH_IRHandler,
  (uint32_t)&CORDIC_IRHandler,
  (uint32_t)&GFXTIM_IRHandler,
  (uint32_t)&DCMIPP_IRHandler,
  (uint32_t)&LTDC_IRHandler,
  (uint32_t)&LTDC_ER_IRHandler,
  (uint32_t)&DMA2D_IRHandler,
  (uint32_t)&JPEG_IRHandler,
  (uint32_t)&GFXMMU_IRHandler,
  (uint32_t)&I3C1_WKUP_IRHandler,
  (uint32_t)&MCE1_IRHandler,
  (uint32_t)&MCE2_IRHandler,
  (uint32_t)&MCE3_IRHandler,
  (uint32_t)&OSPI1_IRHandler,
  (uint32_t)&OSPI2_IRHandler,
  (uint32_t)&FMC_IRHandler,
  (uint32_t)&SDMMC1_IRHandler,
  (uint32_t)&SDMMC2_IRHandler,
  0,
  0,
  (uint32_t)&OTG_FS_IRHandler,
  (uint32_t)&TIM12_IRHandler,
  (uint32_t)&TIM13_IRHandler,
  (uint32_t)&TIM14_IRHandler,
  (uint32_t)&TIM15_IRHandler,
  (uint32_t)&TIM16_IRHandler,
  (uint32_t)&TIM17_IRHandler,
  (uint32_t)&LPTIM1_IRHandler,
  (uint32_t)&LPTIM2_IRHandler,
  (uint32_t)&LPTIM3_IRHandler,
  (uint32_t)&LPTIM4_IRHandler,
  (uint32_t)&LPTIM5_IRHandler,
  (uint32_t)&SPDIF_RX_IRHandler,
  (uint32_t)&MDIOS_IRHandler,
  (uint32_t)&ADF1_FLT0_IRHandler,
  (uint32_t)&CRS_IRHandler,
  (uint32_t)&UCPD1_IRHandler,
  (uint32_t)&CEC_IRHandler,
  (uint32_t)&PSSI_IRHandler,
  (uint32_t)&LPUART1_IRHandler,
  (uint32_t)&WAKEUP_PIN_IRHandler,
  (uint32_t)&GPDMA1_CH8_IRHandler,
  (uint32_t)&GPDMA1_CH9_IRHandler,
  (uint32_t)&GPDMA1_CH10_IRHandler,
  (uint32_t)&GPDMA1_CH11_IRHandler,
  (uint32_t)&GPDMA1_CH12_IRHandler,
  (uint32_t)&GPDMA1_CH13_IRHandler,
  (uint32_t)&GPDMA1_CH14_IRHandler,
  (uint32_t)&GPDMA1_CH15_IRHandler,
  (uint32_t)&HPDMA1_CH8_IRHandler,
  (uint32_t)&HPDMA1_CH9_IRHandler,
  (uint32_t)&HPDMA1_CH10_IRHandler,
  (uint32_t)&HPDMA1_CH11_IRHandler,
  (uint32_t)&HPDMA1_CH12_IRHandler,
  (uint32_t)&HPDMA1_CH13_IRHandler,
  (uint32_t)&HPDMA1_CH14_IRHandler,
  (uint32_t)&HPDMA1_CH15_IRHandler,
  (uint32_t)&GPU2D_IRHandler,
  (uint32_t)&GPU2D_ER_IRHandler,
  (uint32_t)&TCACHE_IRHandler,
  (uint32_t)&FDCAN1_IT0_IRHandler,
  (uint32_t)&FDCAN1_IT1_IRHandler,
  (uint32_t)&FDCAN2_IT0_IRHandler,
  (uint32_t)&FDCAN2_IT1_IRHandler
};

extern uint32_t _etext, _sdata, _edata, _sbss, _ebss, _sidata;
void main(void);

void Reset_Handler(void)
{
    // Copy .data from FLASH to RAM
    uint32_t data_size = (uint32_t)&_edata - (uint32_t)&_sdata;
    uint8_t *flash_data = (uint8_t*)&_sidata;  // Source in FLASH
    uint8_t *sram_data = (uint8_t*)&_sdata;    // Destination in RAM
    
    for (uint32_t i = 0; i < data_size; i++) {
        sram_data[i] = flash_data[i];  // Copy byte by byte
    }
    
    // Zero out uninitialized data in .bss
    uint32_t bss_size = (uint32_t)&_ebss - (uint32_t)&_sbss;
    uint8_t *bss = (uint8_t*)&_sbss;
    
    for (uint32_t i = 0; i < bss_size; i++) {
        bss[i] = 0;  // Zero byte by byte
    }
    
    main();
}
