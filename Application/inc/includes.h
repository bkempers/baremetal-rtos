/*
 * includes.h
 *
 *  Created on: Aug 15, 2025
 *      Author: benkempers
 */

#ifndef INCLUDES_H_
#define INCLUDES_H_

#include "stm32h7rsxx.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define SYS_FREQ 64000000

#define GPIOAEN (1U << 0)
#define GPIOBEN (1U << 1)
#define GPIOCEN (1U << 2)
#define GPIODEN (1U << 3)

#define PIN5  (1U << 5)
#define PIN7  (1U << 7)
#define PIN10 (1U << 10)
#define PIN13 (1U << 13)

/* LED DEFINES */

#define LED1_PIN (PIN10) // PD10
#define LED2_PIN (PIN13) // PD13
#define LED3_PIN (PIN7)  // PB7

#define BTN2_PIN (PIN13) // PC13

/* UART DEFINES */

#define ABP1_CLK SYS_FREQ

#define UART_BAUDRATE 115200

#define CR1_FIFOEN_DISABLE (0U << 29)
#define CR1_TE             (1U << 3)
#define CR1_RE             (1U << 2)
#define CR1_UE             (1U << 0)
#define CR1_RXNEIE         (1U << 5)

#define ISR_ALT_RXNE (1U << 5)
#define ISR_ALT_TXE  (1U << 7)

#define USART3EN (1U << 18)

/* GPDMA DEFINES */

#define GPDMA1EN (1U << 4)

#define GPDMA1_CCR_EN    (1U << 0)
#define GPDMA1_CCR_TCIE  (1U << 8)
#define GPDMA1_CCR_HTIE  (1U << 9)
#define GPDMA1_CCR_DTEIE (1U << 10)
#define GPDMA1_CCR_USEIE (1U << 12)

#define GPDMA1_CTR1_SDW_MASK (0x3U << 0)  // Source data width (bits 1:0)
#define GPDMA1_CTR1_DDW_MASK (0x3U << 16) // Dest data width (bits 17:16)
#define GPDMA1_CRT1_DINC     (1U << 19)
#define GPDMA1_CTR1_SINC     (1U << 3)

#define GPDMA1_CTR2_DREQ        (1U << 10)
#define GPDMA1_CTR2_PFREQ       (1U << 12)
#define GPDMA1_CTR2_REQSEL_MASK (0x7FU << 0)

#define GPDMA1_CSR_TCF  (1U << 8)
#define GPDMA1_CFCR_TCF (1U << 8)

#define USART_CR3_DMAT (1U << 7)

// interrupts
// All clear flags
#define GPDMA_CFCR_ALL_FLAGS (0x7F << 8)

#define GPDMA_CFCR_TCF0   (1U << 8)
#define GPDMA_CFCR_HTF0   (1U << 9)
#define GPDMA_CFCR_DTEF0  (1U << 10)
#define GPDMA_CFCR_ULEF0  (1U << 11)
#define GPDMA_CFCR_USEF0  (1U << 12)
#define GPDMA_CFCR_SUSPF0 (1U << 13)
#define GPDMA_CFCR_TOF0   (1U << 14)

/* ADC DEFINES */

#define ADC12EN       (1U << 5)
#define ADC_CH1       (1U << 6)
#define ADC_SEQ_LEN_1 (0U << 0)
#define ADC_CR_ADEN   (1U << 0)

#define ADC_CR_ADSTART (1U << 2)
#define ADC_CR_ADSTP   (1U << 4)

#define ADC_SR_EOC (1U << 2)

#define ADC_CFGR_CONT (1U << 13)

#define ADC_IER_EOCIE (1U << 2)

/* SYSTICK DEFINES */

#define SYSTICK_LOAD_VAL     64000
#define SYSTICK_CR_EN        (1U << 0)
#define SYSTICK_CR_CLK_SRC   (1U << 2)
#define SYSTICK_CR_COUNTFLAG (1U << 16)
#define SYSTICK_TICKINT      (1U << 1)

/* TIMER DEFINES */

#define TIM1_EN (1U << 0)
#define TIM2_EN (1U << 0)
#define TIM4_EN (1U << 0)

#define TIMx_CR1_CEN (1U << 0)
#define TIMx_SR_UIF  (1U << 0)

#define TIMx_CCMR2_OC4M (1U << 12) | (1U << 13)
#define TIMx_CCER_CC4E  (1U << 12)

#define TIMx_CCMR1_CC1S (1U << 8)
#define TIMx_CCER_CC2E  (1U << 4)

#define AFR10_TIM1    (1U << 8)
#define AFR13_TIM4    (1U << 21)
#define TIMx_SR_CC2IF (1U << 2)

#define TIMx_DIER_UIE (1U << 0)

/* INTERRUPTS DEFINES */
#define APB4ENR_SBSEN (1U << 1)
#define LINE_13       (1U << 13)

#endif /* INCLUDES_H_ */
