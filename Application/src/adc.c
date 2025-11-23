/*
 * adc.c
 *
 *  Created on: Aug 15, 2025
 *      Author: benkempers
 */

#include "adc.h"

void pa4_adc_interrupt_init()
{
    /* configure ADC GPIO pin */
    /* enable clock access to ADC GPIO pin */
    RCC->AHB4ENR |= GPIOAEN;

    /* set mode of PA4 to analog */
    GPIOA->MODER |= (1U << 8);
    GPIOA->MODER |= (1U << 9);

    /* configure the ADC module */
    /* enable clock access to ADC */
    RCC->AHB1ENR |= ADC12EN;

    /* enable ADC end-of-conversion interrupt */
    ADC1->CR |= ADC_IER_EOCIE;

    /* enable ADC interrupt in NVIC */
    __NVIC_EnableIRQ(ADC1_2_IRQn);

    /* configure conversion seq start */
    ADC1->SQR1 |= ADC_CH1;

    /* configure conversion seq len */
    ADC1->SQR1 |= ADC_SEQ_LEN_1;

    /* enable ADC module */
    ADC1->CR |= ADC_CR_ADEN;
}

static void adc_callback()
{
    uint32_t sensor_value = adc_read();
    printf("Sensor value : %d \n\r", (int) sensor_value);
}

void ADC_IRQHandler()
{
    /* wait for conversion to be complete */
    if (ADC1->ISR & ADC_SR_EOC != 0) {
        ADC1->ISR &= ~ADC_SR_EOC;

        adc_callback();
    }
}

void adc_inerrupt_driver(void)
{
    led_init();
    usart_rxtx_init();

    pa4_adc_interrupt_init();
    start_conversion();

    while (1) {
    }
}

void pa4_adc_init()
{
    /* configure ADC GPIO pin */
    /* enable clock access to ADC GPIO pin */
    RCC->AHB4ENR |= GPIOAEN;

    /* set mode of PA4 to analog */
    GPIOA->MODER |= (1U << 8);
    GPIOA->MODER |= (1U << 9);

    /* configure the ADC module */
    /* enable clock access to ADC */
    RCC->AHB1ENR |= ADC12EN;

    /* configure conversion seq start */
    ADC1->SQR1 |= ADC_CH1;

    /* configure conversion seq len */
    ADC1->SQR1 |= ADC_SEQ_LEN_1;

    /* enable ADC module */
    ADC1->CR |= ADC_CR_ADEN;
}

void start_conversion(void)
{
    /* enable continuous conversio */
    ADC1->CFGR |= ADC_CFGR_CONT;

    /* start ADC conversion */
    ADC1->CR |= ADC_CR_ADSTART;
}

void stop_conversion(void)
{
    /* stop ADC conversion & reset ADSTART */
    ADC1->CR |= ADC_CR_ADSTP;

    ADC1->CR |= (0U << 2);
}

uint32_t adc_read(void)
{
    /* wait for conversion to be complete */
    while (!(ADC1->ISR & ADC_SR_EOC)) {
    };

    /* read converted result */
    return (ADC1->DR);
}

void adc_driver(void)
{
    led_init();
    usart_rxtx_init();

    pa4_adc_init();
    start_conversion();

    while (true) {
        uint32_t sensor_value = adc_read();
        printf("Sensor value : %d \n\r", (int) sensor_value);
        led_toggle(1);
    }
}
