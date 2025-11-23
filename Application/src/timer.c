/*
 * timer.c
 *
 *  Created on: Aug 25, 2025
 *      Author: benkempers
 */

#include "timer.h"

void tim2_1hz_interrupt_init()
{
    /* enable clock access */
    RCC->APB1ENR1 |= TIM2_EN;

    /* set prescaler */
    TIM2->PSC = 6400 - 1; // 64 000 000 / 6400

    /* set auto-reload value */
    TIM2->ARR = 10000 - 1;

    /* clear counter */
    TIM2->CNT = 0;

    /* enable timer */
    TIM2->CR1 = TIMx_CR1_CEN;

    /* enable TIM interrupt */
    TIM2->DIER = TIMx_DIER_UIE;

    /* enable TIM interrupt in NVIC */
    __NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler()
{
    /* reset UIF */
    TIM2->SR &= ~TIMx_SR_UIF;

    tim2_callback();
}

void tim2_callback()
{
    printf("1 second\n\r");
    led_toggle(1);
}

void timx_interrupt_driver()
{
    led_init();
    usart_rxtx_init();

    printf("Starting timer interrupt driver.\n\r");

    tim2_1hz_interrupt_init();

    while (true) {
    }
}

void tim2_1hz_init()
{
    /* enable clock access */
    RCC->APB1ENR1 |= TIM2_EN;

    /* set prescaler */
    TIM2->PSC = 6400 - 1; // 64 000 000 / 6400

    /* set auto-reload value */
    TIM2->ARR = 10000 - 1;

    /* clear counter */
    TIM2->CNT = 0;

    /* enable timer */
    TIM2->CR1 = TIMx_CR1_CEN;
}

void tim1_output_capture_init()
{
    /* enable clock access to GPIOD */
    RCC->AHB4ENR |= GPIODEN;

    /* set PD10 to AF mode */
    GPIOD->MODER &= ~(1U << 20);
    GPIOD->MODER |= (1U << 21);

    /* set PD10 alternate function type to TIM1 */
    GPIOD->AFR[1] |= AFR10_TIM1;

    /* enable clock access to tim1*/
    RCC->APB2ENR |= TIM1_EN;

    /* set prescaler */
    TIM1->PSC = 6400 - 1; // 64 000 000 / 6400

    /* set auto-reload value */
    TIM1->ARR = 10000 - 1;

    /* set compare value for 50% duty cycle toggle */
    TIM1->CCR4 = 5000; // Toggle at half period

    /* set output compare toggle mode */
    TIM1->CCMR2 |= TIMx_CCMR2_OC4M;

    /* enable tim1 ch4 in compare mode */
    TIM1->CCER |= TIMx_CCER_CC4E;

    /* Enable main output (required for advanced timers like TIM1) */
    TIM1->BDTR |= TIM_BDTR_MOE;

    /* clear counter */
    TIM1->CNT = 0;

    /* enable timer */
    TIM1->CR1 = TIMx_CR1_CEN;
}

void tim4_input_capture_init()
{
    /* enable clock access to GPIOD */
    RCC->AHB4ENR |= GPIODEN;

    /* set PD13 to AF mode */
    GPIOD->MODER &= ~(1U << 26);
    GPIOD->MODER |= (1U << 27);

    /* set PD13 alternate function type to TIM4 */
    GPIOD->AFR[1] |= AFR13_TIM4;

    /* enable clock access to tim4 */
    RCC->APB1ENR1 |= TIM4_EN;

    /* set prescaler */
    TIM4->PSC = 6400 - 1; // 64 000 000 / 6400

    /* set TIm4 to input mode */
    TIM4->CCMR1 |= TIMx_CCMR1_CC1S;

    /* set TIM4 to capture at rising edge */
    TIM4->CCER |= TIMx_CCER_CC2E;

    /* clear counter */
    TIM4->CNT = 0;

    /* enable timer */
    TIM4->CR1 = TIMx_CR1_CEN;
}

void timx_driver(bool output_tim)
{
    if (!output_tim) {
        tim2_1hz_init();
        led_init();
        usart_rxtx_init();

        while (true) {
            while (!(TIM2->SR & TIMx_SR_UIF)) {
            }

            /* reset UIF */
            TIM2->SR &= ~TIMx_SR_UIF;

            printf("1 second\n\r");
            led_toggle(1);
        }
    } else {
        int timestamp = 0;
        tim1_output_capture_init();
        tim4_input_capture_init();

        while (true) {
            while (!(TIM4->SR & TIMx_SR_CC2IF)) {
            }

            /* read captured output value */
            timestamp = TIM4->CCR2;
            printf("timestamp %d", timestamp);
        }
    }
}
