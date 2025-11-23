/*
 * exti.c
 *
 *  Created on: Sep 8, 2025
 *      Author: benkempers
 */

#include "exti.h"

void pc13_exti_init()
{
    /* disable global interrupts */
    __disable_irq();

    /* enable clock access for GPIOC */
    RCC->AHB4ENR |= GPIOCEN;

    /* set PC13 as input pin */
    GPIOC->MODER &= ~(1U << 26);
    GPIOC->MODER &= ~(1U << 27);

    // Add pull-up for the button (if external pull-up isn't present)
    GPIOC->PUPDR &= ~(3U << 26); // Clear pull-up/down bits
    GPIOC->PUPDR |= (1U << 26);  // Set pull-up

    /* enable clock access for SYSCFG (SBSEN) */
    RCC->APB4ENR |= APB4ENR_SBSEN;

    /* select PORTC for EXTI13 */
    SBS->EXTICR[3] &= ~(0xF << 4); // Clear bits 7:4 (EXTI13 field)
    SBS->EXTICR[3] |= (2 << 4);    // Set bits 7:4 to 0010 (Port C)

    /* unmask EXTI13 */
    EXTI->IMR1 |= (1U << 13);

    /* select falling edge trigger */
    EXTI->FTSR1 |= (1U << 13);

    /* enable EXTI13 line in NVIC */
    __NVIC_EnableIRQ(EXTI13_IRQn);

    /* enable global interrupts */
    __enable_irq();
}

static void exti_callback(void)
{
    printf("Button pressed...\n\r");
    led_toggle(1);
}

void EXTI13_IRQHandler()
{
    if ((EXTI->PR1 & LINE_13) != 0) {
        EXTI->PR1 |= LINE_13;
        exti_callback();
    }
}

void gpio_interrupt_driver()
{
    led_init();
    usart_rxtx_init();

    led_toggle(3);
    printf("Starting GPIO interrupt driver. \n\r");

    pc13_exti_init();

    while (1) {
    }
}
