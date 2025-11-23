/*
 * systick.c
 *
 *  Created on: Aug 18, 2025
 *      Author: benkempers
 */

#include "systick.h"

void systickDelayMS(int delay)
{
    /* configure systick for # of clks / ms */
    SysTick->LOAD = SYSTICK_LOAD_VAL;

    /* clear systick current value reg */
    SysTick->VAL = 0;

    /* enable and select clk source */
    SysTick->CTRL = SYSTICK_CR_EN | SYSTICK_CR_CLK_SRC;

    for (int i = 0; i < delay; i++) {
        while ((SysTick->CTRL & SYSTICK_CR_COUNTFLAG) == 0) {
        }
    }

    /* clear systick current value reg */
    SysTick->CTRL = 0;
}

void systick_driver(int delay)
{
    led_init();
    usart_rxtx_init();

    while (true) {
        printf("1 second\n\r");
        led_toggle(1);
        systickDelayMS(delay);
    }
}

void systick_interrupt_driver()
{
    led_init();
    usart_rxtx_init();
    systickInterrupt();

    while (true) {
    }
}

void systickInterrupt(void)
{
    /* configure systick for # of clks / ms */
    SysTick->LOAD = SYS_FREQ - 1;

    /* clear systick current value reg */
    SysTick->VAL = 0;

    /* enable and select clk source */
    SysTick->CTRL = SYSTICK_CR_EN | SYSTICK_CR_CLK_SRC;

    /* enable interrupt */
    SysTick->CTRL |= SYSTICK_TICKINT;
}

static volatile uint32_t systick_counter = 0;

void SysTick_Handler()
{
    systick_counter++;
    if (systick_counter % 1000 == 0) {    // Every 1000 ticks
        printf("1000 ticks completed\n"); // Should be 1 second if 1ms each
    }
    systick_callback();
}

void systick_callback()
{
    printf("1 second\n\r");
    led_toggle(1);
}
