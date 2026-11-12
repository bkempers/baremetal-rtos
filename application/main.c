#include <stdint.h>
#include <stdio.h>

#include <arch.h>
#include <board.h>
#include <led.h>
#include <button.h>
#include <console.h>

#include <kernel.h>

// Each task gets its own stack
KERNEL_STACK_DEFINE(button_stack, 128);
KERNEL_STACK_DEFINE(led_stack, 128);
KERNEL_STACK_DEFINE(console_stack, 256);
// KERNEL_STACK_DEFINE(bme680_stack,  256);

static void button_task(void)
{
    //TODO: make this interrupt driven
    while(1) {
        if (button_read(BOARD_USER_BUTTON)) {
            led_toggle(BOARD_LED_GREEN);
            kernel_delay_ms(10);
            led_toggle(BOARD_LED_YELLOW);
            kernel_delay_ms(10);
            led_toggle(BOARD_LED_RED);
            kernel_delay_ms(10);
        }
    }
}

static void led_task(void)
{
    while (1) {
        led_toggle(BOARD_LED_GREEN);
        kernel_delay_ms(10);
        led_toggle(BOARD_LED_YELLOW);
        kernel_delay_ms(10);
        led_toggle(BOARD_LED_RED);
        kernel_delay_ms(10);
    }
}

static void console_task(void) {
    while (1) {
        console_process();
        kernel_delay_ms(25);
    }
}

// static void bme680_task(void) {
//     while (1) {
//         BME680_Read_Trigger();
//         kernel_delay_ms(1000);
//     }
// }

int main(void)
{
    if (board_init() != 0) {
        arch_halt();
    }

    kernel_init();

    kernel_add_thread(button_task, button_stack, 128, "button_task");
    // kernel_add_thread(led_task, led_stack, 128, "led_task");
    kernel_add_thread(console_task, console_stack, 256, "console_task");
    
    // kernel_add_thread(bme680_task,  bme680_stack,  256);

    kernel_launch();
}
