#include <stdint.h>
#include <stdio.h>

#include <arch.h>
#include <board.h>
#include <led.h>

#include <kernel.h>

// Each task gets its own stack
KERNEL_STACK_DEFINE(led_stack, 128);
// KERNEL_STACK_DEFINE(console_stack, 256);
// KERNEL_STACK_DEFINE(bme680_stack,  256);

static void led_task(void)
{
    while (1) {
        led_toggle(BOARD_LED_GREEN);
        kernel_delay_ms(50);
        led_toggle(BOARD_LED_YELLOW);
        kernel_delay_ms(50);
        led_toggle(BOARD_LED_RED);
        kernel_delay_ms(50);
    }
}

// static void console_task(void) {
//     while (1) {
//         Console_Process();
//         kernel_delay_ms(25);
//     }
// }
//
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

    kernel_add_thread(led_task, led_stack, 128, "led_task_1");
    // kernel_add_thread(console_task, console_stack, 256);
    // kernel_add_thread(bme680_task,  bme680_stack,  256);

    kernel_launch();
}
