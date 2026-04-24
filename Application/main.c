#include <stdint.h>
#include <stdio.h>

// locals
#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_rcc.h"
#include "stm32h7rs_hal_usart.h"

#include <board.h>
#include <sensors/bme680/bme680_sensor.h>
#include <console.h>
#include <led.h>
#include <system.h>
#include <task_scheduler.h>
#include <display.h>

#include <kernel.h>

// Each task gets its own stack
KERNEL_STACK_DEFINE(led_stack,     128);
// KERNEL_STACK_DEFINE(console_stack, 256);
// KERNEL_STACK_DEFINE(bme680_stack,  256);

static void led_task(void) {
    while (1) {
        Led_Toggle(1);
        kernel_delay_ms(50);
        Led_Toggle(2);
        kernel_delay_ms(50);
        Led_Toggle(3);
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
    HAL_Init();
    SystemClock_Config();

    Led_Init();
    // Console_Init();
    //
    // if (BME680_Sensor_Init()) {
    //     PRINT_INFO("BME680 init failed");
    // }
    //
    // if (Display_Init() == SYS_OK) {
    //     // LVGL_Display_Init();
    //     // Scheduler_AddTask(LVGL_Display_Task, 33);
    // } else {
    //     PRINT_INFO("failed to start display");
    // }
    //
    Led_Reset();

    // Replace Scheduler_AddTask with kernel_add_thread
    kernel_add_thread(led_task, led_stack, 128, "led_task_1");
    // kernel_add_thread(console_task, console_stack, 256);
    // kernel_add_thread(bme680_task,  bme680_stack,  256);

    // Replaces Task_Scheduler_Init() — does not return
    kernel_launch();

    return 1;
}
