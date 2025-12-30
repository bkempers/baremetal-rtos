#include <stdint.h>
#include <stdio.h>

// locals
#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_rcc.h"
#include "stm32h7rs_hal_usart.h"

#include <bme680_sensor.h>
#include <console.h>
#include <led.h>
#include <system.h>
#include <task_scheduler.h>
#include <display.h>

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    __enable_irq();

    Led_Init();
    Console_Init();

    if (BME680_Sensor_Init()) {
        Scheduler_AddTask(BME680_Read_Trigger, 1000);
    }

    if (Display_Init() == SYS_OK) {
        LVGL_Display_Init();
        Scheduler_AddTask(LVGL_Display_Task, 33);
    }

    Led_Reset();

    Scheduler_AddTask(Led_Cycle, 1000);
    Scheduler_AddTask(Console_Process, 25);
    Task_Scheduler_Init();

    while (1) {
    }

    return 1;
}
