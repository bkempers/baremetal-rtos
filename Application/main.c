#include <stdint.h>
#include <stdio.h>

// locals
#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_rcc.h"
#include "stm32h7rs_hal_usart.h"
#include <led.h>
#include <console.h>
#include <system.h>

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    __enable_irq();

    Led_Init();
    Console_Init();

    while (1) {
        Led_Cycle();
        HAL_DelayMS(1000);

        //printf("[%.3f] Hello, World!\r\n", HAL_GetTick()/1000.0f);
        Console_Process();
    }

    return 1;
}
