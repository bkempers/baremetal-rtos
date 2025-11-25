#include <stdint.h>
#include <stdio.h>

#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_rcc.h"

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
        printf("X\r\n");
        for(volatile int i = 0; i < 1000000; i++);

        // printf("hello world \r\n");
        // printf("[%lu] hello world \r\n", HAL_GetTick());
        //printf("[%.3f] Hello, World!\r\n", HAL_GetTick()/1000.0f);
    }

    return 1;
}
