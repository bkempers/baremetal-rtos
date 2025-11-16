#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_rcc.h"
#include <system.h>
#include <led.h>

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    SysTick_Config(64000000);
    __enable_irq();

	Led_Init();
	while(1) {
	    Led_Cycle();
	}
    
    return 1;
}
