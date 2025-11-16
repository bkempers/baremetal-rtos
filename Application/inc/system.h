#ifndef SYSTEM_H
#define SYSTEM_H

#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_rcc.h"
#include "led.h"

void SystemClock_Config(void);
void Error_Handler(void);

// Optionally expose clock frequencies
#define SYSCLK_FREQ    600000000U
#define HCLK_FREQ      300000000U
#define PCLK1_FREQ     150000000U
#define PCLK2_FREQ     150000000U

#endif // SYSTEM_H
