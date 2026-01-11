#ifndef SYSTEM_H
#define SYSTEM_H

#include "led.h"
#include "shell_command.h"
#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_rcc.h"

#define MAJOR_VER 0
#define MINOR_VER 0
#define PATCH_VER 1

#define SYSCLK_FREQ 600000000U
#define HCLK_FREQ   300000000U
#define PCLK1_FREQ  150000000U
#define PCLK2_FREQ  150000000U

typedef enum {
    SYS_ERROR = 0x0,
    SYS_OK    = 0x1,
} SYS_Status;

void SystemClock_Config(void);
void Error_Handler(void);

#endif // SYSTEM_H
