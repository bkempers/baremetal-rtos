#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7rs_hal_usart.h"
#include "stm32h7rs_hal_gpio.h"
#include "stm32h7rs_hal_rcc.h"

#include "system.h"

#define CMD_BUFFER_SIZE 128
#define TRACE_SIZE 8

SYS_Status Console_Init();
void Console_Process();

#endif // CONSOLE_H
