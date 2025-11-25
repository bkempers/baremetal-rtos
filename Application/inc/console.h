#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7rs_hal_usart.h"
#include "stm32h7rs_hal_gpio.h"
#include "stm32h7rs_hal_rcc.h"

#include "system.h"

SYS_Status Console_Init();

#endif // CONSOLE_H
