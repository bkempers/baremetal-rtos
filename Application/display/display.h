#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32h7rs_hal_gpio.h"
#include "stm32h7rs_hal_rcc.h"
#include "stm32h7rs_hal_spi.h"

#include "lvgl.h"

#include "console.h"
#include "st7789.h"
#include "shell_command.h"
#include "system.h"

SYS_Status Display_Init();

void LVGL_Display_Init();
void LVGL_Display_Task();

bool       Display_ReadID();

void Display_TxCpltCallback(SPI_Handle *handle);
void Display_RxCpltCallback(SPI_Handle *handle);
void Display_ErrorCallback(SPI_Handle *handle);
void Display_SuspendCallback(SPI_Handle *handle);

#endif // DISPLAY_H
