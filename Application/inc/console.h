#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7rs_hal_gpio.h"
#include "stm32h7rs_hal_rcc.h"
#include "stm32h7rs_hal_usart.h"

#include "git_info.h"
#include "system.h"

#define CMD_BUFFER_SIZE 128
#define TRACE_SIZE      8

#define PRINT_DEBUG(format, ...) printf("%s:%d %s() - " format "\r\n", __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define PRINT_INFO(format, ...)  printf(format "\r\n", ##__VA_ARGS__)
#define PRINT_ERROR(format, ...) printf("%s:%d %s() - " format "\r\n", __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)

SYS_Status Console_Init();
void       Console_Process();

int Console_Write(const char* data, int len);
int Console_Read(char* data, int len);

#endif // CONSOLE_H
