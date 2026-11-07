#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdbool.h>
#include <stdint.h>

#include "hal_gpio.h"
#include "hal_usart.h"

#include "git_version.h"
// #include "shell_command.h"

#define CMD_BUFFER_SIZE 128
#define TRACE_SIZE 8

#define PRINT_DEBUG(format, ...) printf("%s:%d %s() - " format "\r\n", __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define PRINT_INFO(format, ...)  printf(format "\r\n", ##__VA_ARGS__)
#define PRINT_ERROR(format, ...) printf("%s:%d %s() - " format "\r\n", __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)

int console_init(const usart_config *usart);
void console_process(void);
int console_write(const char *data, int len);
// void console_command(char *command);

#endif // CONSOLE_H
