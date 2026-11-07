#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "console.h"
#include "git_version.h"
#include "led.h"
#include "ringbuffer.h"
#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_usart.h"
#include "system.h"

#define MAX_ARGS 16

// global variables
static char cmd_buffer[CMD_BUFFER_SIZE];
static uint16_t cmd_index = 0;
static volatile bool cmd_ready = false;

typedef struct {
    uint32_t timestamp;
    uint32_t error_code;
} trace_entry_t;

static trace_entry_t     trace_buffer[TRACE_SIZE];
static volatile uint16_t trace_index = 0;

static void print_setup_information();
static int  parse_args(char *line, char *argv[], int max_args);
static void process_rx_buffer();

int console_init(const usart_config *usart)
{
    if (usart == NULL || usart_init(usart) != 0) {
        return -1;
    }

    print_setup_information();
    return 0;
}

void console_process(void)
{
    // process_rx_buffer();

    // Check for complete command
    // if (cmd_ready) {
    //     Console_Command(cmd_buffer);
    //
    //     // Reset for next command
    //     cmd_index = 0;
    //     cmd_ready = false;
    //
    //     Console_Write("sh> ", 4);
    // }

    // USART Error Handler Check
    for (int i = 0; i < TRACE_SIZE; i++) {
        if (trace_buffer[i].error_code != 0) {
            PRINT_INFO("[%d] Error: 0x%d\r\n", trace_buffer[i].timestamp, trace_buffer[i].error_code);

            trace_buffer[i].timestamp  = 0;
            trace_buffer[i].error_code = 0;
        }
    }
    trace_index = 0;
}

int console_write(const char *data, int len)
{
    return (int) usart_write((const uint8_t *)data, (size_t) len);
}

// void Console_Command(char *command)
// {
//     if (strcmp(command, "help") == 0) {
//         PRINT_INFO("Available commands:");
//         for (shell_command_t *cmd = &__start_shell_commands; cmd < &__stop_shell_commands; cmd++) {
//             PRINT_INFO("  %-10s - %s", cmd->name, cmd->help);
//         }
//         return;
//     }
//
//     char *argv[MAX_ARGS];
//     int   argc = parse_args(command, argv, MAX_ARGS);
//
//     for (shell_command_t *cmd = &__start_shell_commands; cmd < &__stop_shell_commands; cmd++) {
//         if (strcmp(cmd->name, command) == 0) {
//             cmd->handler(argc, argv);
//             return;
//         }
//     }
//
//     PRINT_INFO("Command not found: %s", command);
// }

static void print_setup_information()
{
    PRINT_INFO("\n===========================");
    PRINT_INFO("STM32H7RS Serial Console");
    // PRINT_INFO("VERSION: %u.%u.%u", MAJOR_VER, MINOR_VER, PATCH_VER);
    PRINT_INFO("GIT BRANCH: %s & HASH: %s", GIT_BRANCH, GIT_REV);
    PRINT_INFO("\n===========================\n");
}

static int parse_args(char *line, char *argv[], int max_args)
{
    int   argc = 0;
    char *p    = line;

    while (*p && argc < max_args) {
        // Skip whitespace
        while (*p && isspace((unsigned char) *p))
            p++;
        if (!*p)
            break;

        // Mark argument start
        argv[argc++] = p;

        // Find argument end
        while (*p && !isspace((unsigned char) *p))
            p++;

        // Null-terminate
        if (*p)
            *p++ = '\0';
    }

    return argc;
}

// static void process_rx_buffer(void)
// {
//     while (usart_read(&ch, 1) == 1) {
//         char received = (char) ringbuffer_get(&rx_buffer);
//
//         // Process character
//         if (received == '\r' || received == '\n') {
//             if (cmd_index > 0) {
//                 cmd_buffer[cmd_index] = '\0';
//                 cmd_ready             = true;
//                 console_write("\r\n", 2);
//             }
//         } else if (received == '\b' || received == 127) {
//             // Backspace
//             if (cmd_index > 0) {
//                 cmd_index--;
//                 console_write("\b \b", 3);
//             }
//         } else if (received >= 32 && received <= 126) {
//             // Printable character
//             if (cmd_index < 127) {
//                 cmd_buffer[cmd_index++] = received;
//                 console_write(&received, 1);
//             }
//         }
//     }
// }
