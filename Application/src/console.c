#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "console.h"
#include "led.h"
#include "ringbuffer.h"
#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_usart.h"
#include "system.h"

#define MAX_ARGS 16

// global variables
USART_Handle         usart3;
static char          cmd_buffer[CMD_BUFFER_SIZE];
static uint16_t      cmd_index = 0;
static volatile bool cmd_ready = false;

typedef struct {
    uint32_t timestamp;
    uint32_t error_code;
} trace_entry_t;

static trace_entry_t     trace_buffer[TRACE_SIZE];
static volatile uint16_t trace_index = 0;

static ringbuffer    tx_buffer;
static volatile bool tx_busy = false;

static ringbuffer    rx_buffer;
static uint8_t       uart_rx_byte;
static volatile bool rx_busy = false;

static void print_setup_information();
static int  parse_args(char *line, char *argv[], int max_args);
static void process_rx_buffer();

SYS_Status Console_Init()
{
    /* Initialize TX & RX Ring buffers */
    ringbuffer_init(&tx_buffer);
    ringbuffer_init(&rx_buffer);

    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Configure PD8 as USART3_TX (AF7) */
    GPIO_Init gpio_usart3_tx;
    gpio_usart3_tx.Pin       = GPIO_PIN_8;
    gpio_usart3_tx.Mode      = GPIO_MODE_ALT_FUNC_PP;
    gpio_usart3_tx.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio_usart3_tx.Pull      = GPIO_NOPULL_UP;
    gpio_usart3_tx.Alternate = ((uint8_t) 0x07); // AF7
    HAL_GPIO_Init(GPIOD, &gpio_usart3_tx);

    /* Configure PD9 as USART3_RX (AF7) */
    GPIO_Init gpio_usart3_rx;
    gpio_usart3_rx.Pin       = GPIO_PIN_9;
    gpio_usart3_rx.Mode      = GPIO_MODE_ALT_FUNC_PP;
    gpio_usart3_rx.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio_usart3_rx.Pull      = GPIO_PULL_UP;
    gpio_usart3_rx.Alternate = ((uint8_t) 0x07); // AF7
    HAL_GPIO_Init(GPIOD, &gpio_usart3_rx);

    RCC->CCIPR2 = (RCC->CCIPR2 & ~0x7U) | 0x3U;

    __HAL_RCC_USART3_CLK_ENABLE();

    /* Configure USART Handle */
    usart3.Instance            = USART3;
    usart3.Init.BaudRate       = 115200;
    usart3.Init.WordLen        = USART_WORDLENGTH_8B;
    usart3.Init.StopBits       = USART_STOPBITS_1;
    usart3.Init.Parity         = USART_PARITY_NONE;
    usart3.Init.Mode           = USART_MODE_TX_RX;
    usart3.Init.CLKPolarity    = USART_POLARITY_LOW;
    usart3.Init.CLKPhase       = USART_PHASE_1EDGE;
    usart3.Init.CLKLastBit     = USART_LASTBIT_DISABLE;
    usart3.Init.ClockPrescaler = USART_PRESCALER_DIV1;
    usart3.fifoMode            = 0;
    usart3.mask                = 0xFF;
    usart3.State               = HAL_USART_STATE_RESET;

    usart3.txCpltCallback   = HAL_USART_txCpltCallback;
    usart3.rxCpltCallback   = HAL_USART_rxCpltCallback;
    usart3.txrxCpltCallback = HAL_USART_txrxCpltCallback;
    usart3.errorCallback    = HAL_USART_errorCallback;

    // Initialize USART
    if (HAL_USART_Init(&usart3) != HAL_OK) {
        Error_Handler();
    }

    __NVIC_SetPriority(USART3_IRQn, 5);
    __NVIC_EnableIRQ(USART3_IRQn);

    print_setup_information();
    HAL_DelayMS(500);
    HAL_USART_Receiver_IT(&usart3, &uart_rx_byte, 1);

    return SYS_OK;
}

void Console_Process(void)
{
    process_rx_buffer();

    // Check for complete command
    if (cmd_ready) {
        Console_Command(cmd_buffer);

        // Reset for next command
        cmd_index = 0;
        cmd_ready = false;

        Console_Write("sh> ", 4);
    }

    // USART Error Handler Check
    for (int i = 0; i < TRACE_SIZE; i++) {
        if (trace_buffer[i].error_code != 0) {
            PRINT_INFO("[%.3f] Error: 0x%X\r\n", (trace_buffer[i].timestamp / 1000.0f), trace_buffer[i].error_code);

            trace_buffer[i].timestamp  = 0;
            trace_buffer[i].error_code = 0;
        }
    }
    trace_index = 0;
}

void Console_Command(char *command)
{
    if (strcmp(command, "help") == 0) {
        PRINT_INFO("Available commands:");
        for (shell_command_t *cmd = &__start_shell_commands; cmd < &__stop_shell_commands; cmd++) {
            PRINT_INFO("  %-10s - %s", cmd->name, cmd->help);
        }
        return;
    }

    char *argv[MAX_ARGS];
    int   argc = parse_args(command, argv, MAX_ARGS);

    for (shell_command_t *cmd = &__start_shell_commands; cmd < &__stop_shell_commands; cmd++) {
        if (strcmp(cmd->name, command) == 0) {
            cmd->handler(argc, argv);
            return;
        }
    }

    PRINT_INFO("Command not found: %s", command);
}

int Console_Write(const char *data, int len)
{
    // Disable interrupts for atomic buffer access
    __disable_irq();

    ringbuffer_put_many(&tx_buffer, (uint8_t *) data, len);

    // Start transmission if idle
    if (!tx_busy) {
        if (tx_buffer.head == tx_buffer.tail) {
            __enable_irq();
            return -1; // Nothing to send
        }

        tx_busy        = 1;
        uint8_t update = ringbuffer_get(&tx_buffer);
        HAL_USART_Transmit_IT(&usart3, &update, 1);
    }

    __enable_irq();
    return len;
}

void USART3_IRHandler(void)
{
    HAL_USART_IRQHandler(&usart3);
}

void HAL_USART_txCpltCallback(USART_Handle *handle)
{
    // Transmission complete
    if (tx_buffer.head != tx_buffer.tail) {
        uint8_t update = ringbuffer_get(&tx_buffer);
        HAL_USART_Transmit_IT(&usart3, &update, 1);
    } else {
        tx_busy = 0;
        if (handle->State == HAL_USART_STATE_BUSY_TX) {
            handle->State = HAL_USART_STATE_READY;
        } else if (handle->State == HAL_USART_STATE_BUSY_TX_RX) {
            handle->State = HAL_USART_STATE_BUSY_RX; // RX still active
        }
    }
}

void HAL_USART_rxCpltCallback(USART_Handle *handle)
{
    // Read complete
    ringbuffer_put(&rx_buffer, uart_rx_byte);
    HAL_USART_Receiver_IT(&usart3, &uart_rx_byte, 1);
}

void HAL_USART_txrxCpltCallback(USART_Handle *handle)
{
    // TX/RX complete
}

void HAL_USART_errorCallback(USART_Handle *handle)
{
    Led_Toggle(3);

    trace_buffer[trace_index].timestamp  = HAL_GetTick();
    trace_buffer[trace_index].error_code = handle->errorCode;
    trace_index                          = (trace_index + 1) % TRACE_SIZE;

    if (handle->errorCode & USART_ERROR_ORE) {
        char tmp = (char) (handle->Instance->RDR & 0xFF);
    }

    handle->errorCode = USART_ERROR_NONE;
    HAL_USART_Receiver_IT(&usart3, &uart_rx_byte, 1);

    Led_Toggle(3);
    return;
}

static void print_setup_information()
{
    PRINT_INFO("\n===========================");
    PRINT_INFO("STM32H7RS Serial Console");
    PRINT_INFO("VERSION: %u.%u.%u", MAJOR_VER, MINOR_VER, PATCH_VER);
    PRINT_INFO("GIT BRANCH: %s & HASH: %s", GIT_BRANCH, GIT_COMMIT_SHORT);
    PRINT_INFO("\n===========================\n");
}

static int parse_args(char *line, char *argv[], int max_args)
{
    int   argc = 0;
    char *p    = line;

    while (*p && argc < max_args) {
        // Skip whitespace
        while (*p && isspace(*p))
            p++;
        if (!*p)
            break;

        // Mark argument start
        argv[argc++] = p;

        // Find argument end
        while (*p && !isspace(*p))
            p++;

        // Null-terminate
        if (*p)
            *p++ = '\0';
    }

    return argc;
}

static void process_rx_buffer()
{
    while (rx_buffer.head != rx_buffer.tail) {
        char received = (char) ringbuffer_get(&rx_buffer);

        // Process character
        if (received == '\r' || received == '\n') {
            if (cmd_index > 0) {
                cmd_buffer[cmd_index] = '\0';
                cmd_ready             = true;
                Console_Write("\r\n", 2);
            }
        } else if (received == '\b' || received == 127) {
            // Backspace
            if (cmd_index > 0) {
                cmd_index--;
                Console_Write("\b \b", 3);
            }
        } else if (received >= 32 && received <= 126) {
            // Printable character
            if (cmd_index < 127) {
                cmd_buffer[cmd_index++] = received;
                Console_Write(&received, 1);
            }
        }
    }
}
