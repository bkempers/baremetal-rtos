#include <stdio.h>
#include <string.h>

#include "console.h"
#include "led.h"
#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_usart.h"
#include "system.h"

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

static void print_setup_information();

SYS_Status Console_Init()
{
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

    // **Enable RXNE interrupt directly - don't use HAL_USART_Receive_IT**
    USART3->CR1 |= USART_CR1_RXNEIE; // Enable RX interrupt

    // Initialize USART
    if (HAL_USART_Init(&usart3) != HAL_OK) {
        Error_Handler();
    }

    __NVIC_SetPriority(USART3_IRQn, 5);
    __NVIC_EnableIRQ(USART3_IRQn);

    // print_setup_information();

    return SYS_OK;
}

static void print_setup_information()
{
    INFO("STM32H7RS Serial Console");
    INFO("VERSION: %u.%u.%u", MAJOR_VER, MINOR_VER, PATCH_VER);
}

void Console_Process(void)
{
    // Check for complete command
    if (cmd_ready) {
        // Process command
        if (strcmp(cmd_buffer, "system") == 0) {
            INFO("STM32H7RS Serial Console");
        } else if (strcmp(cmd_buffer, "ver") == 0) {
            INFO("VERSION: %u.%u.%u", MAJOR_VER, MINOR_VER, PATCH_VER);
        } else if (strcmp(cmd_buffer, "clock") == 0) {
            INFO("CLOCK: %.1f MHz", (SystemCoreClock / 1e6));
        } else if (strcmp(cmd_buffer, "git") == 0) {
            INFO("GIT BRANCH: %s & HASH: %s", GIT_BRANCH, GIT_COMMIT_SHORT);
        } else if (strlen(cmd_buffer) > 0) {
            INFO("Unknown: %s", cmd_buffer);
        }

        // Reset for next command
        cmd_index = 0;
        cmd_ready = false;

        // Show prompt
        printf("> ");
    }

    // USART Error Handler Check
    for (int i = 0; i < TRACE_SIZE; i++) {
        if (trace_buffer[i].error_code != 0) {
            printf("[%.3f] Error: 0x%X\r\n", (trace_buffer[i].timestamp / 1000.0f), trace_buffer[i].error_code);

            trace_buffer[i].timestamp  = 0;
            trace_buffer[i].error_code = 0;
        }
    }
    trace_index = 0;
}

void USART3_IRHandler(void)
{
    // ========================================================================
    // HANDLE RX OURSELVES (outside HAL state machine)
    // ========================================================================
    if ((USART3->ISR & USART_ISR_RXNE) && (USART3->CR1 & USART_CR1_RXNEIE)) {
        // Reading RDR clears RXNE automatically
        char received = (char) (USART3->RDR & 0xFF);

        // Echo character back
        while (!(USART3->ISR & USART_ISR_TXE))
            ;
        USART3->TDR = received;

        // Process character
        if (received == '\r' || received == '\n') {
            if (cmd_index > 0) {
                cmd_buffer[cmd_index] = '\0';
                cmd_ready             = true;

                // Send newline
                while (!(USART3->ISR & USART_ISR_TXE))
                    ;
                USART3->TDR = '\r';
                while (!(USART3->ISR & USART_ISR_TXE))
                    ;
                USART3->TDR = '\n';
            }
        } else if (received == '\b' || received == 127) {
            // Backspace
            if (cmd_index > 0) {
                cmd_index--;
                // Erase on terminal
                while (!(USART3->ISR & USART_ISR_TXE))
                    ;
                USART3->TDR = '\b';
                while (!(USART3->ISR & USART_ISR_TXE))
                    ;
                USART3->TDR = ' ';
                while (!(USART3->ISR & USART_ISR_TXE))
                    ;
                USART3->TDR = '\b';
            }
        } else if (received >= 32 && received <= 126) {
            // Printable character
            if (cmd_index < 127) {
                cmd_buffer[cmd_index++] = received;
            }
        }
    }

    HAL_USART_IRQHandler(&usart3);
}

void HAL_USART_txCpltCallback(USART_Handle *handle)
{
    // Transmission complete
    Led_Toggle(1);
}

void HAL_USART_rxCpltCallback(USART_Handle *handle)
{
    Led_Toggle(2);

    if (handle->Instance == USART3) {
    }
}

void HAL_USART_txrxCpltCallback(USART_Handle *handle)
{
    // TX/RX complete
    Led_Toggle(2);
}

void HAL_USART_errorCallback(USART_Handle *handle)
{
    // Error_Handler();
    Led_Toggle(3);

    trace_buffer[trace_index].timestamp  = HAL_GetTick();
    trace_buffer[trace_index].error_code = handle->errorCode;
    trace_index                          = (trace_index + 1) % TRACE_SIZE;

    if (handle->errorCode & USART_ERROR_ORE) {
        char tmp = (char) (handle->Instance->RDR & 0xFF);
    }

    handle->errorCode = USART_ERROR_NONE;

    handle->Instance->CR1 |= USART_CR1_RXNEIE;

    Led_Toggle(3);
    return;
}
