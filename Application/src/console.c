#include "console.h"
#include "led.h"

// global variables
USART_Handle usart3;

void HAL_USART_RxCpltCallback(USART_Handle *handle)
{
    if (handle != NULL) {
        // Echo back the character
        //HAL_USART_Transmit(handle, &rx_byte, 1, 100);
        
        // Start receiving next byte
        //HAL_USART_Receiver_IT(handle, &rx_byte, 1);
    }
}

SYS_Status Console_Init()
{
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Configure PD8 as USART3_TX (AF7) */
    GPIO_Init gpio_usart3_tx;
    gpio_usart3_tx.Pin = GPIO_PIN_8;
    gpio_usart3_tx.Mode = GPIO_MODE_ALT_FUNC_PP;
    gpio_usart3_tx.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_usart3_tx.Pull = GPIO_NOPULL_UP;
    gpio_usart3_tx.Alternate = ((uint8_t)0x07); // AF7
    HAL_GPIO_Init(GPIOD, &gpio_usart3_tx);

    /* Configure PD9 as USART3_RX (AF7) */
    GPIO_Init gpio_usart3_rx;
    gpio_usart3_rx.Pin = GPIO_PIN_9;
    gpio_usart3_rx.Mode = GPIO_MODE_ALT_FUNC_PP;
    gpio_usart3_rx.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_usart3_rx.Pull = GPIO_NOPULL_UP;
    gpio_usart3_rx.Alternate = ((uint8_t)0x07); // AF7
    HAL_GPIO_Init(GPIOD, &gpio_usart3_rx);
    
    // __HAL_RCC_USART3_CLK_ENABLE();
    /* Configure USART Handle */
    usart3.Instance = USART3;
    usart3.Init.BaudRate = 115200;
    usart3.Init.WordLen = USART_WORDLENGTH_8B;
    usart3.Init.StopBits = USART_STOPBITS_1;
    usart3.Init.Parity = USART_PARITY_NONE;
    usart3.Init.Mode = USART_MODE_TX_RX;
    usart3.Init.CLKPolarity = USART_POLARITY_LOW;
    usart3.Init.CLKPhase = USART_PHASE_1EDGE;
    usart3.Init.CLKLastBit = USART_LASTBIT_DISABLE;
    usart3.Init.ClockPrescaler = USART_PRESCALER_DIV1;
    usart3.fifoMode = 0;
    usart3.State = HAL_USART_STATE_RESET;

    usart3.txCpltCallback = HAL_USART_txCpltCallback;
    usart3.rxCpltCallback = HAL_USART_rxCpltCallback;
    usart3.txrxCpltCallback = HAL_USART_txrxCpltCallback;
    usart3.errorCallback = HAL_USART_errorCallback;

    // Initialize USART
    if (HAL_USART_Init(&usart3) != HAL_OK) {
        Error_Handler();
    }

    // NVIC_SetPriority(USART3_IRQn, 5);
    // NVIC_EnableIRQ(USART3_IRQn);
    
    return SYS_OK;
}

void USART3_IRHandler(void)
{
    HAL_USART_IRQHandler(&usart3);
}

// Weak callback implementations (override as needed)
__weak void HAL_USART_TxCpltCallback(USART_Handle *handle)
{
    // Transmission complete
}

__weak void HAL_USART_TxRxCpltCallback(USART_Handle *handle)
{
    // TX/RX complete
}

__weak void HAL_USART_ErrorCallback(USART_Handle *handle)
{
    // Error occurred - you can add error handling here
    Error_Handler();
}
