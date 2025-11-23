#include "stm32h7rs_hal_usart.h"
#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_rcc.h"
#include "stm32h7rsxx.h"
#include "stm32h7s3xx.h"

#define USART_TX_RX_TIMEOUT 1000U

/*
 * CR1_M - M0/M1 word length
 * CR1_PCE - Parity control enable
 * CR1_PS - Parity selection
 * CR1_TE - Transmitter enable
 * CR1_RC - Receiver enable
 * CR1_OVER8 - Oversamping by 8-bit
 */
#define USART_CR1_CLEARMASK                                                                                                                          \
    ((uint32_t) (USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8 | USART_CR1_FIFOEN))

/*
 * CR2_CPHA - Clock phase
 * CR2_CPOL - Clock polarity
 * CR2_CLKEN - Clock enable
 * CR2_LBCL - Last clock bit pulse
 * CR2_STOP - Stop bits
 * CR2_SLVEN - Synchronous slave mode enable
 * CR2_DIS_NSS - NSS pin input is ignored
 */
#define USART_CR2_CLEARMASK                                                                                                                          \
    ((uint32_t) (USART_CR2_CPHA | USART_CR2_CPOL | USART_CR2_CLKEN | USART_CR2_LBCL | USART_CR2_STOP | USART_CR2_SLVEN | USART_CR2_DIS_NSS))

static HAL_Status      usart_set_baudrate(USART_TypeDef *USARTx, uint32_t BaudRate);
static inline uint32_t usart_get_clock_source(USART_TypeDef *USARTx);
static uint32_t        usart_get_clock_freq(USART_ClockSource clock_source);
static HAL_Status      usart_set_config(USART_Handle *handle);
static HAL_Status      usart_check_idle(USART_Handle *handle);
static HAL_Status      usart_check_flag(USART_Handle *handle, uint32_t flag, FlagStatus status, uint32_t tickstart, uint32_t timeout);

static void USART_TxISR_16FIFO(USART_Handle *handle);
static void USART_TxISR_8FIFO(USART_Handle *handle);
static void USART_TxISR_16(USART_Handle *handle);
static void USART_TxISR_8(USART_Handle *handle);

static void USART_RxISR_16FIFO(USART_Handle *handle);
static void USART_RxISR_8FIFO(USART_Handle *handle);
static void USART_RxISR_16(USART_Handle *handle);
static void USART_RxISR_8(USART_Handle *handle);

static void usart_enable_interrupt(USART_Handle *handle, uint32_t interrupt);
static void usart_disable_interrupt(USART_Handle *handle, uint32_t interrupt);

HAL_Status HAL_USART_Init(USART_Handle *handle)
{
    if (handle == NULL) {
        return HAL_ERROR;
    }

    if (handle->State == HAL_USART_STATE_RESET) {
        handle->Lock = HAL_LOCKED;
    }

    handle->State = HAL_USART_STATE_BUSY;

    // DISABLE USART
    handle->Instance->CR1 &= ~USART_CR1_UE;

    if (usart_set_config(handle) == HAL_ERROR) {
        return HAL_ERROR;
    }

    /* In Synchronous SPI mode, the following bits must be kept cleared:
    - LINEN bit in the USART_CR2 register
    - HDSEL, SCEN and IREN bits in the USART_CR3 register.
    */
    handle->Instance->CR2 &= ~USART_CR2_LINEN;
    handle->Instance->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);

    // ENABLE USART
    handle->Instance->CR1 |= USART_CR1_UE;

    return usart_check_idle(handle);
}

HAL_Status HAL_USART_DeInit(USART_Handle *handle)
{
    // TODO: add later
    return HAL_OK;
}

HAL_Status HAL_USART_Receive(USART_Handle *handle, uint8_t *rxPointer, uint16_t size, uint32_t timeout)
{
    uint8_t  *rx_data_8b;
    uint16_t *rx_data_16b;
    uint16_t  uhMask;
    uint32_t  tickstart;

    if (handle->State == HAL_USART_STATE_READY) {
        if ((rxPointer == NULL) || (size == 0U)) {
            return HAL_ERROR;
        }

        handle->errorCode = USART_ERROR_NONE;
        handle->State     = HAL_USART_STATE_BUSY_RX;

        /* Init tickstart for timeout management */
        tickstart = HAL_GetTick();

        handle->rxSize  = size;
        handle->rxCount = size;

        /* Computation of USART mask to apply to RDR register */
        // USART_MASK_COMPUTATION(husart);
        // uhMask = husart->Mask;

        /* In case of 9bits/No Parity transfer, pRxData needs to be handled as a
         * uint16_t pointer */
        if ((handle->Init.WordLen == USART_WORDLENGTH_9B) && (handle->Init.Parity == USART_PARITY_NONE)) {
            rx_data_8b  = NULL;
            rx_data_16b = (uint16_t *) rxPointer;
        } else {
            rx_data_8b  = (uint8_t *) rxPointer;
            rx_data_16b = NULL;
        }

        /* as long as data have to be received */
        while (handle->rxCount > 0U) {
            if (usart_check_flag(handle, USART_FLAG_RXNE, RESET, tickstart, timeout) != HAL_OK) {
                return HAL_TIMEOUT;
            }

            if (rx_data_8b == NULL) {
                *rx_data_16b = (uint16_t) (handle->Instance->RDR & uhMask);
                rx_data_16b++;
            } else {
                *rx_data_8b = (uint8_t) (handle->Instance->RDR & (uint8_t) (uhMask & 0xFFU));
                rx_data_8b++;
            }

            handle->rxCount--;
        }

        handle->State = HAL_USART_STATE_READY;

        return HAL_OK;
    } else {
        return HAL_BUSY;
    }
}

uint32_t HAL_USART_Transmit(USART_Handle *handle, const uint8_t *txPointer, uint16_t size, uint32_t timeout)
{
    const uint8_t  *tx_data_8b;
    const uint16_t *tx_data_16b;
    uint32_t        tickstart;

    if (handle->State == HAL_USART_STATE_READY) {
        if ((txPointer == NULL) || (size == 0U)) {
            return HAL_ERROR;
        }

        handle->errorCode = USART_ERROR_NONE;
        handle->State     = HAL_USART_STATE_BUSY_TX;

        /* Init tickstart for timeout management */
        tickstart = HAL_GetTick();

        handle->txSize  = size;
        handle->txCount = size;

        /* In case of 9bits/No Parity transfer, pTxData needs to be handled as a
         * uint16_t pointer */
        if ((handle->Init.WordLen == USART_WORDLENGTH_9B) && (handle->Init.Parity == USART_PARITY_NONE)) {
            tx_data_8b  = NULL;
            tx_data_16b = (const uint16_t *) txPointer;
        } else {
            tx_data_8b  = txPointer;
            tx_data_16b = NULL;
        }

        /* Check the remaining data to be sent */
        while (handle->txCount > 0U) {
            if (usart_check_flag(handle, USART_FLAG_TXE, RESET, tickstart, timeout) != HAL_OK) {
                return HAL_TIMEOUT;
            }
            if (tx_data_8b == NULL) {
                handle->Instance->TDR = (uint16_t) (*tx_data_16b & 0x01FFU);
                tx_data_16b++;
            } else {
                handle->Instance->TDR = (uint8_t) (*tx_data_8b & 0xFFU);
                tx_data_8b++;
            }

            handle->txCount--;
        }

        if (usart_check_flag(handle, USART_FLAG_TC, RESET, tickstart, timeout) != HAL_OK) {
            return HAL_TIMEOUT;
        }

        handle->Instance->ICR = (uint32_t) USART_CLEAR_TCF;
        handle->Instance->ICR = (uint32_t) USART_CLEAR_OREF;

        handle->Instance->RQR |= (uint16_t) USART_RQR_RXFRQ;
        handle->Instance->RQR |= (uint16_t) USART_RQR_TXFRQ;

        handle->State = HAL_USART_STATE_READY;
        return HAL_OK;
    } else {
        return HAL_BUSY;
    }
}

HAL_Status HAL_USART_Receiver_IT(USART_Handle *handle, uint8_t *rxPointer, uint16_t size)
{
    uint16_t nb_dummy_data;

    if (handle->State == HAL_USART_STATE_READY) {
        if ((rxPointer == NULL) || (size == 0U)) {
            return HAL_ERROR;
        }

        handle->rxPointer = rxPointer;
        handle->rxSize    = size;
        handle->rxCount   = size;
        handle->RxISR     = NULL;

        handle->errorCode = USART_ERROR_NONE;
        handle->State     = HAL_USART_STATE_BUSY_RX;

        /* Enable the USART Error Interrupt: (Frame error, noise error, overrun error) */
        SET_BIT(handle->Instance->CR3, USART_CR3_EIE);

        /* Configure Rx interrupt processing */
        if ((handle->fifoMode == USART_CR1_FIFOEN) && (size >= handle->rxProcessData)) {
            /* Set the Rx ISR function pointer according to the data word length */
            if ((handle->Init.WordLen == USART_WORDLENGTH_9B) && (handle->Init.Parity == USART_PARITY_NONE)) {
                handle->RxISR = USART_RxISR_16FIFO;
            } else {
                handle->RxISR = USART_RxISR_8FIFO;
            }

            /* Enable the USART Parity Error interrupt and RX FIFO Threshold interrupt */
            if (handle->Init.Parity != USART_PARITY_NONE) {
                SET_BIT(handle->Instance->CR1, USART_CR1_PEIE);
            }
            SET_BIT(handle->Instance->CR3, USART_CR3_RXFTIE);
        } else {
            /* Set the Rx ISR function pointer according to the data word length */
            if ((handle->Init.WordLen == USART_WORDLENGTH_9B) && (handle->Init.Parity == USART_PARITY_NONE)) {
                handle->RxISR = USART_RxISR_16;
            } else {
                handle->RxISR = USART_RxISR_8;
            }

            /* Enable the USART Parity Error and Data Register not empty Interrupts */
            if (handle->Init.Parity != USART_PARITY_NONE) {
                SET_BIT(handle->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE_RXFNEIE);
            } else {
                SET_BIT(handle->Instance->CR1, USART_CR1_RXNEIE_RXFNEIE);
            }
        }

        return HAL_OK;
    } else {
        return HAL_BUSY;
    }
}

HAL_Status HAL_USART_Transmit_IT(USART_Handle *handle, const uint8_t *txPointer, uint16_t size)
{
    if (handle->State == HAL_USART_STATE_READY) {
        if ((txPointer == NULL) || (size == 0U)) {
            return HAL_ERROR;
        }

        handle->txPointer = txPointer;
        handle->txSize    = size;
        handle->txCount   = size;
        handle->TxISR     = NULL;

        handle->errorCode = USART_ERROR_NONE;
        handle->State     = HAL_USART_STATE_BUSY_TX;

        /* The USART Error Interrupts: (Frame error, noise error, overrun error)
        are not managed by the USART Transmit Process to avoid the overrun interrupt
        when the usart mode is configured for transmit and receive "USART_MODE_TX_RX"
        to benefit for the frame error and noise interrupts the usart mode should be
        configured only for transmit "USART_MODE_TX" */

        /* Configure Tx interrupt processing */
        if (handle->fifoMode == USART_CR1_FIFOEN) {
            /* Set the Tx ISR function pointer according to the data word length */
            if ((handle->Init.WordLen == USART_WORDLENGTH_9B) && (handle->Init.Parity == USART_PARITY_NONE)) {
                handle->TxISR = USART_TxISR_16FIFO;
            } else {
                handle->TxISR = USART_TxISR_8FIFO;
            }

            /* Enable the TX FIFO threshold interrupt */
            usart_enable_interrupt(handle, USART_IT_TXFT);
        } else {
            /* Set the Tx ISR function pointer according to the data word length */
            if ((handle->Init.WordLen == USART_WORDLENGTH_9B) && (handle->Init.Parity == USART_PARITY_NONE)) {
                handle->TxISR = USART_TxISR_16;
            } else {
                handle->TxISR = USART_TxISR_8;
            }

            /* Enable the USART Transmit Data Register Empty Interrupt */
            usart_enable_interrupt(handle, USART_IT_TXE);
        }

        return HAL_OK;
    } else {
        return HAL_BUSY;
    }
}

HAL_Status HAL_USART_Receive_Transmit_IT(USART_Handle *handle, uint8_t *rxPointer, const uint8_t *txPointer, uint16_t size) {}

void USART3_IRHandler(void) {}

static void USART_RxISR_8(USART_Handle *handle) 
{

}

static void USART_RxISR_16(USART_Handle *handle)
{

}

static void USART_RxISR_8FIFO(USART_Handle *handle)
{

}

static void USART_RxISR_16FIFO(USART_Handle *handle)
{

}

static void USART_TxISR_8(USART_Handle *handle)
{
    const HAL_USART_State state = handle->State;

    /* Check that a Tx process is ongoing */
    if ((state == HAL_USART_STATE_BUSY_TX) || (state == HAL_USART_STATE_BUSY_TX_RX)) {
        if (handle->txCount == 0U) {
            /* Disable the USART Transmit data register empty interrupt */
            usart_disable_interrupt(handle, USART_IT_TXE);

            /* Enable the USART Transmit Complete Interrupt */
            usart_enable_interrupt(handle, USART_IT_TC);
        } else {
            handle->Instance->TDR = (uint8_t) (*handle->txPointer & (uint8_t) 0xFF);
            handle->txPointer++;
            handle->txCount--;
        }
    }
}

static void USART_TxISR_16(USART_Handle *handle)
{
    const HAL_USART_State state = handle->State;
    const uint16_t       *tmp;

    if ((state == HAL_USART_STATE_BUSY_TX) || (state == HAL_USART_STATE_BUSY_TX_RX)) {
        if (handle->txCount == 0U) {
            /* Disable the USART Transmit data register empty interrupt */
            usart_disable_interrupt(handle, USART_IT_TXE);

            /* Enable the USART Transmit Complete Interrupt */
            usart_enable_interrupt(handle, USART_IT_TC);
        } else {
            tmp                   = (const uint16_t *) handle->txPointer;
            handle->Instance->TDR = (uint16_t) (*tmp & 0x01FFU);
            handle->txPointer += 2U;
            handle->txCount--;
        }
    }
}

static void USART_TxISR_16FIFO(USART_Handle *handle)
{
    const HAL_USART_State state = handle->State;
    const uint16_t       *tmp;
    uint16_t              nb_tx_data;

    /* Check that a Tx process is ongoing */
    if ((state == HAL_USART_STATE_BUSY_TX) || (state == HAL_USART_STATE_BUSY_TX_RX)) {
        for (nb_tx_data = handle->txProcessData; nb_tx_data > 0U; nb_tx_data--) {
            if (handle->txCount == 0U) {
                /* Disable the TX FIFO threshold interrupt */
                usart_disable_interrupt(handle, USART_IT_TXFT);

                /* Enable the USART Transmit Complete Interrupt */
                usart_enable_interrupt(handle, USART_IT_TC);

                break; /* force exit loop */
            } else if (((handle->Instance->ISR & (USART_FLAG_TXFNF)) == USART_FLAG_TXFNF) == SET) {
                tmp                   = (const uint16_t *) handle->txPointer;
                handle->Instance->TDR = (uint16_t) (*tmp & 0x01FFU);
                handle->txPointer += 2U;
                handle->txCount--;
            } else {
                /* Nothing to do */
            }
        }
    }
}

static void USART_TxISR_8FIFO(USART_Handle *handle)
{
    const HAL_USART_State state = handle->State;
    uint16_t              nb_tx_data;

    /* Check that a Tx process is ongoing */
    if ((state == HAL_USART_STATE_BUSY_TX) || (state == HAL_USART_STATE_BUSY_TX_RX)) {
        for (nb_tx_data = handle->txProcessData; nb_tx_data > 0U; nb_tx_data--) {
            if (handle->txCount == 0U) {
                /* Disable the TX FIFO threshold interrupt */
                usart_disable_interrupt(handle, USART_IT_TXFT);

                /* Enable the USART Transmit Complete Interrupt */
                usart_enable_interrupt(handle, USART_IT_TC);

                break; /* force exit loop */
            } else if (((handle->Instance->ISR & (USART_FLAG_TXFNF)) == USART_FLAG_TXFNF) == SET) {
                handle->Instance->TDR = (uint8_t) (*handle->txPointer & (uint8_t) 0xFF);
                handle->txPointer++;
                handle->txCount--;
            } else {
                /* Nothing to do */
            }
        }
    }
}

static void usart_enable_interrupt(USART_Handle *handle, uint32_t interrupt)
{
    // Extract which control register (CR1=1, CR2=2, CR3=3)
    uint32_t cr_number = (interrupt & USART_CR_MASK) >> USART_CR_POS;

    // Extract the bit position within that register
    uint32_t bit_position = interrupt & USART_IT_MASK;

    // Set the bit in the appropriate control register
    if (cr_number == 1) {
        handle->Instance->CR1 |= (1UL << bit_position);
    } else if (cr_number == 2) {
        handle->Instance->CR2 |= (1UL << bit_position);
    } else {
        handle->Instance->CR3 |= (1UL << bit_position);
    }
}

static void usart_disable_interrupt(USART_Handle *handle, uint32_t interrupt)
{
    // Extract which control register (CR1=1, CR2=2, CR3=3)
    uint32_t cr_number = (interrupt & USART_CR_MASK) >> USART_CR_POS;

    // Extract the bit position within that register
    uint32_t bit_position = interrupt & USART_IT_MASK;

    // Clear the bit in the appropriate control register
    if (cr_number == 1) {
        handle->Instance->CR1 &= ~(1UL << bit_position);
    } else if (cr_number == 2) {
        handle->Instance->CR2 &= ~(1UL << bit_position);
    } else {
        handle->Instance->CR3 &= ~(1UL << bit_position);
    }
}

static HAL_Status usart_check_flag(USART_Handle *handle, USART_Flag flag, FlagStatus status, uint32_t tickstart, uint32_t timeout)
{
    /* Wait until flag is set */
    while ((((handle->Instance->ISR & flag) == flag) ? SET : RESET) == status) {
        /* Check for the Timeout */
        if (((HAL_GetTick() - tickstart) > timeout) || (timeout == 0U)) {
            handle->State = HAL_USART_STATE_READY;
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

static HAL_Status usart_check_idle(USART_Handle *handle)
{
    uint32_t tickstart;
    handle->errorCode = USART_ERROR_NONE;
    tickstart         = HAL_GetTick();

    if ((handle->Instance->CR1 & USART_CR1_TE) == USART_CR1_TE) {
        /* Wait until TEACK flag is set */
        if (usart_check_flag(handle, USART_FLAG_TEACK, RESET, tickstart, USART_TX_RX_TIMEOUT) != HAL_OK) {
            return HAL_TIMEOUT;
        }
    }

    if ((handle->Instance->CR1 & USART_CR1_RE) == USART_CR1_RE) {
        /* Wait until REACK flag is set */
        if (usart_check_flag(handle, USART_FLAG_REACK, RESET, tickstart, USART_TX_RX_TIMEOUT) != HAL_OK) {
            return HAL_TIMEOUT;
        }
    }

    handle->State = HAL_USART_STATE_READY;
    return HAL_OK;
}

static HAL_Status usart_set_config(USART_Handle *handle)
{
    HAL_Status ret = HAL_OK;

    // USART CR1 CONFIGURATION
    uint32_t cr1_config = (uint32_t) handle->Init.WordLen | handle->Init.Parity | handle->Init.Mode | USART_CR1_OVER8;
    MODIFY_REG(handle->Instance->CR1, USART_CR1_CLEARMASK, cr1_config);

    // USART CR2 CONFIGURATION
    uint32_t cr2_config = (uint32_t) USART_CR2_CLKEN;
    cr2_config |= handle->Init.CLKLastBit | handle->Init.CLKPolarity | handle->Init.CLKPhase | handle->Init.StopBits;
    MODIFY_REG(handle->Instance->CR2, USART_CR2_CLEARMASK, cr2_config);

    // USART PRESCALER CONFIGURATION
    MODIFY_REG(handle->Instance->PRESC, USART_PRESC_PRESCALER, handle->Init.ClockPrescaler);

    // USART BRR (baudrate)
    ret = usart_set_baudrate(handle->Instance, handle->Init.BaudRate);

    handle->TxISR = NULL;
    handle->RxISR = NULL;

    return ret;
}

static inline uint32_t usart_get_clock_source(USART_TypeDef *USARTx)
{
    if (USARTx == USART1) {
        return (RCC->CCIPR2 >> 0) & 0x7;
    } else if (USARTx == USART2) {
        return (RCC->CCIPR2 >> 3) & 0x7;
    } else if (USARTx == USART3) {
        return (RCC->CCIPR2 >> 6) & 0x7;
    }

    return USART_CLOCKSOURCE_UNDEFINED;
}

static uint32_t usart_get_clock_freq(USART_ClockSource clock_source)
{
    switch (clock_source) {
    case USART_CLOCKSOURCE_PCLK1:
        return HAL_RCC_GetPCLK1Freq();
    case USART_CLOCKSOURCE_PCLK2:
        return HAL_RCC_GetPCLK2Freq();
    case USART_CLOCKSOURCE_HSI:
        // Check if HSI divider is enabled (bit in RCC_CR)
        if (RCC->CR & RCC_CR_HSIDIV) {
            uint32_t divider = (RCC->CR & RCC_CR_HSIDIV_Msk) >> RCC_CR_HSIDIV_Pos;
            return HSI_VALUE >> divider;
        }
        return HSI_VALUE;
    case USART_CLOCKSOURCE_CSI:
        return CSI_VALUE; // 4 MHz typically
    default:
        return 0;
    }
}

static HAL_Status usart_set_baudrate(USART_TypeDef *USARTx, uint32_t BaudRate)
{
    uint32_t clock_source = usart_get_clock_source(USARTx);

    uint32_t clock_freq = usart_get_clock_freq(clock_source);

    if (clock_freq == 0) {
        return HAL_ERROR;
    }

    USARTx->BRR = ((clock_freq + (BaudRate / 2U)) / BaudRate);
    return HAL_OK;
}

/*
 * UART INTERRUPT
 */

void usart_rxtx_interrupt_init(void)
{
    /* configure GPIO pin */
    // enable clock access
    RCC->AHB4ENR |= GPIODEN;

    /* TX Enable for USART3 GPIO */
    // set PD8 to alternate function
    GPIOD->MODER &= ~(1U << 16); // set bit 16 to 0
    GPIOD->MODER |= (1U << 17);  // set bit 17 to 1

    // set PD8 to alternate function type UART_TX AF7 (0111)
    GPIOD->AFR[1] &= ~(0xF << 0); // Clear bits first
    GPIOD->AFR[1] |= (7U << 0);   // Set AF7

    /* RX Enable for USART3 GPIO */
    // set PD9 to alternate function
    GPIOD->MODER &= ~(1U << 18); // set bit 18 to 0
    GPIOD->MODER |= (1U << 19);  // set bit 19 to 1

    // set PD9 to alternate function type UART_RX AF7 (0111)
    GPIOD->AFR[1] &= ~(0xF << 4); // Clear bits first
    GPIOD->AFR[1] |= (7U << 4);   // Set AF7

    /* configure UART module */
    // enable clock access to UART3
    RCC->APB1ENR1 |= USART3EN;

    // configure baudrate
    uart_set_baudrate(USART3, ABP1_CLK, UART_BAUDRATE);

    // configure transfer direction
    USART3->CR1 = CR1_FIFOEN_DISABLE | CR1_TE | CR1_RE;

    /* enable RXNE interrupt */
    USART3->CR1 |= CR1_RXNEIE;

    /* ENABLE USART3 interrupt in NVIC */
    __NVIC_EnableIRQ(USART3_IRQn);

    // enable uart module
    USART3->CR1 |= CR1_UE;

    for (int i = 0; i < 1000000; i++) {
    }
}

static void uart_callback(void)
{
    char key = USART3->RDR;

    if (key == '1') {
        led_toggle(1);
    } else {
        led_toggle(2);
    }
}

void USART3_IRQHandler(void)
{
    if (USART3->ISR & ISR_ALT_RXNE) {
        uart_callback();
    }
}
