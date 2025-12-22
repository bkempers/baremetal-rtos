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
static HAL_Status      usart_check_flag(USART_Handle *handle, USART_Flag flag, FlagStatus status, uint32_t tickstart, uint32_t timeout);
static void            usart_end_transfer(USART_Handle *handle);
static void            usart_end_transmit_it(USART_Handle *handle);

static void USART_TxISR_8FIFO(USART_Handle *handle);
static void USART_TxISR_8(USART_Handle *handle);

static void USART_RxISR_8FIFO(USART_Handle *handle);
static void USART_RxISR_8(USART_Handle *handle);

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
    uint8_t *rx_data_8b;
    uint16_t uhMask;
    uint32_t tickstart;

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
        // USART_MASK_COMPUTATION(handle);
        // uhMask = handle->Mask;
        rx_data_8b = (uint8_t *) rxPointer;

        /* as long as data have to be received */
        while (handle->rxCount > 0U) {
            if (usart_check_flag(handle, USART_FLAG_RXNE, RESET, tickstart, timeout) != HAL_OK) {
                return HAL_TIMEOUT;
            }
            *rx_data_8b = (uint8_t) (handle->Instance->RDR & (uint8_t) (uhMask & 0xFFU));
            rx_data_8b++;

            handle->rxCount--;
        }

        handle->State = HAL_USART_STATE_READY;

        return HAL_OK;
    } else {
        return HAL_BUSY;
    }
}

HAL_Status HAL_USART_Transmit(USART_Handle *handle, const uint8_t *txPointer, uint16_t size, uint32_t timeout)
{
    const uint8_t *tx_data_8b;
    uint32_t       tickstart;

    if (handle->State == HAL_USART_STATE_READY || handle->State == HAL_USART_STATE_BUSY_RX) {
        if ((txPointer == NULL) || (size == 0U)) {
            return HAL_ERROR;
        }

        handle->errorCode = USART_ERROR_NONE;
        if (handle->State == HAL_USART_STATE_READY) {
            handle->State = HAL_USART_STATE_BUSY_TX;
        } else if (handle->State == HAL_USART_STATE_BUSY_RX) {
            handle->State = HAL_USART_STATE_BUSY_TX_RX;
        }

        /* Init tickstart for timeout management */
        tickstart = HAL_GetTick();

        handle->txSize  = size;
        handle->txCount = size;

        tx_data_8b = txPointer;

        while (handle->txCount > 0U) {
            if (usart_check_flag(handle, USART_FLAG_TXE, SET, tickstart, timeout) != HAL_OK) {
                return HAL_TIMEOUT;
            }
            handle->Instance->TDR = (uint8_t) (*tx_data_8b & 0xFFU);

            tx_data_8b++;
            handle->txCount--;
        }

        if (usart_check_flag(handle, USART_FLAG_TC, SET, tickstart, timeout) != HAL_OK) {
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

    if (handle->State == HAL_USART_STATE_READY || handle->State == HAL_USART_STATE_BUSY_TX) {
        if ((rxPointer == NULL) || (size == 0U)) {
            return HAL_ERROR;
        }

        handle->rxPointer = rxPointer;
        handle->rxSize    = size;
        handle->rxCount   = size;
        handle->RxISR     = NULL;

        handle->errorCode = USART_ERROR_NONE;
        if (handle->State == HAL_USART_STATE_READY) {
            handle->State = HAL_USART_STATE_BUSY_RX;
        } else if (handle->State == HAL_USART_STATE_BUSY_TX) {
            handle->State = HAL_USART_STATE_BUSY_TX_RX;
        }

        /* Enable the USART Error Interrupt: (Frame error, noise error, overrun error) */
        SET_BIT(handle->Instance->CR3, USART_CR3_EIE);

        /* Configure Rx interrupt processing */
        if ((handle->fifoMode == USART_CR1_FIFOEN) && (size >= handle->rxProcessData)) {
            /* Set the Rx ISR function pointer according to the data word length */
            if ((handle->Init.WordLen == USART_WORDLENGTH_9B) && (handle->Init.Parity == USART_PARITY_NONE)) {
                // TODO: not supported
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
                // TODO: not supported
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
                // TODO: not supported
            } else {
                handle->TxISR = USART_TxISR_8FIFO;
            }

            /* Enable the TX FIFO threshold interrupt */
            usart_enable_interrupt(handle, USART_IT_TXFT);
        } else {
            /* Set the Tx ISR function pointer according to the data word length */
            if ((handle->Init.WordLen == USART_WORDLENGTH_9B) && (handle->Init.Parity == USART_PARITY_NONE)) {
                // TODO: not supported
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

HAL_Status HAL_USART_Receive_Transmit_IT(USART_Handle *handle, uint8_t *rxPointer, const uint8_t *txPointer, uint16_t size)
{
    if (handle->State == HAL_USART_STATE_READY) {
        if ((txPointer == NULL) || (rxPointer == NULL) || (size == 0U)) {
            return HAL_ERROR;
        }

        handle->rxPointer = rxPointer;
        handle->rxSize    = size;
        handle->rxCount   = size;
        handle->txPointer = txPointer;
        handle->txSize    = size;
        handle->txCount   = size;

        handle->errorCode = USART_ERROR_NONE;
        handle->State     = HAL_USART_STATE_BUSY_TX_RX;

        /* Configure TxRx interrupt processing */
        if ((handle->fifoMode == USART_CR1_FIFOEN) && (size >= handle->rxProcessData)) {
            /* Set the Rx ISR function pointer according to the data word length */
            if ((handle->Init.WordLen == USART_WORDLENGTH_9B) && (handle->Init.Parity == USART_PARITY_NONE)) {
                // TODO: not supported
            } else {
                handle->TxISR = USART_TxISR_8FIFO;
                handle->RxISR = USART_RxISR_8FIFO;
            }

            /* Enable the USART Error Interrupt: (Frame error, noise error, overrun error) */
            SET_BIT(handle->Instance->CR3, USART_CR3_EIE);

            if (handle->Init.Parity != USART_PARITY_NONE) {
                /* Enable the USART Parity Error interrupt  */
                SET_BIT(handle->Instance->CR1, USART_CR1_PEIE);
            }

            /* Enable the TX and  RX FIFO Threshold interrupts */
            SET_BIT(handle->Instance->CR3, (USART_CR3_TXFTIE | USART_CR3_RXFTIE));
        } else {
            if ((handle->Init.WordLen == USART_WORDLENGTH_9B) && (handle->Init.Parity == USART_PARITY_NONE)) {
                // TODO: not supported
            } else {
                handle->TxISR = USART_TxISR_8;
                handle->RxISR = USART_RxISR_8;
            }

            /* Enable the USART Error Interrupt: (Frame error, noise error, overrun error) */
            SET_BIT(handle->Instance->CR3, USART_CR3_EIE);

            /* Enable the USART Parity Error and USART Data Register not empty Interrupts */
            if (handle->Init.Parity != USART_PARITY_NONE) {
                SET_BIT(handle->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE_RXFNEIE);
            } else {
                SET_BIT(handle->Instance->CR1, USART_CR1_RXNEIE_RXFNEIE);
            }

            /* Enable the USART Transmit Data Register Empty Interrupt */
            SET_BIT(handle->Instance->CR1, USART_CR1_TXEIE_TXFNFIE);
        }

        return HAL_OK;
    } else {
        return HAL_BUSY;
    }
}

HAL_Status HAL_USART_Abort(USART_Handle *handle)
{
    // TODO: add in later
    return HAL_OK;
}

HAL_Status HAL_USART_Abort_IT(USART_Handle *handle)
{
    // TODO: add in later
    return HAL_OK;
}

void HAL_USART_IRQHandler(USART_Handle *handle)
{
    uint32_t isrflags = READ_REG(handle->Instance->ISR);
    uint32_t cr1its   = READ_REG(handle->Instance->CR1);
    uint32_t cr3its   = READ_REG(handle->Instance->CR3);

    uint32_t errorflags;
    uint32_t errorcode;

    /* If no error occurs */
    errorflags = (isrflags & (uint32_t) (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE | USART_ISR_RTOF | USART_ISR_UDR));
    if (errorflags == 0U) {
        /* USART in mode Receiver ---------------------------------------------------*/
        if (((isrflags & USART_ISR_RXNE_RXFNE) != 0U) && (((cr1its & USART_CR1_RXNEIE_RXFNEIE) != 0U) || ((cr3its & USART_CR3_RXFTIE) != 0U))) {
            if (handle->RxISR != NULL) {
                handle->RxISR(handle);
            }
            return;
        }
    }

    /* If some errors occur */
    if ((errorflags != 0U) &&
        (((cr3its & (USART_CR3_RXFTIE | USART_CR3_EIE)) != 0U) || ((cr1its & (USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE)) != 0U))) {
        /* USART parity error interrupt occurred -------------------------------------*/
        if (((isrflags & USART_ISR_PE) != 0U) && ((cr1its & USART_CR1_PEIE) != 0U)) {
            handle->Instance->ICR = (uint32_t) USART_CLEAR_PEF;
            handle->errorCode |= USART_ERROR_PE;
        }

        /* USART frame error interrupt occurred --------------------------------------*/
        if (((isrflags & USART_ISR_FE) != 0U) && ((cr3its & USART_CR3_EIE) != 0U)) {
            handle->Instance->ICR = (uint32_t) USART_CLEAR_FEF;
            handle->errorCode |= USART_ERROR_FE;
        }

        /* USART noise error interrupt occurred --------------------------------------*/
        if (((isrflags & USART_ISR_NE) != 0U) && ((cr3its & USART_CR3_EIE) != 0U)) {
            handle->Instance->ICR = (uint32_t) USART_CLEAR_NEF;
            handle->errorCode |= USART_ERROR_NE;
        }

        /* USART Over-Run interrupt occurred -----------------------------------------*/
        if (((isrflags & USART_ISR_ORE) != 0U) &&
            (((cr1its & USART_CR1_RXNEIE_RXFNEIE) != 0U) || ((cr3its & (USART_CR3_RXFTIE | USART_CR3_EIE)) != 0U))) {
            handle->Instance->ICR = (uint32_t) USART_CLEAR_OREF;
            handle->errorCode |= USART_ERROR_ORE;
        }

        /* USART Receiver Timeout interrupt occurred ---------------------------------*/
        if (((isrflags & USART_ISR_RTOF) != 0U) && ((cr1its & USART_CR1_RTOIE) != 0U)) {
            handle->Instance->ICR = (uint32_t) USART_CLEAR_RTOF;
            handle->errorCode |= USART_ERROR_RTO;
        }

        /* USART SPI slave underrun error interrupt occurred -------------------------*/
        if (((isrflags & USART_ISR_UDR) != 0U) && ((cr3its & USART_CR3_EIE) != 0U)) {
            /* Ignore SPI slave underrun errors when reception is going on */
            if (handle->State == HAL_USART_STATE_BUSY_RX) {
                handle->Instance->ICR = USART_CLEAR_UDRF;
                return;
            } else {
                handle->Instance->ICR = USART_CLEAR_UDRF;
                handle->errorCode |= USART_ERROR_UDR;
            }
        }

        /* Call USART Error Call back function if need be --------------------------*/
        if (handle->errorCode != USART_ERROR_NONE) {
            /* USART in mode Receiver ---------------------------------------------------*/
            if (((isrflags & USART_ISR_RXNE_RXFNE) != 0U) && (((cr1its & USART_CR1_RXNEIE_RXFNEIE) != 0U) || ((cr3its & USART_CR3_RXFTIE) != 0U))) {
                if (handle->RxISR != NULL) {
                    handle->RxISR(handle);
                }
            }

            /* If Overrun error occurs, or if any error occurs in DMA mode reception,
               consider error as blocking */
            errorcode = handle->errorCode & USART_ERROR_ORE;
            if ((HAL_IS_BIT_SET(handle->Instance->CR3, USART_CR3_DMAR)) || (errorcode != 0U)) {
                /* Blocking error : transfer is aborted
                   Set the USART state ready to be able to start again the process,
                   Disable Interrupts, and disable DMA requests, if ongoing */
                usart_end_transfer(handle);

                /* Call registered Error Callback */
                handle->errorCallback(handle);
            } else {
                /* Non Blocking error : transfer could go on.
                   Error is notified to user through user error callback */
                /* Call registered Error Callback */
                handle->errorCallback(handle);
                handle->errorCode = USART_ERROR_NONE;
            }
        }
        return;

    } /* End if some error occurs */

    /* USART in mode Transmitter ------------------------------------------------*/
    if (((isrflags & USART_ISR_TXE_TXFNF) != 0U) && (((cr1its & USART_CR1_TXEIE_TXFNFIE) != 0U) || ((cr3its & USART_CR3_TXFTIE) != 0U))) {
        if (handle->TxISR != NULL) {
            handle->TxISR(handle);
        }
        return;
    }

    /* USART in mode Transmitter (transmission end) -----------------------------*/
    if (((isrflags & USART_ISR_TC) != 0U) && ((cr1its & USART_CR1_TCIE) != 0U)) {
        usart_end_transmit_it(handle);
        return;
    }

    /* USART TX Fifo Empty occurred ----------------------------------------------*/
    if (((isrflags & USART_ISR_TXFE) != 0U) && ((cr1its & USART_CR1_TXFEIE) != 0U)) {
        /* Call registered Tx Fifo Empty Callback */
        handle->txFifoEmptyCallback(handle);
        return;
    }

    /* USART RX Fifo Full occurred ----------------------------------------------*/
    if (((isrflags & USART_ISR_RXFF) != 0U) && ((cr1its & USART_CR1_RXFFIE) != 0U)) {
        /* Call registered Rx Fifo Full Callback */
        handle->rxFifoFullCallback(handle);
        return;
    }
}

__weak void HAL_USART_txHalfCpltCallback(USART_Handle *handle)
{
    UNUSED(handle);
}

__weak void HAL_USART_txCpltCallback(USART_Handle *handle)
{
    UNUSED(handle);
}

__weak void HAL_USART_rxCpltCallback(USART_Handle *handle)
{
    UNUSED(handle);
}

__weak void HAL_USART_rxHalfCpltCallback(USART_Handle *handle)
{
    UNUSED(handle);
}

__weak void HAL_USART_txrxCpltCallback(USART_Handle *handle)
{
    UNUSED(handle);
}

__weak void HAL_USART_errorCallback(USART_Handle *handle)
{
    UNUSED(handle);
}

__weak void HAL_USART_abortCpltCallback(USART_Handle *handle)
{
    UNUSED(handle);
}

static void usart_end_transfer(USART_Handle *handle)
{
    /* Disable TXEIE, TCIE, RXNE, RXFT, TXFT, PE and ERR (Frame error, noise error, overrun error) interrupts */
    CLEAR_BIT(handle->Instance->CR1, (USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE | USART_CR1_TXEIE_TXFNFIE | USART_CR1_TCIE));
    CLEAR_BIT(handle->Instance->CR3, (USART_CR3_EIE | USART_CR3_RXFTIE | USART_CR3_TXFTIE));

    /* At end of process, restore husart->State to Ready */
    handle->State = HAL_USART_STATE_READY;
}

static void usart_end_transmit_it(USART_Handle *handle)
{
    /* Disable the USART Transmit Complete Interrupt */
    usart_disable_interrupt(handle, USART_IT_TC);

    /* Disable the USART Error Interrupt: (Frame error, noise error, overrun error) */
    usart_disable_interrupt(handle, USART_IT_ERR);

    /* Clear TxISR function pointer */
    handle->TxISR = NULL;

    if (handle->State == HAL_USART_STATE_BUSY_TX) {
        /* Clear overrun flag and discard the received data */
        handle->Instance->ICR = USART_CLEAR_OREF;
        handle->Instance->RQR |= (uint16_t) USART_RQR_RXFRQ;

        /* Tx process is completed, restore husart->State to Ready */
        handle->State = HAL_USART_STATE_READY;

        /* Call registered Tx Complete Callback */
        handle->txCpltCallback(handle);
    } else if (handle->rxCount == 0U) {
        /* TxRx process is completed, restore husart->State to Ready */
        handle->State = HAL_USART_STATE_READY;

        /* Call registered Tx Rx Complete Callback */
        handle->txrxCpltCallback(handle);
    } else {
        /* Nothing to do */
    }
}

static void USART_RxISR_8(USART_Handle *handle)
{
    const HAL_USART_State state = handle->State;
    uint16_t              txdatacount;
    uint16_t              uhMask = handle->mask;
    uint32_t              txftie;

    if ((state == HAL_USART_STATE_BUSY_RX) || (state == HAL_USART_STATE_BUSY_TX_RX)) {
        *handle->rxPointer = (uint8_t) (handle->Instance->RDR & (uint8_t) uhMask);
        handle->rxPointer++;
        handle->rxCount--;

        if (handle->rxCount == 0U) {
            /* Disable the USART Parity Error Interrupt and RXNE interrupt*/
            CLEAR_BIT(handle->Instance->CR1, (USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE));

            /* Disable the USART Error Interrupt: (Frame error, noise error, overrun error) */
            CLEAR_BIT(handle->Instance->CR3, USART_CR3_EIE);

            /* Clear RxISR function pointer */
            handle->RxISR = NULL;

            /* txftie and txdatacount are temporary variables for MISRAC2012-Rule-13.5 */
            txftie      = READ_BIT(handle->Instance->CR3, USART_CR3_TXFTIE);
            txdatacount = handle->txCount;

            if (state == HAL_USART_STATE_BUSY_RX) {
                /* Call register Rx Complete Callback */
                handle->rxCpltCallback(handle);
            } else if ((READ_BIT(handle->Instance->CR1, USART_CR1_TCIE) != USART_CR1_TCIE) && (txftie != USART_CR3_TXFTIE) && (txdatacount == 0U)) {
                /* TxRx process is completed, restore handle->State to Ready */
                handle->State = HAL_USART_STATE_READY;

                /* Call registered Tx Rx Complete Callback */
                handle->txrxCpltCallback(handle);
            } else {
                /* Nothing to do */
            }
        } else {
            /* Nothing to do */
        }
    }
}

static void USART_RxISR_8FIFO(USART_Handle *handle)
{
    HAL_USART_State state = handle->State;
    uint16_t        txdatacount;
    uint16_t        rxdatacount;
    uint16_t        uhMask = handle->mask;
    uint16_t        nb_rx_data;
    uint32_t        txftie;

    /* Check that a Rx process is ongoing */
    if ((state == HAL_USART_STATE_BUSY_RX) || (state == HAL_USART_STATE_BUSY_TX_RX)) {
        for (nb_rx_data = handle->rxProcessData; nb_rx_data > 0U; nb_rx_data--) {
            if (((handle->Instance->ISR & (USART_FLAG_RXFNE)) == (USART_FLAG_RXFNE)) == SET) {
                *handle->rxPointer = (uint8_t) (handle->Instance->RDR & (uint8_t) (uhMask & 0xFFU));
                handle->rxPointer++;
                handle->rxCount--;

                if (handle->txCount == 0U) {
                    /* Disable the USART Parity Error Interrupt */
                    CLEAR_BIT(handle->Instance->CR1, USART_CR1_PEIE);

                    /* Disable the USART Error Interrupt: (Frame error, noise error, overrun error)
                       and RX FIFO Threshold interrupt */
                    CLEAR_BIT(handle->Instance->CR3, (USART_CR3_EIE | USART_CR3_RXFTIE));

                    /* Clear RxISR function pointer */
                    handle->RxISR = NULL;

                    /* txftie and txdatacount are temporary variables for MISRAC2012-Rule-13.5 */
                    txftie      = READ_BIT(handle->Instance->CR3, USART_CR3_TXFTIE);
                    txdatacount = handle->txCount;

                    if (state == HAL_USART_STATE_BUSY_RX) {
                        /* Rx process is completed, restore handle->State to Ready */
                        handle->State = HAL_USART_STATE_READY;
                        state         = HAL_USART_STATE_READY;

                        /* Call registered Rx Complete Callback */
                        handle->rxCpltCallback(handle);
                    } else if ((READ_BIT(handle->Instance->CR1, USART_CR1_TCIE) != USART_CR1_TCIE) && (txftie != USART_CR3_TXFTIE) &&
                               (txdatacount == 0U)) {
                        /* TxRx process is completed, restore handle->State to Ready */
                        handle->State = HAL_USART_STATE_READY;
                        state         = HAL_USART_STATE_READY;

                        /* Call registered Tx Rx Complete Callback */
                        handle->txrxCpltCallback(handle);
                    } else {
                        /* Nothing to do */
                    }
                } else {
                    /* Nothing to do */
                }
            }
        }

        /* When remaining number of bytes to receive is less than the RX FIFO
        threshold, next incoming frames are processed as if FIFO mode was
        disabled (i.e. one interrupt per received frame).
        */
        rxdatacount = handle->rxCount;
        if (((rxdatacount != 0U)) && (rxdatacount < handle->rxProcessData)) {
            /* Disable the USART RXFT interrupt*/
            CLEAR_BIT(handle->Instance->CR3, USART_CR3_RXFTIE);

            /* Update the RxISR function pointer */
            handle->RxISR = USART_RxISR_8;

            /* Enable the USART Data Register Not Empty interrupt */
            SET_BIT(handle->Instance->CR1, USART_CR1_RXNEIE_RXFNEIE);
        }
    } else {
        /* Clear RXNE interrupt flag */
        handle->Instance->RDR |= (uint16_t) USART_RQR_RXFRQ;
    }
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

HAL_USART_State HAL_USART_GetState(const USART_Handle *handle)
{
  return handle->State;
}

void usart_enable_interrupt(USART_Handle *handle, uint32_t interrupt)
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

void usart_disable_interrupt(USART_Handle *handle, uint32_t interrupt)
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
    while ((((handle->Instance->ISR & flag) == flag) ? SET : RESET) != status) {
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
        if (usart_check_flag(handle, USART_FLAG_TEACK, SET, tickstart, USART_TX_RX_TIMEOUT) != HAL_OK) {
            return HAL_TIMEOUT;
        }
    }

    if ((handle->Instance->CR1 & USART_CR1_RE) == USART_CR1_RE) {
        /* Wait until REACK flag is set */
        if (usart_check_flag(handle, USART_FLAG_REACK, SET, tickstart, USART_TX_RX_TIMEOUT) != HAL_OK) {
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
    // uint32_t cr1_config = (uint32_t) handle->Init.WordLen | handle->Init.Parity | handle->Init.Mode | USART_CR1_OVER8;
    uint32_t cr1_config = (uint32_t) handle->Init.WordLen | handle->Init.Parity | handle->Init.Mode;
    MODIFY_REG(handle->Instance->CR1, USART_CR1_CLEARMASK, cr1_config);

    // USART CR2 CONFIGURATION
    // Fixed for async UART:
    uint32_t cr2_config = (uint32_t) handle->Init.StopBits;
    // uint32_t cr2_config = (uint32_t) USART_CR2_CLKEN;
    // cr2_config |= handle->Init.CLKLastBit | handle->Init.CLKPolarity | handle->Init.CLKPhase | handle->Init.StopBits;
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
    case USART_CLOCKSOURCE_HSI: {
        // // Check if HSI divider is enabled (bit in RCC_CR)
        // if (RCC->CR & RCC_CR_HSIDIV) {
        //     uint32_t divider = (RCC->CR & RCC_CR_HSIDIV_Msk) >> RCC_CR_HSIDIV_Pos;
        //     return HSI_VALUE >> divider;
        // }
        // return HSI_VALUE;
        uint32_t hsi_freq = HSI_VALUE; // Start with 64 MHz

        // Check if HSIDIV is enabled AND get divider value
        if (RCC->CR & RCC_CR_HSIDIVF) {
            // Get divider value from bits [4:3]
            uint32_t hsidiv = (RCC->CR >> 3) & 0x3;

            // 00 = /1, 01 = /2, 10 = /4, 11 = /8
            const uint8_t div_table[4] = {1, 2, 4, 8};
            hsi_freq /= div_table[hsidiv];
        }

        return hsi_freq;
    }
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
