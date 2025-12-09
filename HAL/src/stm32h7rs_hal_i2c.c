#include "stm32h7rs_hal_i2c.h"
#include "stm32h7s3xx.h"

#define I2C_NO_OPTION_FRAME (0xFFFF0000U)

#define MAX_NBYTE_SIZE 255U

/* Private define to centralize the enable/disable of Interrupts */
#define I2C_XFER_TX_IT     (uint16_t) (0x0001U) /*!< Bit field can be combinated with @ref I2C_XFER_LISTEN_IT */
#define I2C_XFER_RX_IT     (uint16_t) (0x0002U) /*!< Bit field can be combinated with @ref I2C_XFER_LISTEN_IT */
#define I2C_XFER_LISTEN_IT (uint16_t) (0x8000U) /*!< Bit field can be combinated with @ref I2C_XFER_TX_IT and @ref I2C_XFER_RX_IT */
#define I2C_XFER_ERROR_IT  (uint16_t) (0x0010U) /*!< Bit definition to manage addition of global Error and NACK treatment */
#define I2C_XFER_CPLT_IT   (uint16_t) (0x0020U) /*!< Bit definition to manage only STOP evenement */
#define I2C_XFER_RELOAD_IT (uint16_t) (0x0040U) /*!< Bit definition to manage only Reload of NBYTE */

static HAL_Status I2C_WaitOnFlagUntilTimeout(I2C_Handle *handle, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart);
static HAL_Status I2C_IsErrorOccurred(I2C_Handle *handle, uint32_t Timeout, uint32_t Tickstart);
static void       I2C_Flush_TXDR(I2C_Handle *handle);

static HAL_Status I2C_Master_ISR_IT(struct __I2C_Handle *handle, uint32_t ITFlags, uint32_t ITSources);
static HAL_Status I2C_Mem_ISR_IT(struct __I2C_Handle *handle, uint32_t ITFlags, uint32_t ITSources);

// static void I2C_ITAddrCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags);
static void I2C_ITMasterSeqCplt(I2C_Handle *handle);
// static void I2C_ITSlaveSeqCplt(I2C_HandleTypeDef *hi2c);
static void I2C_ITMasterCplt(I2C_Handle *handle, uint32_t ITFlags);
// static void I2C_ITSlaveCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags);
// static void I2C_ITListenCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags);
static void I2C_ITError(I2C_Handle *handle, uint32_t ErrorCode);

static void I2C_Enable_IRQ(I2C_Handle *handle, uint16_t InterruptRequest);
static void I2C_Disable_IRQ(I2C_Handle *handle, uint16_t InterruptRequest);
static void I2C_TransferConfig(I2C_Handle *handle, uint16_t DevAddress, uint8_t Size, uint32_t Mode, uint32_t Request);

HAL_Status HAL_I2C_Init(I2C_Handle *handle)
{
    if (handle == NULL) {
        return HAL_ERROR;
    }

    handle->State = HAL_I2C_STATE_BUSY;

    CLEAR_BIT(handle->Instance->CR1, I2C_CR1_PE);

    handle->Instance->TIMINGR = handle->Init.Timing & TIMING_CLEAR_MASK;

    handle->Instance->OAR1 &= ~I2C_OAR1_OA1EN;
    if (handle->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT) {
        handle->Instance->OAR1 = (I2C_OAR1_OA1EN | handle->Init.OwnAddress1);
    } else /* I2C_ADDRESSINGMODE_10BIT */ {
        handle->Instance->OAR1 = (I2C_OAR1_OA1EN | I2C_OAR1_OA1MODE | handle->Init.OwnAddress1);
    }

    if (handle->Init.AddressingMode == I2C_ADDRESSINGMODE_10BIT) {
        SET_BIT(handle->Instance->CR2, I2C_CR2_ADD10);
    } else {
        CLEAR_BIT(handle->Instance->CR2, I2C_CR2_ADD10);
    }

    handle->Instance->CR2 |= (I2C_CR2_AUTOEND | I2C_CR2_NACK);
    handle->Instance->OAR2 &= ~I2C_OAR2_OA2EN;
    handle->Instance->OAR2 = (handle->Init.DualAddressMode | handle->Init.OwnAddress2 | (handle->Init.OwnAddress2Masks << 8));
    handle->Instance->CR1  = (handle->Init.GeneralCallMode | handle->Init.NoStretchMode);

    SET_BIT(handle->Instance->CR1, I2C_CR1_PE);

    handle->ErrorCode     = HAL_I2C_ERROR_NONE;
    handle->State         = HAL_I2C_STATE_READY;
    handle->PreviousState = (uint32_t) HAL_I2C_MODE_NONE;
    handle->Mode          = HAL_I2C_MODE_NONE;

    return HAL_OK;
}

HAL_Status HAL_I2C_DeInit(I2C_Handle *handle)
{
    // TODO: implement later
    return HAL_OK;
}

HAL_Status HAL_I2C_IsDeviceReady(I2C_Handle *handle, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout)
{
    uint32_t tickstart;

    __IO uint32_t I2C_Trials = 0UL;

    HAL_Status status = HAL_OK;

    FlagStatus tmp1;
    FlagStatus tmp2;

    if (handle->State == HAL_I2C_STATE_READY) {
        if (((((handle->Instance->ISR) & I2C_ISR_BUSY) == I2C_ISR_BUSY) ? SET : RESET) == SET) {
            return HAL_BUSY;
        }

        handle->State     = HAL_I2C_STATE_BUSY;
        handle->ErrorCode = HAL_I2C_ERROR_NONE;

        do {
            /* Generate Start */
            handle->Instance->CR2 = I2C_GENERATE_START(handle->Init.AddressingMode, DevAddress);

            /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
            /* Wait until STOPF flag is set or a NACK flag is set*/
            tickstart = HAL_GetTick();

            tmp1 = (handle->Instance->ISR & (I2C_ISR_STOPF)) == I2C_ISR_STOPF ? SET : RESET;
            tmp2 = (handle->Instance->ISR & (I2C_ISR_NACKF)) == I2C_ISR_NACKF ? SET : RESET;

            while ((tmp1 == RESET) && (tmp2 == RESET)) {
                if (Timeout != HAL_MAX_DELAY) {
                    if (((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U)) {
                        /* Update I2C state */
                        handle->State = HAL_I2C_STATE_READY;

                        /* Update I2C error code */
                        handle->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;

                        return HAL_ERROR;
                    }
                }

                tmp1 = (handle->Instance->ISR & (I2C_ISR_STOPF)) == I2C_ISR_STOPF ? SET : RESET;
                tmp2 = (handle->Instance->ISR & (I2C_ISR_NACKF)) == I2C_ISR_NACKF ? SET : RESET;
            }

            /* Check if the NACKF flag has not been set */
            if (((handle->Instance->ISR & (I2C_ISR_NACKF)) == I2C_ISR_NACKF ? SET : RESET) == RESET) {
                /* Wait until STOPF flag is reset */
                if (I2C_WaitOnFlagUntilTimeout(handle, I2C_ISR_STOPF, RESET, Timeout, tickstart) != HAL_OK) {
                    /* A non acknowledge appear during STOP Flag waiting process, a new trial must be performed */
                    if (handle->ErrorCode == HAL_I2C_ERROR_AF) {
                        /* Clear STOP Flag */
                        handle->Instance->ICR = I2C_ISR_STOPF;

                        /* Reset the error code for next trial */
                        handle->ErrorCode = HAL_I2C_ERROR_NONE;
                    } else {
                        status = HAL_ERROR;
                    }
                } else {
                    /* A acknowledge appear during STOP Flag waiting process, this mean that device respond to its address */

                    /* Clear STOP Flag */
                    handle->Instance->ICR = I2C_ISR_STOPF;

                    /* Device is ready */
                    handle->State = HAL_I2C_STATE_READY;
                    return HAL_OK;
                }
            } else {
                /* A non acknowledge is detected, this mean that device not respond to its address,
                   a new trial must be performed */

                /* Clear NACK Flag */
                handle->Instance->ICR = I2C_ISR_NACKF;

                /* Wait until STOPF flag is reset */
                if (I2C_WaitOnFlagUntilTimeout(handle, I2C_ISR_STOPF, RESET, Timeout, tickstart) != HAL_OK) {
                    status = HAL_ERROR;
                } else {
                    /* Clear STOP Flag, auto generated with autoend*/
                    handle->Instance->ICR = I2C_ISR_STOPF;
                }
            }

            /* Increment Trials */
            I2C_Trials++;

            if ((I2C_Trials < Trials) && (status == HAL_ERROR)) {
                status = HAL_OK;
            }

        } while (I2C_Trials < Trials);

        /* Update I2C state */
        handle->State = HAL_I2C_STATE_READY;

        /* Update I2C error code */
        handle->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
        return HAL_ERROR;
    } else {
        return HAL_BUSY;
    }
}

static HAL_Status I2C_WaitOnFlagUntilTimeout(I2C_Handle *handle, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart)
{
    while (((((handle->Instance->ISR) & Flag) == Flag) ? SET : RESET) == Status) {
        /* Check if an error is detected */
        if (I2C_IsErrorOccurred(handle, Timeout, Tickstart) != HAL_OK) {
            return HAL_ERROR;
        }

        /* Check for the Timeout */
        if (Timeout != HAL_MAX_DELAY) {
            if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U)) {
                if (((((handle->Instance->ISR) & Flag) == Flag) ? SET : RESET) == Status) {
                    handle->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
                    handle->State = HAL_I2C_STATE_READY;
                    handle->Mode  = HAL_I2C_MODE_NONE;
                    return HAL_ERROR;
                }
            }
        }
    }
    return HAL_OK;
}

static HAL_Status I2C_IsErrorOccurred(I2C_Handle *handle, uint32_t Timeout, uint32_t Tickstart)
{
    HAL_Status   status     = HAL_OK;
    uint32_t     itflag     = handle->Instance->ISR;
    uint32_t     error_code = 0;
    uint32_t     tickstart  = Tickstart;
    uint32_t     tmp1;
    HAL_I2C_Mode tmp2;

    if (HAL_IS_BIT_SET(itflag, I2C_ISR_NACKF)) {
        /* Clear NACKF Flag */
        handle->Instance->ICR = I2C_ISR_NACKF;

        /* Wait until STOP Flag is set or timeout occurred */
        /* AutoEnd should be initiate after AF */
        while ((((((handle->Instance->ISR) & I2C_ISR_STOPF) == I2C_ISR_STOPF) ? SET : RESET) == RESET) && (status == HAL_OK)) {
            /* Check for the Timeout */
            if (Timeout != HAL_MAX_DELAY) {
                if (((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U)) {
                    tmp1 = (uint32_t) (handle->Instance->CR2 & I2C_CR2_STOP);
                    tmp2 = handle->Mode;

                    /* In case of I2C still busy, try to regenerate a STOP manually */
                    if ((((((handle->Instance->ISR) & I2C_ISR_BUSY) == I2C_ISR_BUSY) ? SET : RESET) != RESET) && (tmp1 != I2C_CR2_STOP) &&
                        (tmp2 != HAL_I2C_MODE_SLAVE)) {
                        /* Generate Stop */
                        handle->Instance->CR2 |= I2C_CR2_STOP;

                        /* Update Tick with new reference */
                        tickstart = HAL_GetTick();
                    }

                    while (((((handle->Instance->ISR) & I2C_ISR_STOPF) == I2C_ISR_STOPF) ? SET : RESET) == RESET) {
                        /* Check for the Timeout */
                        if ((HAL_GetTick() - tickstart) > I2C_TIMEOUT_STOPF) {
                            error_code |= HAL_I2C_ERROR_TIMEOUT;
                            status = HAL_ERROR;
                            break;
                        }
                    }
                }
            }
        }

        /* In case STOP Flag is detected, clear it */
        if (status == HAL_OK) {
            /* Clear STOP Flag */
            handle->Instance->ICR = I2C_ISR_STOPF;
        }
        error_code |= HAL_I2C_ERROR_AF;
        status = HAL_ERROR;
    }

    /* Refresh Content of Status register */
    itflag = handle->Instance->ISR;

    /* Then verify if an additional errors occurs */
    /* Check if a Bus error occurred */
    if (HAL_IS_BIT_SET(itflag, I2C_ISR_BERR)) {
        error_code |= HAL_I2C_ERROR_BERR;

        /* Clear BERR flag */
        handle->Instance->ICR = I2C_ISR_BERR;
        status                = HAL_ERROR;
    }

    /* Check if an Over-Run/Under-Run error occurred */
    if (HAL_IS_BIT_SET(itflag, I2C_ISR_OVR)) {
        error_code |= HAL_I2C_ERROR_OVR;

        /* Clear OVR flag */
        handle->Instance->ICR = I2C_ISR_OVR;
        status                = HAL_ERROR;
    }

    /* Check if an Arbitration Loss error occurred */
    if (HAL_IS_BIT_SET(itflag, I2C_ISR_ARLO)) {
        error_code |= HAL_I2C_ERROR_ARLO;

        /* Clear ARLO flag */
        handle->Instance->ICR = I2C_ISR_ARLO;
        status                = HAL_ERROR;
    }

    if (status != HAL_OK) {
        /* Flush TX register */
        I2C_Flush_TXDR(handle);

        /* Clear Configuration Register 2 */
        handle->Instance->CR2 &= (uint32_t) ~((uint32_t) (I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN));

        handle->ErrorCode |= error_code;
        handle->State = HAL_I2C_STATE_READY;
        handle->Mode  = HAL_I2C_MODE_NONE;
    }

    return status;
}

static void I2C_Flush_TXDR(I2C_Handle *handle)
{
    /* If a pending TXIS flag is set */
    /* Write a dummy data in TXDR to clear it */
    if (((((handle->Instance->ISR) & I2C_ISR_TXIS) == I2C_ISR_TXIS) ? SET : RESET) != RESET) {
        handle->Instance->TXDR = 0x00U;
    }

    /* Flush TX register if not empty */
    if (((((handle->Instance->ISR) & I2C_ISR_TXE) == I2C_ISR_TXE) ? SET : RESET) == RESET) {
        handle->Instance->ISR |= I2C_ISR_TXE;
    }
}

HAL_Status HAL_I2C_Master_TX_IT(I2C_Handle *handle, uint32_t DevAddress, uint8_t *ptrData, uint16_t Size)
{
    uint32_t tx_mode;

    if (handle->State == HAL_I2C_STATE_READY) {
        if (((((handle->Instance->ISR) & I2C_ISR_BUSY) == I2C_ISR_BUSY) ? SET : RESET) == SET) {
            return HAL_BUSY;
        }

        handle->State     = HAL_I2C_STATE_BUSY_TX;
        handle->Mode      = HAL_I2C_MODE_MASTER;
        handle->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        handle->buffPtr     = ptrData;
        handle->xferCount   = Size;
        handle->xferOptions = I2C_NO_OPTION_FRAME;
        handle->xferISR     = I2C_Master_ISR_IT;

        if (handle->xferCount > MAX_NBYTE_SIZE) {
            handle->xferSize = MAX_NBYTE_SIZE;
            tx_mode          = I2C_RELOAD_MODE;
        } else {
            handle->xferSize = handle->xferCount;
            tx_mode          = I2C_AUTOEND_MODE;
        }

        /* Send Slave Address */
        /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
        I2C_TransferConfig(handle, DevAddress, (uint8_t) handle->xferSize, tx_mode, I2C_GENERATE_START_WRITE);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                  to avoid the risk of I2C interrupt handle execution before current
                  process unlock */

        /* Enable ERR, TC, STOP, NACK, TXI interrupt */
        /* possible to enable all of these */
        /* I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI |
          I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
        I2C_Enable_IRQ(handle, I2C_XFER_TX_IT);

        return HAL_OK;
    } else {
        return HAL_BUSY;
    }
}

HAL_Status HAL_I2C_Master_RX_IT(I2C_Handle *handle, uint16_t DevAddress, uint8_t *ptrData, uint16_t Size)
{
    uint32_t rx_mode;

    if (handle->State == HAL_I2C_STATE_READY) {
        if (((((handle->Instance->ISR) & I2C_ISR_BUSY) == I2C_ISR_BUSY) ? SET : RESET) == SET) {
            return HAL_BUSY;
        }

        handle->State     = HAL_I2C_STATE_BUSY_RX;
        handle->Mode      = HAL_I2C_MODE_MASTER;
        handle->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        handle->buffPtr     = ptrData;
        handle->xferCount   = Size;
        handle->xferOptions = I2C_NO_OPTION_FRAME;
        handle->xferISR     = I2C_Master_ISR_IT;

        if (handle->xferCount > MAX_NBYTE_SIZE) {
            handle->xferSize = MAX_NBYTE_SIZE;
            rx_mode          = I2C_RELOAD_MODE;
        } else {
            handle->xferSize = handle->xferCount;
            rx_mode          = I2C_AUTOEND_MODE;
        }

        /* Send Slave Address */
        /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
        I2C_TransferConfig(handle, DevAddress, (uint8_t) handle->xferSize, rx_mode, I2C_GENERATE_START_READ);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                  to avoid the risk of I2C interrupt handle execution before current
                  process unlock */

        /* Enable ERR, TC, STOP, NACK, RXI interrupt */
        /* possible to enable all of these */
        /* I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI |
          I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
        I2C_Enable_IRQ(handle, I2C_XFER_RX_IT);

        return HAL_OK;
    } else {
        return HAL_BUSY;
    }
}

static void I2C_TransferConfig(I2C_Handle *handle, uint16_t DevAddress, uint8_t Size, uint32_t Mode, uint32_t Request)
{
    uint32_t tmp;

    /* Declaration of tmp to prevent undefined behavior of volatile usage */
    tmp = ((uint32_t) (((uint32_t) DevAddress & I2C_CR2_SADD) | (((uint32_t) Size << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES) | (uint32_t) Mode |
                       (uint32_t) Request) &
           (~0x80000000U));

    /* update CR2 register */
    MODIFY_REG(handle->Instance->CR2,
               ((I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND |
                 (I2C_CR2_RD_WRN & (uint32_t) (Request >> (31U - I2C_CR2_RD_WRN_Pos))) | I2C_CR2_START | I2C_CR2_STOP)),
               tmp);
}

static HAL_Status I2C_Master_ISR_IT(struct __I2C_Handle *handle, uint32_t ITFlags, uint32_t ITSources)
{
    uint16_t devaddress;
    uint32_t tmpITFlags = ITFlags;

    if ((I2C_CHECK_FLAG(tmpITFlags, I2C_ISR_NACKF) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_CR1_NACKIE) != RESET)) {
        /* Clear NACK Flag */
        handle->Instance->ISR = I2C_ISR_NACKF;

        /* Set corresponding Error Code */
        /* No need to generate STOP, it is automatically done */
        /* Error callback will be send during stop flag treatment */
        handle->ErrorCode |= HAL_I2C_ERROR_AF;

        /* Flush TX register */
        I2C_Flush_TXDR(handle);
    } else if ((I2C_CHECK_FLAG(tmpITFlags, I2C_ISR_RXNE) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_CR1_RXIE) != RESET)) {
        /* Remove RXNE flag on temporary variable as read done */
        tmpITFlags &= ~I2C_ISR_RXNE;

        /* Read data from RXDR */
        *handle->buffPtr = (uint8_t) handle->Instance->RXDR;

        /* Increment Buffer pointer */
        handle->buffPtr++;
        handle->xferSize--;
        handle->xferCount--;
    } else if ((I2C_CHECK_FLAG(tmpITFlags, I2C_ISR_TXIS) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_CR1_TXIE) != RESET)) {
        /* Write data to TXDR */
        handle->Instance->TXDR = *handle->buffPtr;

        /* Increment Buffer pointer */
        handle->buffPtr++;
        handle->xferSize--;
        handle->xferCount--;
    } else if ((I2C_CHECK_FLAG(tmpITFlags, I2C_ISR_TCR) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_CR1_TCIE) != RESET)) {
        if ((handle->xferCount != 0U) && (handle->xferSize == 0U)) {
            devaddress = (uint16_t) (handle->Instance->CR2 & I2C_CR2_SADD);

            if (handle->xferCount > MAX_NBYTE_SIZE) {
                handle->xferSize = MAX_NBYTE_SIZE;
                I2C_TransferConfig(handle, devaddress, (uint8_t) handle->xferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
            } else {
                handle->xferSize = handle->xferCount;
                if (handle->xferOptions != I2C_NO_OPTION_FRAME) {
                    I2C_TransferConfig(handle, devaddress, (uint8_t) handle->xferSize, handle->xferOptions, I2C_NO_STARTSTOP);
                } else {
                    I2C_TransferConfig(handle, devaddress, (uint8_t) handle->xferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
                }
            }
        } else {
            /* Call TxCpltCallback() if no stop mode is set */
            if ((handle->Instance->CR2 & I2C_CR2_AUTOEND) != I2C_AUTOEND_MODE) {
                /* Call I2C Master Sequential complete process */
                I2C_ITMasterSeqCplt(handle);
            } else {
                /* Wrong size Status regarding TCR flag event */
                /* Call the corresponding callback to inform upper layer of End of Transfer */
                I2C_ITError(handle, HAL_I2C_ERROR_SIZE);
            }
        }
    } else if ((I2C_CHECK_FLAG(tmpITFlags, I2C_ISR_TC) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_CR1_TCIE) != RESET)) {
        if (handle->xferCount == 0U) {
            if ((handle->Instance->CR2 & I2C_CR2_AUTOEND) != I2C_AUTOEND_MODE) {
                /* Generate a stop condition in case of no transfer option */
                if (handle->xferOptions == I2C_NO_OPTION_FRAME) {
                    /* Generate Stop */
                    handle->Instance->CR2 |= I2C_CR2_STOP;
                } else {
                    /* Call I2C Master Sequential complete process */
                    I2C_ITMasterSeqCplt(handle);
                }
            }
        } else {
            /* Wrong size Status regarding TC flag event */
            /* Call the corresponding callback to inform upper layer of End of Transfer */
            I2C_ITError(handle, HAL_I2C_ERROR_SIZE);
        }
    } else {
        /* Nothing to do */
    }

    if ((I2C_CHECK_FLAG(tmpITFlags, I2C_ISR_STOPF) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_CR1_STOPIE) != RESET)) {
        /* Call I2C Master complete process */
        I2C_ITMasterCplt(handle, tmpITFlags);
    }

    return HAL_OK;
}

static void I2C_Enable_IRQ(I2C_Handle *handle, uint16_t InterruptRequest)
{
    uint32_t tmpisr = 0U;

    if ((InterruptRequest & I2C_XFER_LISTEN_IT) == I2C_XFER_LISTEN_IT) {
        /* Enable ERR, STOP, NACK and ADDR interrupts */
        tmpisr |= I2C_CR1_ADDRIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_ERRIE;
    }

    if ((InterruptRequest & I2C_XFER_TX_IT) == I2C_XFER_TX_IT) {
        /* Enable ERR, TC, STOP, NACK and TXI interrupts */
        tmpisr |= I2C_CR1_ERRIE | I2C_CR1_TCIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_TXIE;
    }

    if ((InterruptRequest & I2C_XFER_RX_IT) == I2C_XFER_RX_IT) {
        /* Enable ERR, TC, STOP, NACK and RXI interrupts */
        tmpisr |= I2C_CR1_ERRIE | I2C_CR1_TCIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_RXIE;
    }

    if (InterruptRequest == I2C_XFER_ERROR_IT) {
        /* Enable ERR and NACK interrupts */
        tmpisr |= I2C_CR1_ERRIE | I2C_CR1_NACKIE;
    }

    if (InterruptRequest == I2C_XFER_CPLT_IT) {
        /* Enable STOP interrupts */
        tmpisr |= I2C_CR1_STOPIE;
    }

    /* Enable interrupts only at the end */
    /* to avoid the risk of I2C interrupt handle execution before */
    /* all interrupts requested done */
    handle->Instance->ISR |= tmpisr;
}

static void I2C_Disable_IRQ(I2C_Handle *handle, uint16_t InterruptRequest)
{
    // TODO: implement later
}

// HAL_Status HAL_I2C_Slave_TX_IT(I2C_Handle *handle, uint8_t *ptrData, uint16_t Size);
// HAL_Status HAL_I2C_Slave_RX_IT(I2C_Handle *handle, uint8_t *ptrData, uint16_t Size);

HAL_Status HAL_I2C_Mem_Write_IT(I2C_Handle *handle, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *ptrData, uint16_t Size)
{
    if (handle->State == HAL_I2C_STATE_READY) {
        if ((ptrData == NULL) || (Size == 0U)) {
            handle->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
            return HAL_ERROR;
        }

        if (((((handle->Instance->ISR) & I2C_ISR_BUSY) == I2C_ISR_BUSY) ? SET : RESET) == SET) {
            return HAL_BUSY;
        }

        handle->State     = HAL_I2C_STATE_BUSY_TX;
        handle->Mode      = HAL_I2C_MODE_MEM;
        handle->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        handle->xferSize    = 0U;
        handle->buffPtr     = ptrData;
        handle->xferCount   = Size;
        handle->xferOptions = I2C_NO_OPTION_FRAME;
        handle->xferISR     = I2C_Mem_ISR_IT;
        handle->DevAddress  = DevAddress;

        /* If Memory address size is 8Bit */
        if (MemAddSize == I2C_MEMADD_SIZE_8BIT) {
            /* Prefetch Memory Address */
            handle->Instance->TXDR = I2C_MEM_ADD_LSB(MemAddress);

            /* Reset Memaddress content */
            handle->MemAddress = 0xFFFFFFFFU;
        }
        /* If Memory address size is 16Bit */
        else {
            /* Prefetch Memory Address (MSB part, LSB will be manage through interrupt) */
            handle->Instance->TXDR = I2C_MEM_ADD_MSB(MemAddress);

            /* Prepare Memaddress buffer for LSB part */
            handle->MemAddress = I2C_MEM_ADD_LSB(MemAddress);
        }

        /* Send Slave Address and Memory Address */
        I2C_TransferConfig(handle, DevAddress, (uint8_t) MemAddSize, I2C_RELOAD_MODE, I2C_GENERATE_START_WRITE);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                  to avoid the risk of I2C interrupt handle execution before current
                  process unlock */

        /* Enable ERR, TC, STOP, NACK, TXI interrupt */
        /* possible to enable all of these */
        /* I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI |
          I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
        I2C_Enable_IRQ(handle, I2C_XFER_TX_IT);

        return HAL_OK;
    } else {
        return HAL_BUSY;
    }
}

HAL_Status HAL_I2C_Mem_Read_IT(I2C_Handle *handle, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *ptrData, uint16_t Size)
{
    if (handle->State == HAL_I2C_STATE_READY) {
        if ((ptrData == NULL) || (Size == 0U)) {
            handle->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
            return HAL_ERROR;
        }

        if (((((handle->Instance->ISR) & I2C_ISR_BUSY) == I2C_ISR_BUSY) ? SET : RESET) == SET) {
            return HAL_BUSY;
        }

        handle->State     = HAL_I2C_STATE_BUSY_RX;
        handle->Mode      = HAL_I2C_MODE_MEM;
        handle->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        handle->buffPtr     = ptrData;
        handle->xferCount   = Size;
        handle->xferOptions = I2C_NO_OPTION_FRAME;
        handle->xferISR     = I2C_Mem_ISR_IT;
        handle->DevAddress  = DevAddress;

        /* If Memory address size is 8Bit */
        if (MemAddSize == I2C_MEMADD_SIZE_8BIT) {
            /* Prefetch Memory Address */
            handle->Instance->TXDR = I2C_MEM_ADD_LSB(MemAddress);

            /* Reset Memaddress content */
            handle->MemAddress = 0xFFFFFFFFU;
        }
        /* If Memory address size is 16Bit */
        else {
            /* Prefetch Memory Address (MSB part, LSB will be manage through interrupt) */
            handle->Instance->TXDR = I2C_MEM_ADD_MSB(MemAddress);

            /* Prepare Memaddress buffer for LSB part */
            handle->MemAddress = I2C_MEM_ADD_LSB(MemAddress);
        }
        /* Send Slave Address and Memory Address */
        I2C_TransferConfig(handle, DevAddress, (uint8_t) MemAddSize, I2C_SOFTEND_MODE, I2C_GENERATE_START_WRITE);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                  to avoid the risk of I2C interrupt handle execution before current
                  process unlock */

        /* Enable ERR, TC, STOP, NACK, TXI interrupt */
        /* possible to enable all of these */
        /* I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI |
          I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
        I2C_Enable_IRQ(handle, I2C_XFER_TX_IT);

        return HAL_OK;
    } else {
        return HAL_BUSY;
    }
}

static HAL_Status I2C_Mem_ISR_IT(struct __I2C_Handle *handle, uint32_t ITFlags, uint32_t ITSources) {}

// HAL_Status HAL_I2C_Master_Seq_TX_IT(I2C_Handle *handle, uint16_t DevAddress, uint8_t *ptrData, uint16_t Size, uint32_t TXOptions) {}
// HAL_Status HAL_I2C_Master_Seq_RX_IT(I2C_Handle *handle, uint16_t DevAddress, uint8_t *ptrData, uint16_t Size, uint32_t TXOptions) {}

// HAL_Status HAL_I2C_Slave_Seq_TX_IT(I2C_Handle *handle, uint8_t *ptrData, uint16_t Size, uint32_t TXOptions);
// HAL_Status HAL_I2C_Slave_Seq_RX_IT(I2C_Handle *handle, uint8_t *ptrData, uint16_t Size, uint32_t TXOptions);

HAL_Status HAL_I2C_Master_Abort_IT(I2C_Handle *handle, uint16_t DevAddress)
{
    // TODO: implement later
    return HAL_OK;
}

void HAL_I2C_EV_IRQHandler(I2C_Handle *handle)
{
    /* Get current IT Flags and IT sources value */
    uint32_t itflags   = READ_REG(handle->Instance->ISR);
    uint32_t itsources = READ_REG(handle->Instance->CR1);

    /* I2C events treatment -------------------------------------*/
    if (handle->xferISR != NULL) {
        handle->xferISR(handle, itflags, itsources);
    }
}

void HAL_I2C_ER_IRQHandler(I2C_Handle *handle)
{
    uint32_t itflags   = READ_REG(handle->Instance->ISR);
    uint32_t itsources = READ_REG(handle->Instance->CR1);
    uint32_t tmperror;

    /* I2C Bus error interrupt occurred ------------------------------------*/
    if ((((((itflags) &I2C_ISR_BERR) == I2C_ISR_BERR) ? SET : RESET) != RESET) &&
        (((((itsources) & (I2C_CR1_ERRIE)) == (I2C_CR1_ERRIE)) ? SET : RESET) != RESET)) {
        handle->ErrorCode |= HAL_I2C_ERROR_BERR;
        handle->Instance->ICR = I2C_ISR_BERR;
    }

    /* I2C Over-Run/Under-Run interrupt occurred ----------------------------------------*/
    if ((((((itflags) &I2C_ISR_OVR) == I2C_ISR_OVR) ? SET : RESET) != RESET) &&
        (((((itsources) & (I2C_CR1_ERRIE)) == (I2C_CR1_ERRIE)) ? SET : RESET) != RESET)) {
        handle->ErrorCode |= HAL_I2C_ERROR_OVR;
        handle->Instance->ICR = I2C_ISR_OVR;
    }

    /* I2C Arbitration Loss error interrupt occurred -------------------------------------*/
    if ((((((itflags) &I2C_ISR_ARLO) == I2C_ISR_ARLO) ? SET : RESET) != RESET) &&
        (((((itsources) & (I2C_CR1_ERRIE)) == (I2C_CR1_ERRIE)) ? SET : RESET) != RESET)) {
        handle->ErrorCode |= HAL_I2C_ERROR_ARLO;
        handle->Instance->ICR = I2C_ISR_ARLO;
    }

    /* Store current volatile hi2c->ErrorCode, misra rule */
    tmperror = handle->ErrorCode;

    /* Call the Error Callback in case of Error detected */
    if ((tmperror & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_OVR | HAL_I2C_ERROR_ARLO)) != HAL_I2C_ERROR_NONE) {
        I2C_ITError(handle, tmperror);
    }
}

__weak void HAL_I2C_MasterTxCpltCallback(I2C_Handle *handle)
{
    UNUSED(handle);
}

__weak void HAL_I2C_MasterRxCpltCallback(I2C_Handle *handle)
{
    UNUSED(handle);
}

__weak void HAL_I2C_ListenCpltCallback(I2C_Handle *handle)
{
    UNUSED(handle);
}

__weak void HAL_I2C_MemTxCpltCallback(I2C_Handle *handle)
{
    UNUSED(handle);
}

__weak void HAL_I2C_MemRxCpltCallback(I2C_Handle *handle)
{
    UNUSED(handle);
}

__weak void HAL_I2C_ErrorCallback(I2C_Handle *handle)
{
    UNUSED(handle);
}

__weak void HAL_I2C_AbortCpltCallback(I2C_Handle *handle)
{
    UNUSED(handle);
}

HAL_I2C_State HAL_I2C_GetState(const I2C_Handle *handle)
{
    return handle->State;
}

HAL_I2C_Mode HAL_I2C_GetMode(const I2C_Handle *handle)
{
    return handle->Mode;
}

uint32_t HAL_I2C_GetError(const I2C_Handle *handle)
{
    return handle->ErrorCode;
}
