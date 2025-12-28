#include "stm32h7rs_hal_spi.h"
#include "stm32h7rs_hal.h"

static void SPI_TxISR_8BIT(SPI_Handle *handle);
static void SPI_TxISR_16BIT(SPI_Handle *handle);
static void SPI_TxISR_32BIT(SPI_Handle *handle);
static void SPI_RxISR_8BIT(SPI_Handle *handle);
static void SPI_RxISR_16BIT(SPI_Handle *handle);
static void SPI_RxISR_32BIT(SPI_Handle *handle);
static void SPI_AbortTransfer(SPI_Handle *handle);
static void SPI_CloseTransfer(SPI_Handle *handle);
static uint32_t SPI_GetPacketSize(const SPI_Handle *handle);

HAL_Status HAL_SPI_Init(SPI_Handle *handle)
{
    uint32_t packet_length;
    uint32_t crc_length;

    if (handle == NULL) {
        return HAL_ERROR;
    }

    /* Verify that the SPI instance supports Data Size higher than 16bits */
    if ((IS_SPI_LIMITED_INSTANCE(handle->Instance)) && (handle->Init.DataSize > SPI_DATASIZE_16BIT)) {
        return HAL_ERROR;
    }

  /* Verify that the SPI instance supports requested data packing */
  packet_length = SPI_GetPacketSize(handle);
  if (((IS_SPI_LIMITED_INSTANCE(handle->Instance)) && (packet_length > SPI_LOWEND_FIFO_SIZE)) ||
      ((IS_SPI_FULL_INSTANCE(handle->Instance)) && (packet_length > SPI_HIGHEND_FIFO_SIZE)))
  {
    return HAL_ERROR;
  }
  
  handle->State = HAL_SPI_STATE_BUSY;

  /* Disable the selected SPI peripheral */
  CLEAR_BIT(handle->Instance->CR1, SPI_CR1_SPE);

  /*----------------------- SPIx CR1 & CR2 Configuration ---------------------*/
  /* Configure : SPI Mode, Communication Mode, Clock polarity and phase, NSS management,
  Communication speed, First bit, CRC calculation state, CRC Length */

  /* SPIx NSS Software Management Configuration */
  if ((handle->Init.NSS == SPI_NSS_SOFT) && (((handle->Init.Mode == SPI_CFG2_MASTER) &&  \
                                            (handle->Init.NSSPolarity == SPI_NSS_POLARITY_LOW)) || \
                                           ((handle->Init.Mode == SPI_MODE_SLAVE) && \
                                            (handle->Init.NSSPolarity == SPI_NSS_POLARITY_HIGH))))
  {
    SET_BIT(handle->Instance->CR1, SPI_CR1_SSI);
  }

  /* SPIx Master Rx Auto Suspend Configuration */
  if (((handle->Init.Mode & SPI_CFG2_MASTER) == SPI_CFG2_MASTER) && (handle->Init.DataSize >= SPI_DATASIZE_8BIT))
  {
    MODIFY_REG(handle->Instance->CR1, SPI_CR1_MASRX, handle->Init.MasterReceiverAutoSusp);
  }
  else
  {
    CLEAR_BIT(handle->Instance->CR1, SPI_CR1_MASRX);
  }

  /* SPIx CFG1 Configuration */
  WRITE_REG(handle->Instance->CFG1, (handle->Init.BaudRatePrescaler | handle->Init.CRCCalculation | crc_length |
                                   handle->Init.FifoThreshold     | handle->Init.DataSize));

  /* SPIx CFG2 Configuration */
  WRITE_REG(handle->Instance->CFG2, (handle->Init.NSSPMode              | handle->Init.TIMode    |
                                   handle->Init.NSSPolarity             | handle->Init.NSS       |
                                   handle->Init.CLKPolarity             | handle->Init.CLKPhase  |
                                   handle->Init.FirstBit                | handle->Init.Mode      |
                                   handle->Init.MasterInterDataIdleness | handle->Init.Direction |
                                   handle->Init.MasterSSIdleness        | handle->Init.IOSwap    |
                                   handle->Init.ReadyMasterManagement   | handle->Init.ReadyPolarity));

  /* Insure that AFCNTR is managed only by Master */
  if ((handle->Init.Mode & SPI_CFG2_MASTER) == SPI_CFG2_MASTER)
  {
    /* Alternate function GPIOs control */
    MODIFY_REG(handle->Instance->CFG2, SPI_CFG2_AFCNTR, (handle->Init.MasterKeepIOState));
  }

  handle->ErrorCode = HAL_SPI_ERROR_NONE;
  handle->State     = HAL_SPI_STATE_READY;

  return HAL_OK;
}

HAL_Status HAL_SPI_DeInit(SPI_Handle *handle)
{
    //TODO: implement later
    return HAL_OK;
}

// polling tx/rx

HAL_Status HAL_SPI_Transmit_IT(SPI_Handle *handle, const uint8_t *ptrData, uint16_t Size)
{
  if ((ptrData == NULL) || (Size == 0UL))
  {
    return HAL_ERROR;
  }

  if (handle->State != HAL_SPI_STATE_READY)
  {
    return HAL_BUSY;
  }

  /* Set the transaction information */
  handle->State       = HAL_SPI_STATE_BUSY_TX;
  handle->ErrorCode   = HAL_SPI_ERROR_NONE;
  handle->txBuffPtr  = (const uint8_t *)ptrData;
  handle->txSize  = Size;
  handle->txCount = Size;

  /* Init field not used in handle to zero */
  handle->rxBuffPtr  = NULL;
  handle->rxSize  = (uint16_t) 0UL;
  handle->rxCount = (uint16_t) 0UL;
  handle->rxISR       = NULL;

  /* Set the function for IT treatment */
  if ((handle->Init.DataSize > SPI_DATASIZE_16BIT) && (IS_SPI_FULL_INSTANCE(handle->Instance))) {
    handle->txISR = SPI_TxISR_32BIT;
  } else if (handle->Init.DataSize > SPI_DATASIZE_8BIT) {
    handle->txISR = SPI_TxISR_16BIT;
  } else {
    handle->txISR = SPI_TxISR_8BIT;
  }

  /* Configure communication direction : 1Line */
  if (handle->Init.Direction == SPI_DIRECTION_1LINE) {
    SET_BIT(handle->Instance->CR1, SPI_CR1_HDDIR);
  } else {
      MODIFY_REG(handle->Instance->CFG2, SPI_CFG2_COMM, SPI_CFG2_COMM_0);
  }

  /* Set the number of data at current transfer */
  MODIFY_REG(handle->Instance->CR2, SPI_CR2_TSIZE, Size);

  /* Enable SPI peripheral */
  SET_BIT(handle->Instance->CR1, SPI_CR1_SPE);

  /* Enable EOT, TXP, FRE, MODF and UDR interrupts */
  handle->Instance->IER |=  (SPI_IER_EOTIE | SPI_IER_TXPIE | SPI_IER_UDRIE | SPI_IER_TIFREIE | SPI_IER_MODFIE);

  if (handle->Init.Mode == SPI_CFG2_MASTER) {
    /* Master transfer start */
    SET_BIT(handle->Instance->CR1, SPI_CR1_CSTART);
  }

  return HAL_OK;
}

HAL_Status HAL_SPI_Receive_IT(SPI_Handle *handle, uint8_t *ptrData, uint16_t Size)
{
  if (handle->State != HAL_SPI_STATE_READY)
  {
    return HAL_BUSY;
  }

  if ((ptrData == NULL) || (Size == 0UL))
  {
    return HAL_ERROR;
  }

  /* Set the transaction information */
  handle->State       = HAL_SPI_STATE_BUSY_RX;
  handle->ErrorCode   = HAL_SPI_ERROR_NONE;
  handle->rxBuffPtr  = (uint8_t *)ptrData;
  handle->rxSize  = Size;
  handle->rxCount = Size;

  /* Init field not used in handle to zero */
  handle->txBuffPtr  = NULL;
  handle->txSize  = (uint16_t) 0UL;
  handle->txCount = (uint16_t) 0UL;
  handle->txISR       = NULL;

  /* Set the function for IT treatment */
  if ((handle->Init.DataSize > SPI_DATASIZE_16BIT) && (IS_SPI_FULL_INSTANCE(handle->Instance))) {
    handle->rxISR = SPI_RxISR_32BIT;
  } else if (handle->Init.DataSize > SPI_DATASIZE_8BIT) {
    handle->rxISR = SPI_RxISR_16BIT;
  } else {
    handle->rxISR = SPI_RxISR_8BIT;
  }

  /* Configure communication direction : 1Line */
  if (handle->Init.Direction == SPI_DIRECTION_1LINE) {
      CLEAR_BIT(handle->Instance->CR1, SPI_CR1_HDDIR);
  } else {
      MODIFY_REG(handle->Instance->CFG2, SPI_CFG2_COMM, SPI_CFG2_COMM_1);
  }

  /* Note : The SPI must be enabled after unlocking current process
            to avoid the risk of SPI interrupt handle execution before current
            process unlock */

  /* Set the number of data at current transfer */
  MODIFY_REG(handle->Instance->CR2, SPI_CR2_TSIZE, Size);

  /* Enable SPI peripheral */
    SET_BIT(handle->Instance->CR1, SPI_CR1_SPE);

  /* Enable EOT, RXP, OVR, FRE and MODF interrupts */
  handle->Instance->IER |= (SPI_IER_EOTIE | SPI_IER_RXPIE | SPI_IER_OVRIE | SPI_IER_TIFREIE | SPI_IER_MODFIE);

  if (handle->Init.Mode == SPI_CFG2_MASTER)
  {
    /* Master transfer start */
    SET_BIT(handle->Instance->CR1, SPI_CR1_CSTART);
  }

  return HAL_OK;
}

HAL_Status HAL_SPI_TransmitReceive_IT(SPI_Handle *handle, const uint8_t *txData, uint8_t *rxData, uint16_t Size)
{
    //TODO: implement later
    return HAL_OK;
}

void HAL_SPI_IRQHandler(SPI_Handle *handle) {
  uint32_t itsource = handle->Instance->IER;
  uint32_t itflag   = handle->Instance->SR;
  uint32_t trigger  = itsource & itflag;
  uint32_t cfg1     = handle->Instance->CFG1;
  uint32_t handled  = 0UL;

  HAL_SPI_State State = handle->State;
  __IO uint16_t *prxdr_16bits = (__IO uint16_t *)(&(handle->Instance->RXDR));

  /* SPI in SUSPEND mode  ----------------------------------------------------*/
  if (HAL_IS_BIT_SET(itflag, SPI_SR_SUSP) && HAL_IS_BIT_SET(itsource, SPI_SR_EOT))
  {
    /* Clear the Suspend flag */
     SET_BIT(handle->Instance->IFCR, SPI_IFCR_SUSPC);

    /* Suspend on going, Call the Suspend callback */
    handle->suspendCallback(handle);
  }

  /* SPI in mode Transmitter and Receiver ------------------------------------*/
  if (HAL_IS_BIT_CLR(trigger, SPI_SR_OVR) && HAL_IS_BIT_CLR(trigger, SPI_SR_UDR) && \
      HAL_IS_BIT_SET(trigger, SPI_SR_DXP))
  {
    handle->txISR(handle);
    handle->rxISR(handle);
    handled = 1UL;
  }

  /* SPI in mode Receiver ----------------------------------------------------*/
  if (HAL_IS_BIT_CLR(trigger, SPI_SR_OVR) && HAL_IS_BIT_SET(trigger, SPI_SR_RXP) && \
      HAL_IS_BIT_CLR(trigger, SPI_SR_DXP))
  {
    handle->rxISR(handle);
    handled = 1UL;
  }

  /* SPI in mode Transmitter -------------------------------------------------*/
  if (HAL_IS_BIT_CLR(trigger, SPI_SR_UDR) && HAL_IS_BIT_SET(trigger, SPI_SR_TXP) && \
      HAL_IS_BIT_CLR(trigger, SPI_SR_DXP))
  {
    handle->txISR(handle);
    handled = 1UL;
  }

  if (handled != 0UL)
  {
    return;
  }

  /* SPI End Of Transfer: DMA or IT based transfer */
  if (HAL_IS_BIT_SET(trigger, SPI_SR_EOT))
  {
    /* Clear EOT/TXTF/SUSP flag */
      SET_BIT(handle->Instance->IFCR, SPI_IFCR_EOTC);
      SET_BIT(handle->Instance->IFCR, SPI_IFCR_TXTFC);
      SET_BIT(handle->Instance->IFCR, SPI_IFCR_SUSPC);

    /* Disable EOT interrupt */
      handle->Instance->IER &= ~(SPI_IER_EOTIE);

    /* For the IT based receive extra polling maybe required for last packet */
    if (HAL_IS_BIT_CLR(handle->Instance->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN))
    {
      /* Pooling remaining data */
      while (handle->rxCount != 0UL)
      {
        /* Receive data in 32 Bit mode */
        if (handle->Init.DataSize > SPI_DATASIZE_16BIT)
        {
          *((uint32_t *)handle->rxBuffPtr) = *((__IO uint32_t *)&handle->Instance->RXDR);
          handle->rxBuffPtr += sizeof(uint32_t);
        }
        /* Receive data in 16 Bit mode */
        else if (handle->Init.DataSize > SPI_DATASIZE_8BIT)
        {
          *((uint16_t *)handle->rxBuffPtr) = *((__IO uint16_t *)&handle->Instance->RXDR);
          handle->rxBuffPtr += sizeof(uint16_t);
        }
        /* Receive data in 8 Bit mode */
        else
        {
          *((uint8_t *)handle->rxBuffPtr) = *((__IO uint8_t *)&handle->Instance->RXDR);
          handle->rxBuffPtr += sizeof(uint8_t);
        }

        handle->rxCount--;
      }
    }

    /* Call SPI Standard close procedure */
    SPI_CloseTransfer(handle);

    handle->State = HAL_SPI_STATE_READY;
    if (handle->ErrorCode != HAL_SPI_ERROR_NONE)
    {
      handle->errorCallback(handle);
      return;
    }

    /* Call appropriate user callback */
    if (State == HAL_SPI_STATE_BUSY_TX_RX)
    {
      handle->txrxCpltCallback(handle);
    }
    else if (State == HAL_SPI_STATE_BUSY_RX)
    {
      handle->rxCpltCallback(handle);
    }
    else if (State == HAL_SPI_STATE_BUSY_TX)
    {
      handle->txCpltCallback(handle);
    }
    else
    {
      /* End of the appropriate call */
    }

    return;
  }

  /* SPI in Error Treatment --------------------------------------------------*/
  if ((trigger & (SPI_SR_MODF | SPI_SR_OVR | SPI_SR_TIFRE | SPI_SR_UDR)) != 0UL)
  {
    /* SPI Overrun error interrupt occurred ----------------------------------*/
    if ((trigger & SPI_SR_OVR) != 0UL)
    {
      SET_BIT(handle->ErrorCode, HAL_SPI_ERROR_OVR);
      SET_BIT(handle->Instance->IFCR, SPI_IFCR_OVRC);
    }

    /* SPI Mode Fault error interrupt occurred -------------------------------*/
    if ((trigger & SPI_SR_MODF) != 0UL)
    {
      SET_BIT(handle->ErrorCode, HAL_SPI_ERROR_MODF);
      SET_BIT(handle->Instance->IFCR, (uint32_t)SPI_IFCR_MODFC);
    }

    /* SPI Frame error interrupt occurred ------------------------------------*/
    if ((trigger & SPI_SR_TIFRE) != 0UL)
    {
      SET_BIT(handle->ErrorCode, HAL_SPI_ERROR_FRE);
      SET_BIT(handle->Instance->IFCR, SPI_IFCR_TIFREC);
    }

    /* SPI Underrun error interrupt occurred ------------------------------------*/
    if ((trigger & SPI_SR_UDR) != 0UL)
    {
      SET_BIT(handle->ErrorCode, HAL_SPI_ERROR_UDR);
      SET_BIT(handle->Instance->IFCR, SPI_IFCR_UDRC);
    }

    if (handle->ErrorCode != HAL_SPI_ERROR_NONE)
    {
      /* Disable SPI peripheral */
      CLEAR_BIT(handle->Instance->CR1, SPI_CR1_SPE);

      /* Disable all interrupts */
      handle->Instance->IER &= ~(SPI_IER_EOTIE | SPI_IER_RXPIE | SPI_IER_TXPIE | SPI_IER_MODFIE | SPI_IER_OVRIE | SPI_IER_TIFREIE | SPI_IER_UDRIE);

      /* Disable the SPI DMA requests if enabled */
      if (HAL_IS_BIT_SET(cfg1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN))
      {
        /* Disable the SPI DMA requests */
        CLEAR_BIT(handle->Instance->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);

        // /* Abort the SPI DMA Rx channel */
        // if (hande->hdmarx != NULL)
        // {
        //   /* Set the SPI DMA Abort callback :
        //   will lead to call HAL_SPI_ErrorCallback() at end of DMA abort procedure */
        //   handle->hdmarx->XferAbortCallback = SPI_DMAAbortOnError;
        //   if (HAL_OK != HAL_DMA_Abort_IT(handle->hdmarx))
        //   {
        //     SET_BIT(handle->ErrorCode, HAL_SPI_ERROR_ABORT);
        //   }
        // }
        // /* Abort the SPI DMA Tx channel */
        // if (handle->hdmatx != NULL)
        // {
        //   /* Set the SPI DMA Abort callback :
        //   will lead to call HAL_SPI_ErrorCallback() at end of DMA abort procedure */
        //   handle->hdmatx->XferAbortCallback = SPI_DMAAbortOnError;
        //   if (HAL_OK != HAL_DMA_Abort_IT(handle->hdmatx))
        //   {
        //     SET_BIT(handle->ErrorCode, HAL_SPI_ERROR_ABORT);
        //   }
        // }
      }
      else
      {
        /* Restore hspi->State to Ready */
        handle->State = HAL_SPI_STATE_READY;

        /* Call user error callback */
        handle->errorCallback(handle);
      }
    }
    return;
  }
}

__weak void HAL_SPI_TxCpltCallback(SPI_Handle *handle) {
    UNUSED(handle);
}

__weak void HAL_SPI_RxCpltCallback(SPI_Handle *handle) {
    UNUSED(handle);
}

__weak void HAL_SPI_TxRxCpltCallback(SPI_Handle *handle) {
    UNUSED(handle);
}

__weak void HAL_SPI_TxHalfCpltCallback(SPI_Handle *handle) {
    UNUSED(handle);
}

__weak void HAL_SPI_RxHalfCpltCallback(SPI_Handle *handle) {
    UNUSED(handle);
}

__weak void HAL_SPI_TxRxHalfCpltCallback(SPI_Handle *handle) {
    UNUSED(handle);
}

__weak void HAL_SPI_ErrorCallback(SPI_Handle *handle) {
    UNUSED(handle);
}

__weak void HAL_SPI_AbortCpltCallback(SPI_Handle *handle) {
    UNUSED(handle);
}

__weak void HAL_SPI_SuspendCallback(SPI_Handle *handle) {
    UNUSED(handle);
}

// dma tx/rx

HAL_Status HAL_SPI_Abort(SPI_Handle *handle);
HAL_Status HAL_SPI_AbortIT(SPI_Handle *handle);

static void SPI_RxISR_8BIT(SPI_Handle *handle) {
    /* Receive data in 8 Bit mode */
  *((uint8_t *)handle->rxBuffPtr) = (*(__IO uint8_t *)&handle->Instance->RXDR);
  handle->rxBuffPtr += sizeof(uint8_t);
  handle->rxCount--;

  /* Disable IT if no more data excepted */
  if (handle->rxCount == 0UL)
  {
    /* Disable RXP interrupts */
      handle->Instance->IER &= ~(SPI_IER_RXPIE);
  }
}

static void SPI_RxISR_16BIT(SPI_Handle *handle) {
    /* Receive data in 16 Bit mode */
  *((uint16_t *)handle->rxBuffPtr) = (*(__IO uint16_t *)&handle->Instance->RXDR);
  handle->rxBuffPtr += sizeof(uint16_t);
  handle->rxCount--;

  /* Disable IT if no more data excepted */
  if (handle->rxCount == 0UL)
  {
    /* Disable RXP interrupts */
      handle->Instance->IER &= ~(SPI_IER_RXPIE);
  }
}

static void SPI_RxISR_32BIT(SPI_Handle *handle) {
    /* Receive data in 32 Bit mode */
  *((uint32_t *)handle->rxBuffPtr) = (*(__IO uint32_t *)&handle->Instance->RXDR);
  handle->rxBuffPtr += sizeof(uint32_t);
  handle->rxCount--;

  /* Disable IT if no more data excepted */
  if (handle->rxCount == 0UL)
  {
    /* Disable RXP interrupts */
      handle->Instance->IER &= ~(SPI_IER_RXPIE);
  }
}

static void SPI_TxISR_8BIT(SPI_Handle *handle) {
  /* Transmit data in 8 Bit mode */
  *(__IO uint8_t *)&handle->Instance->TXDR = *((const uint8_t *)handle->txBuffPtr);
  handle->txBuffPtr += sizeof(uint8_t);
  handle->txCount--;

  /* Disable IT if no more data excepted */
  if (handle->txCount == 0UL)
  {
    /* Disable TXP interrupts */
      handle->Instance->IER &= ~(SPI_IER_TXPIE);
  }
}

static void SPI_TxISR_16BIT(SPI_Handle *handle) {
  /* Transmit data in 16 Bit mode */
  *(__IO uint16_t *)&handle->Instance->TXDR = *((const uint16_t *)handle->txBuffPtr);
  handle->txBuffPtr += sizeof(uint16_t);
  handle->txCount--;

  /* Disable IT if no more data excepted */
  if (handle->txCount == 0UL)
  {
    /* Disable TXP interrupts */
      handle->Instance->IER &= ~(SPI_IER_TXPIE);
  }
}

static void SPI_TxISR_32BIT(SPI_Handle *handle) {
  /* Transmit data in 32 Bit mode */
  *(__IO uint32_t *)&handle->Instance->TXDR = *((const uint32_t *)handle->txBuffPtr);
  handle->txBuffPtr += sizeof(uint32_t);
  handle->txCount--;

  /* Disable IT if no more data excepted */
  if (handle->txCount == 0UL)
  {
    /* Disable TXP interrupts */
      handle->Instance->IER &= ~(SPI_IER_TXPIE);
  }
}

HAL_SPI_State HAL_SPI_GetState(const SPI_Handle *handle)
{
    return handle->State;
}

uint32_t HAL_SPI_GetError(const SPI_Handle *handle)
{
    return handle->ErrorCode;
}

static void SPI_CloseTransfer(SPI_Handle *handle)
{
  uint32_t itflag = handle->Instance->SR;

  SET_BIT(handle->Instance->IFCR, SPI_IFCR_EOTC);
  SET_BIT(handle->Instance->IFCR, SPI_IFCR_TXTFC);

  /* Disable SPI peripheral */
  CLEAR_BIT(handle->Instance->CR1, SPI_CR1_SPE);

  /* Disable ITs */
  handle->Instance->IER &= ~(SPI_IER_EOTIE | SPI_IER_TXPIE | SPI_IER_RXPIE | SPI_IER_DXPIE | SPI_IER_UDRIE | SPI_IER_OVRIE | SPI_IER_TIFREIE | SPI_IER_MODFIE);

  /* Disable Tx DMA Request */
  CLEAR_BIT(handle->Instance->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);

  /* Report UnderRun error for non RX Only communication */
  if (handle->State != HAL_SPI_STATE_BUSY_RX)
  {
    if ((itflag & SPI_SR_UDR) != 0UL)
    {
      SET_BIT(handle->ErrorCode, HAL_SPI_ERROR_UDR);
      SET_BIT(handle->Instance->IFCR, SPI_IFCR_UDRC);
    }
  }

  /* Report OverRun error for non TX Only communication */
  if (handle->State != HAL_SPI_STATE_BUSY_TX)
  {
    if ((itflag & SPI_SR_OVR) != 0UL)
    {
      SET_BIT(handle->ErrorCode, HAL_SPI_ERROR_OVR);
      SET_BIT(handle->Instance->IFCR, SPI_IFCR_OVRC);
    }

#if (USE_SPI_CRC != 0UL)
    /* Check if CRC error occurred */
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      if ((itflag & SPI_FLAG_CRCERR) != 0UL)
      {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
        __HAL_SPI_CLEAR_CRCERRFLAG(hspi);
      }
    }
#endif /* USE_SPI_CRC */
  }

  /* SPI Mode Fault error interrupt occurred -------------------------------*/
  if ((itflag & SPI_SR_MODF) != 0UL)
  {
    SET_BIT(handle->ErrorCode, HAL_SPI_ERROR_MODF);
    SET_BIT(handle->Instance->IFCR, (uint32_t)SPI_IFCR_MODFC);
  }

  /* SPI Frame error interrupt occurred ------------------------------------*/
  if ((itflag & SPI_SR_TIFRE) != 0UL)
  {
    SET_BIT(handle->ErrorCode, HAL_SPI_ERROR_FRE);
    SET_BIT(handle->Instance->IFCR, SPI_IFCR_TIFREC);
  }

  handle->txCount = (uint16_t)0UL;
  handle->rxCount = (uint16_t)0UL;
}

static uint32_t SPI_GetPacketSize(const SPI_Handle *handle)
{
    uint32_t fifo_threashold = (handle->Init.FifoThreshold >> SPI_CFG1_FTHLV_Pos) + 1UL;
    uint32_t data_size       = (handle->Init.DataSize      >> SPI_CFG1_DSIZE_Pos) + 1UL;

    /* Convert data size to Byte */
    data_size = (data_size + 7UL) / 8UL;

    return data_size * fifo_threashold;
}
