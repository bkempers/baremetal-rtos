#include "stm32h7rs_hal_dma.h"

static void DMA_SetConfig(DMA_Handle const *const handle, uint32_t src, uint32_t dest, uint32_t data_size);
static void DMA_Init_Channel(DMA_Handle const *const handle);

HAL_Status HAL_DMA_Init(DMA_Handle *handle)
{
  uint32_t tickstart = HAL_GetTick();

  if (handle == NULL)
  {
    return HAL_ERROR;
  }

    /* Initialize the callbacks */
  if (handle->State == HAL_DMA_STATE_RESET)
  {
    /* Clean all callbacks */
    handle->XferCpltCallback     = NULL;
    handle->XferHalfCpltCallback = NULL;
    handle->XferErrorCallback    = NULL;
    handle->XferAbortCallback    = NULL;
    handle->XferSuspendCallback  = NULL;
  }

  /* Update the DMA channel state */
  handle->State = HAL_DMA_STATE_BUSY;

  /* Disable the DMA channel */
  handle->Instance->CCR |= (DMA_CCR_SUSP | DMA_CCR_RESET);

  /* Check if the DMA channel is effectively disabled */
  while ((handle->Instance->CCR & DMA_CCR_EN) != 0U)
  {
    /* Check for the Timeout */
    if ((HAL_GetTick() - tickstart) > HAL_TIMEOUT_DMA_ABORT)
    {
      /* Update the DMA channel error code */
      handle->ErrorCode = HAL_DMA_ERROR_TIMEOUT;

      /* Update the DMA channel state */
      handle->State = HAL_DMA_STATE_ERROR;

      return HAL_ERROR;
    }
  }

  /* Initialize the DMA channel registers */
  DMA_Init_Channel(handle);

  /* Update DMA channel operation mode */
  handle->Mode = handle->Init.Mode;

  /* Update the DMA channel error code */
  handle->ErrorCode = HAL_DMA_ERROR_NONE;

  /* Update the DMA channel state */
  handle->State = HAL_DMA_STATE_READY;

  return HAL_OK;
}

HAL_Status HAL_DMA_DeInit(DMA_Handle *handle)
{
    // TODO: implement later
    return HAL_OK;
}

HAL_Status HAL_DMA_Start(DMA_Handle *handle, uint32_t src, uint32_t dest, uint32_t data_size)
{
  /* Check the DMA peripheral handle parameter */
  if (handle == NULL)
  {
    return HAL_ERROR;
  }

  /* Check DMA channel state */
  if (handle->State == HAL_DMA_STATE_READY)
  {
    /* Update the DMA channel state */
    handle->State = HAL_DMA_STATE_BUSY;

    /* Update the DMA channel error code */
    handle->ErrorCode = HAL_DMA_ERROR_NONE;

    /* Configure the source address, destination address, the data size and clear flags */
    DMA_SetConfig(handle, src, dest, data_size);

    /* Enable DMA channel */
    handle->Instance->CCR |=  DMA_CCR_EN;
  }
  else
  {
    /* Update the DMA channel error code */
    handle->ErrorCode = HAL_DMA_ERROR_BUSY;

    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_Status HAL_DMA_Start_IT(DMA_Handle *handle, uint32_t src, uint32_t dest, uint32_t data_size)
{
  /* Check the DMA peripheral handle parameter */
  if (handle == NULL)
  {
    return HAL_ERROR;
  }

  /* Check DMA channel state */
  if (handle->State == HAL_DMA_STATE_READY)
  {
    /* Update the DMA channel state */
    handle->State = HAL_DMA_STATE_BUSY;

    /* Update the DMA channel error code */
    handle->ErrorCode = HAL_DMA_ERROR_NONE;

    /* Configure the source address, destination address, the data size and clear flags */
    DMA_SetConfig(handle, src, dest, data_size);

    /* Enable common interrupts: Transfer Complete and Transfer Errors ITs */
    handle->Instance->CCR |= (DMA_CCR_TCIE | DMA_CCR_DTEIE | DMA_CCR_ULEIE | DMA_CCR_USEIE | DMA_CCR_TOIE);

    /* Check half transfer complete callback */
    if (handle->XferHalfCpltCallback != NULL)
    {
      /* If Half Transfer complete callback is set, enable the corresponding IT */
        handle->Instance->CCR |= DMA_CCR_HTIE;
    }

    /* Check Half suspend callback */
    if (handle->XferSuspendCallback != NULL)
    {
      /* If Transfer suspend callback is set, enable the corresponding IT */
        handle->Instance->CCR |= DMA_CCR_SUSPIE;
    }

    /* Enable DMA channel */
    handle->Instance->CCR |= DMA_CCR_EN;
  }
  else
  {
    /* Update the DMA channel error code */
    handle->ErrorCode = HAL_DMA_ERROR_BUSY;

    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_Status HAL_DMA_Abort(DMA_Handle *const handle)
{
  /* Get tick number */
  uint32_t tickstart =  HAL_GetTick();

  /* Check the DMA peripheral handle parameter */
  if (handle == NULL)
  {
    return HAL_ERROR;
  }

  /* Check DMA channel state */
  if (handle->State != HAL_DMA_STATE_BUSY)
  {
    /* Update the DMA channel error code */
    handle->ErrorCode = HAL_DMA_ERROR_NO_XFER;

    return HAL_ERROR;
  }
  else
  {
    /* Suspend the channel */
    handle->Instance->CCR |= DMA_CCR_SUSP;

    /* Update the DMA channel state */
    handle->State = HAL_DMA_STATE_SUSPEND;

    /* Check if the DMA Channel is suspended */
    while ((handle->Instance->CSR & DMA_CSR_SUSPF) == 0U)
    {
      /* Check for the Timeout */
      if ((HAL_GetTick() - tickstart) > HAL_TIMEOUT_DMA_ABORT)
      {
        /* Update the DMA channel error code */
        handle->ErrorCode |= HAL_DMA_ERROR_TIMEOUT;

        /* Update the DMA channel state */
        handle->State = HAL_DMA_STATE_ERROR;
        
        // TODO: add linked list
        // /* Check DMA channel transfer mode */
        // if ((handle->Mode & DMA_LINKEDLIST) == DMA_LINKEDLIST)
        // {
        //   /* Update the linked-list queue state */
        //   handle->LinkedListQueue->State = HAL_DMA_QUEUE_STATE_READY;
        // }

        return HAL_ERROR;
      }
    }

    /* Reset the channel */
    handle->Instance->CCR |= DMA_CCR_RESET;

    /* Update the DMA channel state */
    handle->State = HAL_DMA_STATE_ABORT;

    /* Clear all status flags */
    handle->Instance->CFCR = (DMA_CSR_TCF | DMA_CSR_HTF | DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF | DMA_CSR_SUSPF | DMA_CSR_TOF);

    /* Update the DMA channel state */
    handle->State = HAL_DMA_STATE_READY;
    
    // TODO: add linked list
    // /* Check DMA channel transfer mode */
    // if ((handle->Mode & DMA_LINKEDLIST) == DMA_LINKEDLIST)
    // {
    //   /* Update the linked-list queue state */
    //   handle->LinkedListQueue->State = HAL_DMA_QUEUE_STATE_READY;
    //
    //   /* Clear remaining data size to ensure loading linked-list from memory next start */
    //   handle->Instance->CBR1 = 0U;
    // }
  }

  return HAL_OK;
}

HAL_Status HAL_DMA_Abort_IT(DMA_Handle *const handle)
{
  /* Check the DMA peripheral handle parameter */
  if (handle == NULL)
  {
    return HAL_ERROR;
  }

  /* Check DMA channel state */
  if (handle->State != HAL_DMA_STATE_BUSY)
  {
    /* Update the DMA channel error code */
    handle->ErrorCode = HAL_DMA_ERROR_NO_XFER;

    return HAL_ERROR;
  }
  else
  {
    /* Update the DMA channel state */
    handle->State = HAL_DMA_STATE_ABORT;

    /* Suspend the channel and activate suspend interrupt */
    handle->Instance->CCR |= (DMA_CCR_SUSP | DMA_CCR_SUSPIE);
  }

  return HAL_OK;
}

HAL_Status HAL_DMA_PollForTransfer(DMA_Handle *const handle, HAL_DMA_LevelComplete CompleteLevel, uint32_t Timeout)
{
  /* Get tick number */
  uint32_t tickstart = HAL_GetTick();
  uint32_t level_flag;
  uint32_t tmp_csr;

  /* Check the DMA peripheral handle parameter */
  if (handle == NULL)
  {
    return HAL_ERROR;
  }

  /* Check DMA channel state */
  if (handle->State != HAL_DMA_STATE_BUSY)
  {
    /* Update the DMA channel error code */
    handle->ErrorCode = HAL_DMA_ERROR_NO_XFER;

    return HAL_ERROR;
  }

  // TODO: add linked list
  // /* Polling mode is not supported in circular mode */
  // if ((handle->Mode & DMA_LINKEDLIST_CIRCULAR) == DMA_LINKEDLIST_CIRCULAR)
  // {
  //   /* Update the DMA channel error code */
  //   handle->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
  //
  //   return HAL_ERROR;
  // }

  /* Get the level transfer complete flag */
  level_flag = ((CompleteLevel == HAL_DMA_FULL_TRANSFER) ? DMA_CSR_IDLEF : DMA_CSR_HTF);

  /* Get DMA channel status */
  tmp_csr = handle->Instance->CSR;

  while ((tmp_csr & level_flag) == 0U)
  {
    /* Check for the timeout */
    if (Timeout != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
      {
        /* Update the DMA channel error code */
        handle->ErrorCode |= HAL_DMA_ERROR_TIMEOUT;

        /*
          If timeout, abort the current transfer.
          Note that the Abort function will
          - Clear all transfer flags.
          - Unlock.
          - Set the State.
        */
        (void)HAL_DMA_Abort(handle);

        return HAL_ERROR;
      }
    }

    /* Get a newer CSR register value */
    tmp_csr = handle->Instance->CSR;
  }

  /* Check trigger overrun flag */
  if ((tmp_csr & DMA_CSR_TOF) != 0U)
  {
    /* Update the DMA channel error code */
    handle->ErrorCode |= HAL_DMA_ERROR_TO;

    /* Clear the error flag */
    handle->Instance->CFCR = DMA_CSR_TOF;
  }

  /* Check error flags */
  if ((tmp_csr & (DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF)) != 0U)
  {
    /* Check the data transfer error flag */
    if ((tmp_csr & DMA_CSR_DTEF) != 0U)
    {
      /* Update the DMA channel error code */
      handle->ErrorCode |= HAL_DMA_ERROR_DTE;

      /* Clear the error flag */
      handle->Instance->CFCR = DMA_CSR_DTEF;
    }

    /* Check the update link error flag */
    if ((tmp_csr & DMA_CSR_ULEF) != 0U)
    {
      /* Update the DMA channel error code */
      handle->ErrorCode |= HAL_DMA_ERROR_ULE;

      /* Clear the error flag */
      handle->Instance->CFCR = DMA_CSR_ULEF;
    }

    /* Check the user setting error flag */
    if ((tmp_csr & DMA_CSR_USEF) != 0U)
    {
      /* Update the DMA channel error code */
      handle->ErrorCode |= HAL_DMA_ERROR_USE;

      /* Clear the error flag */
      handle->Instance->CFCR = DMA_CSR_USEF;
    }

    /* Reset the channel */
    handle->Instance->CCR |= DMA_CCR_RESET;

    /* Update the DMA channel state */
    handle->State = HAL_DMA_STATE_READY;

    // TODO: add linked list
    // /* Check DMA channel transfer mode */
    // if ((handle->Mode & DMA_LINKEDLIST) == DMA_LINKEDLIST)
    // {
    //   /* Update the linked-list queue state */
    //   handle->LinkedListQueue->State = HAL_DMA_QUEUE_STATE_READY;
    // }

    return HAL_ERROR;
  }

  /* Clear the transfer level flag */
  if (CompleteLevel == HAL_DMA_HALF_TRANSFER)
  {
    /* Clear the Half Transfer flag */
      handle->Instance->CFCR = DMA_CSR_HTF;
  }
  else if (CompleteLevel == HAL_DMA_FULL_TRANSFER)
  {
    /* Clear the transfer flags */
      handle->Instance->CFCR = (DMA_CSR_TCF | DMA_CSR_HTF);

    /* Update the DMA channel state */
    handle->State = HAL_DMA_STATE_READY;

    // TODO: add linked list
    // /* Check DMA channel transfer mode */
    // if ((handle->Mode & DMA_LINKEDLIST) == DMA_LINKEDLIST)
    // {
    //   /* Update the linked-list queue state */
    //   handle->LinkedListQueue->State = HAL_DMA_QUEUE_STATE_READY;
    // }
  }
  else
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

void HAL_DMA_IRQHandler(DMA_Handle *const handle)
{
    const DMA_TypeDef *p_dma_instance = GET_DMA_INSTANCE(handle);
  uint32_t global_it_flag =  1UL << (GET_DMA_CHANNEL(handle) & 0x1FU);

  /* Global Interrupt Flag management *********************************************************************************/
  if (IS_DMA_GLOBAL_ACTIVE_FLAG(p_dma_instance, global_it_flag) == 0U)
  {
    return; /* the global interrupt flag for the current channel is down , nothing to do */
  }

  /* Data Transfer Error Interrupt management *************************************************************************/
  if ((handle->Instance->CSR & DMA_CSR_DTEF) != 0U)
  {
    /* Check if interrupt source is enabled */
    if ((handle->Instance->CCR & DMA_CCR_DTEIE) != 0U)
    {
      /* Clear the transfer error flag */
        handle->Instance->CFCR = DMA_CSR_DTEF;

      /* Update the DMA channel error code */
      handle->ErrorCode |= HAL_DMA_ERROR_DTE;
    }
  }

  /* Update Linked-list Error Interrupt management ********************************************************************/
  if ((handle->Instance->CSR & DMA_CSR_ULEF) != 0U)
  {
    /* Check if interrupt source is enabled */
    if ((handle->Instance->CCR & DMA_CCR_ULEIE)!= 0U)
    {
      /* Clear the update linked-list error flag */
        handle->Instance->CFCR = DMA_CSR_ULEF;

      /* Update the DMA channel error code */
      handle->ErrorCode |= HAL_DMA_ERROR_ULE;
    }
  }

  /* User Setting Error Interrupt management **************************************************************************/
  if ((handle->Instance->CSR & DMA_CSR_USEF) != 0U)
  {
    /* Check if interrupt source is enabled */
    if ((handle->Instance->CCR & DMA_CCR_USEIE) != 0U)
    {
      /* Clear the user setting error flag */
        handle->Instance->CFCR = DMA_CSR_USEF;

      /* Update the DMA channel error code */
      handle->ErrorCode |= HAL_DMA_ERROR_USE;
    }
  }

  /* Trigger Overrun Interrupt management *****************************************************************************/
  if ((handle->Instance->CSR & DMA_CSR_TOF) != 0U)
  {
    /* Check if interrupt source is enabled */
    if ((handle->Instance->CCR & DMA_CCR_TOIE) != 0U)
    {
      /* Clear the trigger overrun flag */
        handle->Instance->CFCR = DMA_CSR_TOF;

      /* Update the DMA channel error code */
      handle->ErrorCode |= HAL_DMA_ERROR_TO;
    }
  }

  /* Half Transfer Complete Interrupt management **********************************************************************/
  if ((handle->Instance->CSR & DMA_CSR_HTF) != 0U)
  {
    /* Check if interrupt source is enabled */
    if ((handle->Instance->CCR & DMA_CCR_HTIE) != 0U)
    {
      /* Clear the half transfer flag */
        handle->Instance->CFCR = DMA_CSR_HTF;

      /* Check half transfer complete callback */
      if (handle->XferHalfCpltCallback != NULL)
      {
        /* Half transfer callback */
        handle->XferHalfCpltCallback(handle);
      }
    }
  }

  /* Suspend Transfer Interrupt management ****************************************************************************/
  if ((handle->Instance->CSR & DMA_CSR_SUSPF) != 0U)
  {
    /* Check if interrupt source is enabled */
    if ((handle->Instance->CCR & DMA_CCR_SUSPIE) != 0U)
    {
      /* Clear the block transfer complete flag */
        handle->Instance->CFCR = DMA_CSR_SUSPF;

      /* Check DMA channel state */
      if (handle->State == HAL_DMA_STATE_ABORT)
      {
        /* Disable the suspend transfer interrupt */
          handle->Instance->CCR &= ~DMA_CCR_SUSPIE;

        /* Reset the channel internal state and reset the FIFO */
        handle->Instance->CCR |= DMA_CCR_RESET;

        /* Update the DMA channel state */
        handle->State = HAL_DMA_STATE_READY;

        // TODO: add linked list
        // /* Check DMA channel transfer mode */
        // if ((handle->Mode & DMA_LINKEDLIST) == DMA_LINKEDLIST)
        // {
        //   /* Update the linked-list queue state */
        //   handle->LinkedListQueue->State = HAL_DMA_QUEUE_STATE_READY;
        //
        //   /* Clear remaining data size to ensure loading linked-list from memory next start */
        //   handle->Instance->CBR1 = 0U;
        // }

        /* Check transfer abort callback */
        if (handle->XferAbortCallback != NULL)
        {
          /* Transfer abort callback */
          handle->XferAbortCallback(handle);
        }

        return;
      }
      else
      {
        /* Update the DMA channel state */
        handle->State = HAL_DMA_STATE_SUSPEND;

        /* Check transfer suspend callback */
        if (handle->XferSuspendCallback != NULL)
        {
          /* Transfer suspend callback */
          handle->XferSuspendCallback(handle);
        }
      }
    }
  }

  /* Transfer Complete Interrupt management ***************************************************************************/
  if ((handle->Instance->CSR & DMA_CSR_TCF) != 0U)
  {
    /* Check if interrupt source is enabled */
    if ((handle->Instance->CCR & DMA_CCR_TCIE) != 0U)
    {
        // TODO: add linked list
      /* Check DMA channel transfer mode */
      // if ((handle->Mode & DMA_LINKEDLIST) == DMA_LINKEDLIST)
      // {
      //   /* If linked-list transfer */
      //   if (handle->Instance->CLLR == 0U)
      //   {
      //     if (handle->Instance->CBR1 == 0U)
      //     {
      //       /* Update the DMA channel state */
      //       handle->State = HAL_DMA_STATE_READY;
      //
      //       /* Update the linked-list queue state */
      //       handle->LinkedListQueue->State = HAL_DMA_QUEUE_STATE_READY;
      //     }
      //   }
      // }
      // else
      // {
        /* If normal transfer */
        if (handle->Instance->CBR1 == 0U)
        {
          /* Update the DMA channel state */
          handle->State = HAL_DMA_STATE_READY;
        }
      // }

      /* Clear TC and HT transfer flags */
        handle->Instance->CCR &= ~(DMA_CCR_TCIE | DMA_CCR_HTIE);

      /* Check transfer complete callback */
      if (handle->XferCpltCallback != NULL)
      {
        /* Channel Transfer Complete callback */
        handle->XferCpltCallback(handle);
      }
    }
  }

  /* Manage error case ************************************************************************************************/
  if (handle->ErrorCode != HAL_DMA_ERROR_NONE)
  {
    /* Reset the channel internal state and reset the FIFO */
    handle->Instance->CCR |= DMA_CCR_RESET;

    /* Update the DMA channel state */
    handle->State = HAL_DMA_STATE_READY;

    // TODO: add linked list
    // /* Check DMA channel transfer mode */
    // if ((handle->Mode & DMA_LINKEDLIST) == DMA_LINKEDLIST)
    // {
    //   /* Update the linked-list queue state */
    //   handle->LinkedListQueue->State = HAL_DMA_QUEUE_STATE_READY;
    // }

    /* Check transfer error callback */
    if (handle->XferErrorCallback != NULL)
    {
      /* Transfer error callback */
      handle->XferErrorCallback(handle);
    }
  }
}

HAL_DMA_State HAL_DMA_GetState(DMA_Handle const *const handle) {
    return handle->State;
}

uint32_t HAL_DMA_GetError(DMA_Handle const *const handle) {
    return handle->ErrorCode;
}

static void DMA_SetConfig(DMA_Handle const *const handle, uint32_t src, uint32_t dest, uint32_t data_size)
{
  /* Configure the DMA channel data size */
  MODIFY_REG(handle->Instance->CBR1, DMA_CBR1_BNDT, (data_size & DMA_CBR1_BNDT));

  /* Clear all interrupt flags */
  handle->Instance->CFCR = (DMA_CSR_TCF | DMA_CSR_HTF | DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF | DMA_CSR_SUSPF |
                       DMA_CSR_TOF);

  /* Configure DMA channel source address */
  handle->Instance->CSAR = src;

  /* Configure DMA channel destination address */
  handle->Instance->CDAR = dest;
}

static void DMA_Init_Channel(DMA_Handle const *const handle)
{
  uint32_t tmpreg;

  /* Prepare DMA Channel Control Register (CCR) value *****************************************************************/
  tmpreg = handle->Init.Priority;

  /* Write DMA Channel Control Register (CCR) */
  MODIFY_REG(handle->Instance->CCR, DMA_CCR_PRIO | DMA_CCR_LAP | DMA_CCR_LSM, tmpreg);

  /* Prepare DMA Channel Transfer Register (CTR1) value ***************************************************************/
  tmpreg = handle->Init.DestInc | handle->Init.DestDataWidth | handle->Init.SrcInc | handle->Init.SrcDataWidth;

  /* Add parameters specific to HPDMA and GPDMA */
  if ((IS_HPDMA_INSTANCE(handle->Instance) != 0U) || (IS_GPDMA_INSTANCE(handle->Instance) != 0U))
  {
    tmpreg |= (handle->Init.TransferAllocatedPort                                             |
               (((handle->Init.DestBurstLen - 1U) << DMA_CTR1_DBL_1_Pos) & DMA_CTR1_DBL_1) |
               (((handle->Init.SrcBurstLen - 1U) << DMA_CTR1_SBL_1_Pos) & DMA_CTR1_SBL_1));
  }

  /* Write DMA Channel Transfer Register 1 (CTR1) */
  WRITE_REG(handle->Instance->CTR1, tmpreg);

  /* Prepare DMA Channel Transfer Register 2 (CTR2) value *************************************************************/
  tmpreg = handle->Init.BlockRequest | (handle->Init.Request & DMA_CTR2_REQSEL) | handle->Init.TransferEventMode;

  /* Memory to Peripheral Transfer */
  if ((handle->Init.Direction) == DMA_MEMORY_TO_PERIPH)
  {
    if ((IS_HPDMA_INSTANCE(handle->Instance) != 0U) || (IS_GPDMA_INSTANCE(handle->Instance) != 0U))
    {
      tmpreg |= DMA_CTR2_DREQ;
    }
  }
  /* Memory to Memory Transfer */
  else if ((handle->Init.Direction) == DMA_MEMORY_TO_MEMORY)
  {
    tmpreg |= DMA_CTR2_SWREQ;
  }
  else
  {
    /* Nothing to do */
  }

  /* Set DMA channel operation mode */
  tmpreg |= handle->Init.Mode;

  /* Write DMA Channel Transfer Register 2 (CTR2) */
  MODIFY_REG(handle->Instance->CTR2, (DMA_CTR2_TCEM  | DMA_CTR2_TRIGPOL | DMA_CTR2_TRIGSEL | DMA_CTR2_TRIGM |
                                    DMA_CTR2_PFREQ | DMA_CTR2_BREQ  | DMA_CTR2_DREQ    | DMA_CTR2_SWREQ   |
                                    DMA_CTR2_REQSEL), tmpreg);


  /* Write DMA Channel Block Register 1 (CBR1) ************************************************************************/
  WRITE_REG(handle->Instance->CBR1, 0U);

  /* If 2D Addressing is supported by current channel */
  if (IS_DMA_2D_ADDRESSING_INSTANCE(handle->Instance) != 0U)
  {
    /* Write DMA Channel Transfer Register 3 (CTR3) *******************************************************************/
    WRITE_REG(handle->Instance->CTR3, 0U);

    /* Write DMA Channel Block Register 2 (CBR2) **********************************************************************/
    WRITE_REG(handle->Instance->CBR2, 0U);
  }

  /* Write DMA Channel linked-list address register (CLLR) ************************************************************/
  WRITE_REG(handle->Instance->CLLR, 0U);
}


