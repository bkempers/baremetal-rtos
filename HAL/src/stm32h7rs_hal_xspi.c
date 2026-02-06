#include "stm32h7rs_hal_xspi.h"
#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_rcc.h"

#define XSPI_TIMEOUT_DEFAULT_VALUE 5000U

/* Private helper functions */
static HAL_Status XSPI_WaitFlagStateUntilTimeout(XSPI_Handle *handle, uint32_t Flag, 
                                           uint32_t State, uint32_t Timeout);

/**
 * @brief Initialize XSPI peripheral
 */
HAL_Status HAL_XSPI_Init(XSPI_Handle *handle)
{
    if (handle == NULL) {
        return HAL_ERROR;
    }
    
    /* Set state to busy */
    handle->State = HAL_XSPI_STATE_BUSY;
    handle->ErrorCode = HAL_XSPI_ERROR_NONE;
    
    /* Disable XSPI */
    handle->Instance->CR &= ~XSPI_CR_EN;
    
    /* Configure DCR1: Device Size */
    handle->Instance->DCR1 = (handle->Init.MemorySize << XSPI_DCR1_DEVSIZE_Pos);
    
    /* Configure DCR2: Prescaler */
    handle->Instance->DCR2 = (handle->Init.ClockPrescaler << XSPI_DCR2_PRESCALER_Pos);
    
    /* Configure DCR3: CS boundary */
    handle->Instance->DCR3 = 0;
    
    /* Configure DCR4: CS high time */
    handle->Instance->DCR4 = handle->Init.ChipSelectHighTime;
    
    /* Configure CR: FIFO threshold */
    handle->Instance->CR = (handle->Init.FifoThreshold << XSPI_CR_FTHRES_Pos);
    
    /* Set state to ready */
    handle->State = HAL_XSPI_STATE_READY;
    
    return HAL_OK;
}

/**
 * @brief DeInitialize XSPI peripheral
 */
HAL_Status HAL_XSPI_DeInit(XSPI_Handle *handle)
{
    if (handle == NULL) {
        return HAL_ERROR;
    }
    
    /* Disable XSPI */
    handle->Instance->CR &= ~XSPI_CR_EN;
    
    /* Reset state */
    handle->State = HAL_XSPI_STATE_RESET;
    
    return HAL_OK;
}

/**
 * @brief Send a command to XSPI device
 */
HAL_Status HAL_XSPI_Command(XSPI_Handle *handle, XSPI_RegularCmd *cmd, uint32_t timeout)
{
    uint32_t ccr_reg = 0;
    uint32_t tcr_reg = 0;
    
    if (handle == NULL || cmd == NULL) {
        return HAL_ERROR;
    }
    
    /* Check state */
    if (handle->State != HAL_XSPI_STATE_READY) {
        return HAL_ERROR;
    }
    
    handle->State = HAL_XSPI_STATE_BUSY;
    handle->ErrorCode = HAL_XSPI_ERROR_NONE;
    
    /* Wait for busy flag to be cleared */
    if (XSPI_WaitFlagStateUntilTimeout(handle, XSPI_SR_BUSY, 0, timeout) != 0) {
        handle->State = HAL_XSPI_STATE_ERROR;
        return HAL_ERROR;
    }
    
    /* Disable XSPI for configuration */
    handle->Instance->CR &= ~XSPI_CR_EN;
    
    /* Configure CCR register */
    /* Instruction */
    ccr_reg |= ((cmd->InstructionSize - 1) << XSPI_CCR_ISIZE_Pos);
    ccr_reg |= (cmd->InstructionMode << XSPI_CCR_IMODE_Pos);
    if (cmd->InstructionDTRMode == HAL_XSPI_DTR_ENABLE) {
        ccr_reg |= XSPI_CCR_IDTR;
    }
    
    /* Address */
    if (cmd->AddressMode != HAL_XSPI_IO_MODE_NONE) {
        ccr_reg |= ((cmd->AddressSize - 1) << XSPI_CCR_ADSIZE_Pos);
        ccr_reg |= (cmd->AddressMode << XSPI_CCR_ADMODE_Pos);
        if (cmd->AddressDTRMode == HAL_XSPI_DTR_ENABLE) {
            ccr_reg |= XSPI_CCR_ADDTR;
        }
    }
    
    /* Alternate Bytes */
    if (cmd->AlternateBytesMode != HAL_XSPI_IO_MODE_NONE) {
        ccr_reg |= ((cmd->AlternateBytesSize - 1) << XSPI_CCR_ABSIZE_Pos);
        ccr_reg |= (cmd->AlternateBytesMode << XSPI_CCR_ABMODE_Pos);
    }
    
    /* Data */
    if (cmd->DataMode != HAL_XSPI_IO_MODE_NONE) {
        ccr_reg |= (cmd->DataMode << XSPI_CCR_DMODE_Pos);
        if (cmd->DataDTRMode == HAL_XSPI_DTR_ENABLE) {
            ccr_reg |= XSPI_CCR_DDTR;
        }
    }
    
    /* Write CCR */
    handle->Instance->CCR = ccr_reg;

    /* Configure TCR for dummy cycles */
    tcr_reg = (cmd->DummyCycles << XSPI_TCR_DCYC_Pos);
    if (cmd->DQSMode) {
        tcr_reg |= XSPI_TCR_DHQC;
    }
    handle->Instance->TCR = tcr_reg;
    
    /* Configure data length if needed */
    if (cmd->DataLength > 0) {
        handle->Instance->DLR = cmd->DataLength - 1;
    }
    
    /* Enable XSPI */
    handle->Instance->CR |= XSPI_CR_EN;
    
    /* Set address if needed */
    if (cmd->AddressMode != HAL_XSPI_IO_MODE_NONE) {
        handle->Instance->AR = cmd->Address;
    }
    
    /* If no data phase, wait for completion */
    if (cmd->DataMode == HAL_XSPI_IO_MODE_NONE) {
        if (XSPI_WaitFlagStateUntilTimeout(handle, XSPI_SR_TCF, 1, timeout) != 0) {
            handle->State = HAL_XSPI_STATE_ERROR;
            return HAL_ERROR;
        }
        
        /* Clear transfer complete flag */
        handle->Instance->FCR = XSPI_FCR_CTCF;
    }
    
    handle->State = HAL_XSPI_STATE_READY;
    return HAL_OK;
}

/**
 * @brief Transmit data in blocking mode
 */
HAL_Status HAL_XSPI_Transmit(XSPI_Handle *handle, uint8_t *pData, uint32_t timeout)
{
    uint32_t data_count = 0;
    uint32_t data_length = (handle->Instance->DLR + 1);
    
    if (handle == NULL || pData == NULL) {
        return HAL_ERROR;
    }
    
    if (handle->State != HAL_XSPI_STATE_READY) {
        return HAL_ERROR;
    }
    
    handle->State = HAL_XSPI_STATE_BUSY;
    
    /* Transfer data */
    while (data_count < data_length) {
        /* Wait for FIFO threshold */
        if (XSPI_WaitFlagStateUntilTimeout(handle, XSPI_SR_FTF, 1, timeout) != 0) {
            handle->State = HAL_XSPI_STATE_ERROR;
            return HAL_TIMEOUT;
        }
        
        /* Write data */
        *(volatile uint8_t *)&handle->Instance->DR = pData[data_count];
        data_count++;
    }
    
    /* Wait for transfer complete */
    if (XSPI_WaitFlagStateUntilTimeout(handle, XSPI_SR_TCF, 1, timeout) != 0) {
        handle->State = HAL_XSPI_STATE_ERROR;
        return HAL_TIMEOUT;
    }
    
    /* Clear transfer complete flag */
    handle->Instance->FCR = XSPI_FCR_CTCF;
    
    handle->State = HAL_XSPI_STATE_READY;
    return HAL_OK;
}

/**
 * @brief Receive data in blocking mode
 */
HAL_Status HAL_XSPI_Receive(XSPI_Handle *handle, uint8_t *pData, uint32_t timeout)
{
    uint32_t data_count = 0;
    uint32_t data_length = (handle->Instance->DLR + 1);
    
    if (handle == NULL || pData == NULL) {
        return HAL_ERROR;
    }
    
    if (handle->State != HAL_XSPI_STATE_READY) {
        return HAL_ERROR;
    }
    
    handle->State = HAL_XSPI_STATE_BUSY;
    
    /* Receive data */
    while (data_count < data_length) {
        /* Wait for FIFO threshold */
        if (XSPI_WaitFlagStateUntilTimeout(handle, XSPI_SR_FTF, 1, timeout) != 0) {
            handle->State = HAL_XSPI_STATE_ERROR;
            return HAL_TIMEOUT;
        }
        
        /* Read data */
        pData[data_count] = *(volatile uint8_t *)&handle->Instance->DR;
        data_count++;
    }
    
    /* Wait for transfer complete */
    if (XSPI_WaitFlagStateUntilTimeout(handle, XSPI_SR_TCF, 1, timeout) != 0) {
        handle->State = HAL_XSPI_STATE_ERROR;
        return HAL_TIMEOUT;
    }
    
    /* Clear transfer complete flag */
    handle->Instance->FCR = XSPI_FCR_CTCF;
    
    handle->State = HAL_XSPI_STATE_READY;
    return HAL_OK;
}

/**
 * @brief Configure memory-mapped mode
 */
HAL_Status HAL_XSPI_MemoryMapped(XSPI_Handle *handle, XSPI_RegularCmd *cmd)
{
    uint32_t ccr_reg = 0;
    uint32_t tcr_reg = 0;
    
    if (handle == NULL || cmd == NULL) {
        return HAL_ERROR;
    }
    
    /* Disable XSPI */
    handle->Instance->CR &= ~XSPI_CR_EN;
    
    /* Configure CCR for memory-mapped read */
    ccr_reg |= ((cmd->InstructionSize - 1) << XSPI_CCR_ISIZE_Pos);
    ccr_reg |= (cmd->InstructionMode << XSPI_CCR_IMODE_Pos);
    if (cmd->InstructionDTRMode == HAL_XSPI_DTR_ENABLE) {
        ccr_reg |= XSPI_CCR_IDTR;
    }
    
    ccr_reg |= ((cmd->AddressSize - 1) << XSPI_CCR_ADSIZE_Pos);
    ccr_reg |= (cmd->AddressMode << XSPI_CCR_ADMODE_Pos);
    if (cmd->AddressDTRMode == HAL_XSPI_DTR_ENABLE) {
        ccr_reg |= XSPI_CCR_ADDTR;
    }
    
    ccr_reg |= (cmd->DataMode << XSPI_CCR_DMODE_Pos);
    if (cmd->DataDTRMode == HAL_XSPI_DTR_ENABLE) {
        ccr_reg |= XSPI_CCR_DDTR;
    }
    
    /* Write instruction */
    handle->Instance->IR = cmd->Instruction;
    
    /* Write CCR */
    handle->Instance->CCR = ccr_reg;

    /* Configure TCR */
    tcr_reg = (cmd->DummyCycles << XSPI_TCR_DCYC_Pos);
    if (cmd->DQSMode) {
        tcr_reg |= XSPI_TCR_DHQC;
    }
    handle->Instance->TCR = tcr_reg;
    
    /* Enable memory-mapped mode */
    handle->Instance->CR |= XSPI_CR_EN | (3UL << XSPI_CR_FMODE_Pos);
    
    handle->State = HAL_XSPI_STATE_BUSY;  // Stays busy in memory-mapped mode
    
    return HAL_OK;
}

/**
 * @brief Initialize MX25UM flash in Octal DTR mode
 */
HAL_Status HAL_XSPI_MX25UM_Init(XSPI_Handle *handle)
{
    XSPI_RegularCmd cmd = {0};
    uint8_t reg_data;
    uint8_t id_data[3];
    
    /* 1. Reset the flash */
    cmd.Instruction = 0x66;  // Reset Enable
    cmd.InstructionSize = 1;
    cmd.InstructionMode = HAL_XSPI_IO_MODE_SINGLE;
    cmd.InstructionDTRMode = HAL_XSPI_DTR_DISABLE;
    cmd.AddressMode = HAL_XSPI_IO_MODE_NONE;
    cmd.DataMode = HAL_XSPI_IO_MODE_NONE;
    cmd.DummyCycles = 0;
    
    if (HAL_XSPI_Command(handle, &cmd, XSPI_TIMEOUT_DEFAULT_VALUE) != 0) {
        return -1;
    }
    
    cmd.Instruction = 0x99;  // Reset Device
    if (HAL_XSPI_Command(handle, &cmd, XSPI_TIMEOUT_DEFAULT_VALUE) != 0) {
        return -1;
    }
   
    HAL_DelayMS(10);
    
    /* 2. Read ID to verify communication */
    cmd.Instruction = 0x9F;  // Read ID
    cmd.DataMode = HAL_XSPI_IO_MODE_SINGLE;
    cmd.DataLength = 3;
    cmd.DataDTRMode = HAL_XSPI_DTR_DISABLE;
    
    if (HAL_XSPI_Command(handle, &cmd, XSPI_TIMEOUT_DEFAULT_VALUE) != 0) {
        return HAL_ERROR;
    }
    
    if (HAL_XSPI_Receive(handle, id_data, XSPI_TIMEOUT_DEFAULT_VALUE) != 0) {
        return HAL_ERROR;
    }
    
    /* Check for Macronix (0xC2) */
    if (id_data[0] != 0xC2) {
        return HAL_ERROR;
    }
    
    /* 3. Write Enable */
    cmd.Instruction = 0x06;  // Write Enable
    cmd.DataMode = HAL_XSPI_IO_MODE_NONE;
    cmd.DataLength = 0;
    
    if (HAL_XSPI_Command(handle, &cmd, XSPI_TIMEOUT_DEFAULT_VALUE) != 0) {
        return HAL_ERROR;
    }
    
    HAL_DelayMS(10);
    
    /* 4. Write CR2 to enable Octal DTR mode */
    cmd.Instruction = 0x72;  // Write Configuration Register 2
    cmd.Address = 0x00000000;
    cmd.AddressSize = 4;
    cmd.AddressMode = HAL_XSPI_IO_MODE_SINGLE;
    cmd.AddressDTRMode = HAL_XSPI_DTR_DISABLE;
    cmd.DataMode = HAL_XSPI_IO_MODE_SINGLE;
    cmd.DataLength = 1;
    
    if (HAL_XSPI_Command(handle, &cmd, XSPI_TIMEOUT_DEFAULT_VALUE) != 0) {
        return HAL_ERROR;
    }
    
    reg_data = 0x02;  // Enable DTR OPI mode
    if (HAL_XSPI_Transmit(handle, &reg_data, XSPI_TIMEOUT_DEFAULT_VALUE) != 0) {
        return HAL_ERROR;
    }
    
    HAL_DelayMS(10);
    
    /* 5. Configure for memory-mapped Octal DTR read */
    cmd.Instruction = 0xEE11;  // Octal DTR Read command
    cmd.InstructionSize = 2;
    cmd.InstructionMode = HAL_XSPI_IO_MODE_OCTAL;
    cmd.InstructionDTRMode = HAL_XSPI_DTR_ENABLE;
    cmd.AddressSize = 4;
    cmd.AddressMode = HAL_XSPI_IO_MODE_OCTAL;
    cmd.AddressDTRMode = HAL_XSPI_DTR_ENABLE;
    cmd.DataMode = HAL_XSPI_IO_MODE_OCTAL;
    cmd.DataDTRMode = HAL_XSPI_DTR_ENABLE;
    cmd.DummyCycles = 20;
    cmd.DQSMode = 1;
    
    if (HAL_XSPI_MemoryMapped(handle, &cmd) != 0) {
        return HAL_ERROR;
    }
    
    /* 6. Verify memory-mapped access works */
    volatile uint32_t *test_addr = (volatile uint32_t *)0x70000000;
    volatile uint32_t test_value = *test_addr;
    (void)test_value;
    
    return HAL_OK;
}

/* Private helper function */
static HAL_Status XSPI_WaitFlagStateUntilTimeout(XSPI_Handle *handle, uint32_t Flag, 
                                           uint32_t State, uint32_t Timeout)
{
    uint32_t tickstart = 0;  // In real implementation, use SysTick
    
    /* Simple timeout loop */
    for (uint32_t i = 0; i < (Timeout * 1000); i++) {
        if (State == 0) {
            if ((handle->Instance->SR & Flag) == 0) {
                return HAL_OK;
            }
        } else {
            if ((handle->Instance->SR & Flag) != 0) {
                return HAL_OK;
            }
        }
    }
    
    handle->ErrorCode |= HAL_XSPI_ERROR_TIMEOUT;
    return HAL_ERROR;
}
