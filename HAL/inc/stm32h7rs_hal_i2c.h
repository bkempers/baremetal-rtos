#ifndef STM32H7RS_HAL_I2C_H
#define STM32H7RS_HAL_I2C_H

#include "stm32h7rs_hal.h"
#include "stm32h7rsxx.h"
#include <stdint.h>

typedef struct {
    uint32_t Timing;
    uint32_t OwnAddress1;
    uint32_t AddressingMode;
    uint32_t DualAddressMode;
    uint32_t OwnAddress2;
    uint32_t OwnAddress2Masks;
    uint32_t GeneralCallMode;
    uint32_t NoStretchMode;
} I2C_Init;

typedef enum {
    HAL_I2C_STATE_RESET          = 0x00U, /*!< Peripheral is not yet Initialized         */
    HAL_I2C_STATE_READY          = 0x20U, /*!< Peripheral Initialized and ready for use  */
    HAL_I2C_STATE_BUSY           = 0x24U, /*!< An internal process is ongoing            */
    HAL_I2C_STATE_BUSY_TX        = 0x21U, /*!< Data Transmission process is ongoing      */
    HAL_I2C_STATE_BUSY_RX        = 0x22U, /*!< Data Reception process is ongoing         */
    HAL_I2C_STATE_LISTEN         = 0x28U, /*!< Address Listen Mode is ongoing            */
    HAL_I2C_STATE_BUSY_TX_LISTEN = 0x29U, /*!< Address Listen Mode and Data Transmission
                                              process is ongoing                         */
    HAL_I2C_STATE_BUSY_RX_LISTEN = 0x2AU, /*!< Address Listen Mode and Data Reception
                                              process is ongoing                         */
    HAL_I2C_STATE_ABORT = 0x60U,          /*!< Abort user request ongoing                */
} HAL_I2C_State;

typedef enum {
    HAL_I2C_MODE_NONE   = 0x00U, /*!< No I2C communication on going             */
    HAL_I2C_MODE_MASTER = 0x10U, /*!< I2C communication is in Master Mode       */
    HAL_I2C_MODE_SLAVE  = 0x20U, /*!< I2C communication is in Slave Mode        */
    HAL_I2C_MODE_MEM    = 0x40U  /*!< I2C communication is in Memory Mode       */
} HAL_I2C_Mode;

#define HAL_I2C_ERROR_NONE             (0x00000000U) /*!< No error              */
#define HAL_I2C_ERROR_BERR             (0x00000001U) /*!< BERR error            */
#define HAL_I2C_ERROR_ARLO             (0x00000002U) /*!< ARLO error            */
#define HAL_I2C_ERROR_AF               (0x00000004U) /*!< ACKF error            */
#define HAL_I2C_ERROR_OVR              (0x00000008U) /*!< OVR error             */
#define HAL_I2C_ERROR_DMA              (0x00000010U) /*!< DMA transfer error    */
#define HAL_I2C_ERROR_TIMEOUT          (0x00000020U) /*!< Timeout error         */
#define HAL_I2C_ERROR_SIZE             (0x00000040U) /*!< Size Management error */
#define HAL_I2C_ERROR_DMA_PARAM        (0x00000080U) /*!< DMA Parameter Error   */
#define HAL_I2C_ERROR_INVALID_CALLBACK (0x00000100U) /*!< Invalid Callback error */
#define HAL_I2C_ERROR_INVALID_PARAM    (0x00000200U) /*!< Invalid Parameters error  */

#define TIMING_CLEAR_MASK (0xF0FFFFFFU) /*!< I2C TIMING clear register Mask */
#define I2C_TIMEOUT_ADDR  (10000U)      /*!< 10 s  */
#define I2C_TIMEOUT_BUSY  (25U)         /*!< 25 ms */
#define I2C_TIMEOUT_DIR   (25U)         /*!< 25 ms */
#define I2C_TIMEOUT_RXNE  (25U)         /*!< 25 ms */
#define I2C_TIMEOUT_STOPF (25U)         /*!< 25 ms */
#define I2C_TIMEOUT_TC    (25U)         /*!< 25 ms */
#define I2C_TIMEOUT_TCR   (25U)         /*!< 25 ms */
#define I2C_TIMEOUT_TXIS  (25U)         /*!< 25 ms */
#define I2C_TIMEOUT_FLAG  (25U)         /*!< 25 ms */

#define I2C_NO_STARTSTOP         (0x00000000U)
#define I2C_GENERATE_STOP        (uint32_t) (0x80000000U | I2C_CR2_STOP)
#define I2C_GENERATE_START_READ  (uint32_t) (0x80000000U | I2C_CR2_START | I2C_CR2_RD_WRN)
#define I2C_GENERATE_START_WRITE (uint32_t) (0x80000000U | I2C_CR2_START)

#define I2C_ADDRESSINGMODE_7BIT  (0x00000001U)
#define I2C_ADDRESSINGMODE_10BIT (0x00000002U)

#define I2C_MEMADD_SIZE_8BIT  (0x00000001U)
#define I2C_MEMADD_SIZE_16BIT (0x00000002U)

#define I2C_RELOAD_MODE  I2C_CR2_RELOAD
#define I2C_AUTOEND_MODE I2C_CR2_AUTOEND
#define I2C_SOFTEND_MODE (0x00000000U)

#define I2C_MEM_ADD_MSB(__ADDRESS__) ((uint8_t) ((uint16_t) (((uint16_t) ((__ADDRESS__) & (uint16_t) (0xFF00U))) >> 8U)))
#define I2C_MEM_ADD_LSB(__ADDRESS__) ((uint8_t) ((uint16_t) ((__ADDRESS__) & (uint16_t) (0x00FFU))))

#define I2C_CHECK_FLAG(__ISR__, __FLAG__)    ((((__ISR__) & ((__FLAG__) & I2C_FLAG_MASK)) == ((__FLAG__) & I2C_FLAG_MASK)) ? SET : RESET)
#define I2C_CHECK_IT_SOURCE(__CR1__, __IT__) ((((__CR1__) & (__IT__)) == (__IT__)) ? SET : RESET)

#define I2C_GENERATE_START(__ADDMODE__, __ADDRESS__)                                                                                                 \
    (((__ADDMODE__) == I2C_ADDRESSINGMODE_7BIT)                                                                                                      \
         ? (uint32_t) ((((uint32_t) (__ADDRESS__) & (I2C_CR2_SADD)) | (I2C_CR2_START) | (I2C_CR2_AUTOEND)) & (~I2C_CR2_RD_WRN))                      \
         : (uint32_t) ((((uint32_t) (__ADDRESS__) & (I2C_CR2_SADD)) | (I2C_CR2_ADD10) | (I2C_CR2_START) | (I2C_CR2_AUTOEND)) & (~I2C_CR2_RD_WRN)))

#define I2C_FLAG_MASK (0x0001FFFFU)

typedef struct __I2C_Handle {
    I2C_TypeDef  *Instance;
    I2C_Init      Init;
    uint8_t      *buffPtr;
    uint16_t      xferSize;
    __IO uint16_t xferCount;
    __IO uint32_t xferOptions;
    __IO uint32_t PreviousState;

    HAL_Status (*xferISR)(struct __I2C_Handle *handle, uint32_t Flags, uint32_t Sources);

    __IO HAL_I2C_State State;
    __IO HAL_I2C_Mode  Mode;
    __IO uint32_t      ErrorCode;
    __IO uint32_t      AddrEventCount;
    __IO uint32_t      DevAddress;
    __IO uint32_t      MemAddress;
} I2C_Handle;

HAL_Status HAL_I2C_Init(I2C_Handle *handle);
HAL_Status HAL_I2C_DeInit(I2C_Handle *handle);

// polling
HAL_Status HAL_I2C_IsDeviceReady(I2C_Handle *handle, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);

// interrupt
HAL_Status HAL_I2C_Master_TX_IT(I2C_Handle *handle, uint32_t DevAddress, uint8_t *ptrData, uint16_t Size);
HAL_Status HAL_I2C_Master_RX_IT(I2C_Handle *handle, uint16_t DevAddress, uint8_t *ptrData, uint16_t Size);
// HAL_Status HAL_I2C_Slave_TX_IT(I2C_Handle *handle, uint8_t *ptrData, uint16_t Size);
// HAL_Status HAL_I2C_Slave_RX_IT(I2C_Handle *handle, uint8_t *ptrData, uint16_t Size);
HAL_Status HAL_I2C_Mem_Write_IT(I2C_Handle *handle, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *ptrData, uint16_t Size);
HAL_Status HAL_I2C_Mem_Read_IT(I2C_Handle *handle, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *ptrData, uint16_t Size);

// HAL_Status HAL_I2C_Master_Seq_TX_IT(I2C_Handle *handle, uint16_t DevAddress, uint8_t *ptrData, uint16_t Size, uint32_t TXOptions);
// HAL_Status HAL_I2C_Master_Seq_RX_IT(I2C_Handle *handle, uint16_t DevAddress, uint8_t *ptrData, uint16_t Size, uint32_t TXOptions);
//  HAL_Status HAL_I2C_Slave_Seq_TX_IT(I2C_Handle *handle, uint8_t *ptrData, uint16_t Size, uint32_t TXOptions);
//  HAL_Status HAL_I2C_Slave_Seq_RX_IT(I2C_Handle *handle, uint8_t *ptrData, uint16_t Size, uint32_t TXOptions);
HAL_Status HAL_I2C_Master_Abort_IT(I2C_Handle *handle, uint16_t DevAddress);

// dma

void HAL_I2C_EV_IRQHandler(I2C_Handle *handle);
void HAL_I2C_ER_IRQHandler(I2C_Handle *handle);
void HAL_I2C_MasterTxCpltCallback(I2C_Handle *handle);
void HAL_I2C_MasterRxCpltCallback(I2C_Handle *handle);
void HAL_I2C_ListenCpltCallback(I2C_Handle *handle);
void HAL_I2C_MemTxCpltCallback(I2C_Handle *handle);
void HAL_I2C_MemRxCpltCallback(I2C_Handle *handle);
void HAL_I2C_ErrorCallback(I2C_Handle *handle);
void HAL_I2C_AbortCpltCallback(I2C_Handle *handle);

HAL_I2C_State HAL_I2C_GetState(const I2C_Handle *handle);
HAL_I2C_Mode  HAL_I2C_GetMode(const I2C_Handle *handle);
uint32_t      HAL_I2C_GetError(const I2C_Handle *handle);

#endif // STM32H7RS_HAL_I2C_H
