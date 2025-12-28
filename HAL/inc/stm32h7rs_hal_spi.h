#ifndef STM32H7RS_HAL_SPI_H
#define STM32H7RS_HAL_SPI_H

#include "stm32h7rs_hal.h"
#include "stm32h7rsxx.h"
#include "stm32h7s3xx.h"
#include <stdint.h>

typedef struct {
    uint32_t Mode;
    uint32_t Direction;
    uint32_t DataSize;
    uint32_t CLKPolarity;
    uint32_t CLKPhase;
    uint32_t NSS;
    uint32_t BaudRatePrescaler;
    uint32_t FirstBit;
    uint32_t TIMode;
    uint32_t CRCCalculation;
    uint32_t CRCPolynomial;
    uint32_t CRCLength;
    uint32_t NSSPMode;
    uint32_t NSSPolarity;
    uint32_t FifoThreshold;
    uint32_t TxCRCInitializationPattern;
    uint32_t RxCRCInitializationPattern;
    uint32_t MasterSSIdleness;
    uint32_t MasterInterDataIdleness;
    uint32_t MasterReceiverAutoSusp;
    uint32_t MasterKeepIOState;
    uint32_t IOSwap;
    uint32_t ReadyMasterManagement;
    uint32_t ReadyPolarity;
} SPI_Init;

typedef enum {
    HAL_SPI_STATE_RESET      = 0x00UL, /*!< Peripheral not Initialized                         */
    HAL_SPI_STATE_READY      = 0x01UL, /*!< Peripheral Initialized and ready for use           */
    HAL_SPI_STATE_BUSY       = 0x02UL, /*!< an internal process is ongoing                     */
    HAL_SPI_STATE_BUSY_TX    = 0x03UL, /*!< Data Transmission process is ongoing               */
    HAL_SPI_STATE_BUSY_RX    = 0x04UL, /*!< Data Reception process is ongoing                  */
    HAL_SPI_STATE_BUSY_TX_RX = 0x05UL, /*!< Data Transmission and Reception process is ongoing */
    HAL_SPI_STATE_ERROR      = 0x06UL, /*!< SPI error state                                    */
    HAL_SPI_STATE_ABORT      = 0x07UL  /*!< SPI abort is ongoing                               */
} HAL_SPI_State;

typedef struct __SPI_Handle {
    SPI_TypeDef *Instance;
    SPI_Init     Init;

    const uint8_t *txBuffPtr;
    uint16_t       txSize;
    __IO uint16_t  txCount;
    uint8_t       *rxBuffPtr;
    uint16_t       rxSize;
    __IO uint16_t  rxCount;

    void (*rxISR)(struct __SPI_Handle *handle);
    void (*txISR)(struct __SPI_Handle *handle);

    uint32_t CRCSize;
    // dma handle
    __IO HAL_SPI_State State;
    __IO uint32_t      ErrorCode;

    void (*txCpltCallback)(struct __SPI_Handle *handle);
    void (*rxCpltCallback)(struct __SPI_Handle *handle);
    void (*txrxCpltCallback)(struct __SPI_Handle *handle);
    void (*txHalfCpltCallback)(struct __SPI_Handle *handle);
    void (*rxHalfCpltCallback)(struct __SPI_Handle *handle);
    void (*errorCallback)(struct __SPI_Handle *handle);
    void (*abortCallback)(struct __SPI_Handle *handle);
    void (*suspendCallback)(struct __SPI_Handle *handle);
} SPI_Handle;

#define SPI_LOWEND_FIFO_SIZE  8UL
#define SPI_HIGHEND_FIFO_SIZE 16UL

#define SPI_MODE_SLAVE (0x00000000UL)

#define SPI_DIRECTION_2LINES        (0x00000000UL)
#define SPI_DIRECTION_2LINES_TXONLY SPI_CFG2_COMM_0
#define SPI_DIRECTION_2LINES_RXONLY SPI_CFG2_COMM_1
#define SPI_DIRECTION_1LINE         SPI_CFG2_COMM

#define SPI_NSS_SOFT        SPI_CFG2_SSM
#define SPI_NSS_HARD_INPUT  (0x00000000UL)
#define SPI_NSS_HARD_OUTPUT SPI_CFG2_SSOE

#define SPI_NSS_POLARITY_LOW  (0x00000000UL)
#define SPI_NSS_POLARITY_HIGH SPI_CFG2_SSIOP

#define HAL_SPI_ERROR_NONE          (0x00000000UL) /*!< No error                               */
#define HAL_SPI_ERROR_MODF          (0x00000001UL) /*!< MODF error                             */
#define HAL_SPI_ERROR_CRC           (0x00000002UL) /*!< CRC error                              */
#define HAL_SPI_ERROR_OVR           (0x00000004UL) /*!< OVR error                              */
#define HAL_SPI_ERROR_FRE           (0x00000008UL) /*!< FRE error                              */
#define HAL_SPI_ERROR_DMA           (0x00000010UL) /*!< DMA transfer error                     */
#define HAL_SPI_ERROR_FLAG          (0x00000020UL) /*!< Error on RXP/TXP/DXP/FTLVL/FRLVL Flag  */
#define HAL_SPI_ERROR_ABORT         (0x00000040UL) /*!< Error during SPI Abort procedure       */
#define HAL_SPI_ERROR_UDR           (0x00000080UL) /*!< Underrun error                         */
#define HAL_SPI_ERROR_TIMEOUT       (0x00000100UL) /*!< Timeout error                          */
#define HAL_SPI_ERROR_UNKNOW        (0x00000200UL) /*!< Unknown error                          */
#define HAL_SPI_ERROR_NOT_SUPPORTED (0x00000400UL) /*!< Requested operation not supported      */

#define SPI_DATASIZE_4BIT  (0x00000003UL)
#define SPI_DATASIZE_5BIT  (0x00000004UL)
#define SPI_DATASIZE_6BIT  (0x00000005UL)
#define SPI_DATASIZE_7BIT  (0x00000006UL)
#define SPI_DATASIZE_8BIT  (0x00000007UL)
#define SPI_DATASIZE_9BIT  (0x00000008UL)
#define SPI_DATASIZE_10BIT (0x00000009UL)
#define SPI_DATASIZE_11BIT (0x0000000AUL)
#define SPI_DATASIZE_12BIT (0x0000000BUL)
#define SPI_DATASIZE_13BIT (0x0000000CUL)
#define SPI_DATASIZE_14BIT (0x0000000DUL)
#define SPI_DATASIZE_15BIT (0x0000000EUL)
#define SPI_DATASIZE_16BIT (0x0000000FUL)
#define SPI_DATASIZE_17BIT (0x00000010UL)
#define SPI_DATASIZE_18BIT (0x00000011UL)
#define SPI_DATASIZE_19BIT (0x00000012UL)
#define SPI_DATASIZE_20BIT (0x00000013UL)
#define SPI_DATASIZE_21BIT (0x00000014UL)
#define SPI_DATASIZE_22BIT (0x00000015UL)
#define SPI_DATASIZE_23BIT (0x00000016UL)
#define SPI_DATASIZE_24BIT (0x00000017UL)
#define SPI_DATASIZE_25BIT (0x00000018UL)
#define SPI_DATASIZE_26BIT (0x00000019UL)
#define SPI_DATASIZE_27BIT (0x0000001AUL)
#define SPI_DATASIZE_28BIT (0x0000001BUL)
#define SPI_DATASIZE_29BIT (0x0000001CUL)
#define SPI_DATASIZE_30BIT (0x0000001DUL)
#define SPI_DATASIZE_31BIT (0x0000001EUL)
#define SPI_DATASIZE_32BIT (0x0000001FUL)

HAL_Status HAL_SPI_Init(SPI_Handle *handle);
HAL_Status HAL_SPI_DeInit(SPI_Handle *handle);

// polling tx/rx

HAL_Status HAL_SPI_Transmit_IT(SPI_Handle *handle, const uint8_t *ptrData, uint16_t Size);
HAL_Status HAL_SPI_Receive_IT(SPI_Handle *handle, uint8_t *ptrData, uint16_t Size);
HAL_Status HAL_SPI_TransmitReceive_IT(SPI_Handle *handle, const uint8_t *txData, uint8_t *rxData, uint16_t Size);

void HAL_SPI_IRQHandler(SPI_Handle *handle);
void HAL_SPI_TxCpltCallback(SPI_Handle *handle);
void HAL_SPI_RxCpltCallback(SPI_Handle *handle);
void HAL_SPI_TxRxCpltCallback(SPI_Handle *handle);
void HAL_SPI_TxHalfCpltCallback(SPI_Handle *handle);
void HAL_SPI_RxHalfCpltCallback(SPI_Handle *handle);
void HAL_SPI_TxRxHalfCpltCallback(SPI_Handle *handle);
void HAL_SPI_ErrorCallback(SPI_Handle *handle);
void HAL_SPI_AbortCpltCallback(SPI_Handle *handle);
void HAL_SPI_SuspendCallback(SPI_Handle *handle);

// dma tx/rx

HAL_Status HAL_SPI_Abort(SPI_Handle *handle);
HAL_Status HAL_SPI_AbortIT(SPI_Handle *handle);

HAL_SPI_State HAL_SPI_GetState(const SPI_Handle *handle);
uint32_t      HAL_SPI_GetError(const SPI_Handle *handle);

#endif // STM32H7RS_HAL_SPI_H
