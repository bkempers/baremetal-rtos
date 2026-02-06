#ifndef STM32H7RS_HAL_XSPI_H
#define STM32H7RS_HAL_XSPI_H

#include "stm32h7rsxx.h"
#include "stm32h7rs_hal.h"
#include <stdint.h>

typedef enum {
    HAL_XSPI_STATE_RESET      = 0x00U,
    HAL_XSPI_STATE_READY      = 0x01U,
    HAL_XSPI_STATE_BUSY       = 0x02U,
    HAL_XSPI_STATE_ERROR      = 0x04U
} HAL_XSPI_State;

typedef enum {
    HAL_XSPI_MEMTYPE_MICRON   = 0x00U,
    HAL_XSPI_MEMTYPE_MACRONIX = 0x01U,
    HAL_XSPI_MEMTYPE_APMEM    = 0x02U
} HAL_XSPI_Memory;

typedef enum {
    HAL_XSPI_FUNCTIONAL_MODE_INDIRECT_WRITE = 0x00U,
    HAL_XSPI_FUNCTIONAL_MODE_INDIRECT_READ  = 0x01U,
    HAL_XSPI_FUNCTIONAL_MODE_AUTO_POLLING   = 0x02U,
    HAL_XSPI_FUNCTIONAL_MODE_MEMORY_MAPPED  = 0x03U
} HAL_XSPI_FunctionalMode;

typedef enum {
    HAL_XSPI_IO_MODE_NONE   = 0x00U,
    HAL_XSPI_IO_MODE_SINGLE = 0x01U,
    HAL_XSPI_IO_MODE_DUAL   = 0x02U,
    HAL_XSPI_IO_MODE_QUAD   = 0x03U,
    HAL_XSPI_IO_MODE_OCTAL  = 0x04U
} HAL_XSPI_IOMode;

typedef enum {
    HAL_XSPI_DTR_DISABLE = 0x00U,
    HAL_XSPI_DTR_ENABLE  = 0x01U
} HAL_XSPI_DTRMode;

typedef struct {
    uint32_t ClockPrescaler;           /* Clock prescaler (0-255) */
    uint32_t FifoThreshold;            /* FIFO threshold (0-31) */
    uint32_t MemorySize;               /* Device size (0-31) -> 2^(SIZE+1) bytes */
    uint32_t ChipSelectHighTime;       /* CS high time in cycles */
    HAL_XSPI_Memory MemoryType;        /* Memory type */
} XSPI_Init;

/* XSPI Regular Command Structure */
typedef struct {
    uint32_t Instruction;                       /* Command code */
    uint32_t InstructionSize;                   /* Instruction width (1-4 bytes) */
    HAL_XSPI_IOMode InstructionMode;     /* Instruction IO mode */
    HAL_XSPI_DTRMode InstructionDTRMode; /* DTR mode for instruction */
    
    uint32_t Address;                           /* Address to send */
    uint32_t AddressSize;                       /* Address width (1-4 bytes) */
    HAL_XSPI_IOMode AddressMode;         /* Address IO mode */
    HAL_XSPI_DTRMode AddressDTRMode;     /* DTR mode for address */
    
    uint32_t AlternateBytesSize;                /* Alternate bytes width */
    HAL_XSPI_IOMode AlternateBytesMode;  /* Alternate bytes IO mode */
    
    uint32_t DataLength;                        /* Number of data bytes */
    HAL_XSPI_IOMode DataMode;            /* Data IO mode */
    HAL_XSPI_DTRMode DataDTRMode;        /* DTR mode for data */
    
    uint32_t DummyCycles;                       /* Number of dummy cycles */
    uint32_t DQSMode;                           /* DQS enable/disable */
} XSPI_RegularCmd;

/* XSPI Handle */
typedef struct {
    XSPI_TypeDef *Instance;                     /* XSPI peripheral instance */
    XSPI_Init Init;                      /* Init parameters */
    HAL_XSPI_State State;                /* State */
    uint32_t ErrorCode;                         /* Error code */
} XSPI_Handle;

/* Error Codes */
#define HAL_XSPI_ERROR_NONE           0x00000000U
#define HAL_XSPI_ERROR_TIMEOUT        0x00000001U
#define HAL_XSPI_ERROR_TRANSFER       0x00000002U
#define HAL_XSPI_ERROR_INVALID_PARAM  0x00000004U

/* Memory Size Macros */
#define HAL_XSPI_SIZE_256MB  27U  /* 2^28 = 256MB */
#define HAL_XSPI_SIZE_128MB  26U
#define HAL_XSPI_SIZE_64MB   25U

/* Function Prototypes */
HAL_Status HAL_XSPI_Init(XSPI_Handle *handle);
HAL_Status HAL_XSPI_DeInit(XSPI_Handle *handle);
HAL_Status HAL_XSPI_Command(XSPI_Handle *handle, XSPI_RegularCmd *cmd, uint32_t timeout);
HAL_Status HAL_XSPI_Transmit(XSPI_Handle *handle, uint8_t *pData, uint32_t timeout);
HAL_Status HAL_XSPI_Receive(XSPI_Handle *handle, uint8_t *pData, uint32_t timeout);
HAL_Status HAL_XSPI_MemoryMapped(XSPI_Handle *handle, XSPI_RegularCmd *cmd);

/* Helper for MX25UM initialization */
HAL_Status HAL_XSPI_MX25UM_Init(XSPI_Handle *handle);

#endif /* STM32H7RS_HAL_XSPI_H */
