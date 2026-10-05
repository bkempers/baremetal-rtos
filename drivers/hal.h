#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "compiler.h"

typedef enum {
    HAL_ERROR   = 0x0,
    HAL_OK      = 0x1,
    HAL_BUSY    = 0x2,
    HAL_TIMEOUT = 0x3
} HAL_Status;

typedef enum {
    HAL_UNLOCKED = 0x0,
    HAL_LOCKED   = 0x1
} HAL_Lock;

#define HAL_IS_BIT_SET(REG, BIT) (((REG) & (BIT)) == (BIT))
#define HAL_IS_BIT_CLR(REG, BIT) (((REG) & (BIT)) == 0U)

#define HAL_MAX_DELAY 0xFFFFFFFFU

/* Unrecoverable hardware-bringup failure. Defined by the board, since only the
 * board knows how to signal it (LED, pin, breakpoint). Never returns. */
void system_error_handle(void);

#endif
