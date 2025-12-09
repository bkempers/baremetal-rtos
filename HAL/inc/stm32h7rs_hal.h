#ifndef STM32H7RS_HAL_H
#define STM32H7RS_HAL_H

#include "stm32h7rsxx.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Weak attribute for GCC
#if defined(__GNUC__)
#define __weak   __attribute__((weak))
#define __packed __attribute__((__packed__))
// #define __aligned(x) __attribute__((aligned(x)))
#elif defined(__ICCARM__) // IAR compiler
#define __weak       __weak
#define __packed     __packed
#define __aligned(x) _Pragma("data_alignment=" #x)
#elif defined(__CC_ARM) // Keil compiler
#define __weak       __weak
#define __packed     __packed
#define __aligned(x) __align(x)
#else
#warning "Unsupported compiler"
#define __weak
#define __packed
#define __aligned(x)
#endif

#define UNUSED(X) (void) X /* To avoid gcc/g++ warnings */

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

HAL_Status HAL_Init(void);
HAL_Status HAL_DeInit(void);

typedef enum {
    HAL_TICK_FREQ_10HZ    = 100U,
    HAL_TICK_FREQ_100HZ   = 10U,
    HAL_TICK_FREQ_1KHZ    = 1U,
    HAL_TICK_FREQ_DEFAULT = HAL_TICK_FREQ_1KHZ
} HAL_TickFreq;

#define HAL_IS_BIT_SET(REG, BIT) (((REG) & (BIT)) == (BIT))
#define HAL_IS_BIT_CLR(REG, BIT) (((REG) & (BIT)) == 0U)

#define HAL_MAX_DELAY 0xFFFFFFFFU

extern __IO uint32_t uwTick;
extern uint32_t      uwTickPrio;
extern HAL_TickFreq  uwTickFreq;

HAL_Status HAL_InitTick(uint32_t tick_priority);
void       HAL_IncTick(void);
uint32_t   HAL_GetTick(void);
void       HAL_DelayMS(uint32_t milliseconds);

#endif
