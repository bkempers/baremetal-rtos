#ifndef CONFIG_H
#define CONFIG_H

#include "stm32h7s3xx.h"

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

#define NUM_THREADS 4
#define STACK_SIZE 100

#define IDLE_STACK_WORDS    64u
#define TICK_RATE_HZ        1000u
#define STACK_FILL_PATTERN  0xDEADBEEFu
#define TICK_PRIORITY       15u

#endif // CONFIG_H
