#ifndef CONFIG_H
#define CONFIG_H

#include "stm32h7s3xx.h"

#include <stdint.h>

#define NUM_THREADS 4
#define STACK_SIZE 100

#define IDLE_STACK_WORDS    64u
#define TICK_RATE_HZ        1000u
#define STACK_FILL_PATTERN  0xDEADBEEFu
#define TICK_PRIORITY       15u

#endif // CONFIG_H
