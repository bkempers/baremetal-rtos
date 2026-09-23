#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7rs_hal_gpio.h"
#include "stm32h7rs_hal_rcc.h"
#include "stm32h7rs_hal_tim.h"

#include "system.h"

typedef void (*TaskFunc_t)(void);

typedef struct {
    TaskFunc_t func;
    uint32_t   period_ms;
    uint32_t   last_run;
    bool       enabled;
} PeriodicTask_t;

SYS_Status Task_Scheduler_Init(void);
bool       Scheduler_AddTask(TaskFunc_t func, uint32_t period_ms);

#endif
