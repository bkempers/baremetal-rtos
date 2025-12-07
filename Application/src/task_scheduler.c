#include "stm32h7rs_hal_rcc.h"
#include "stm32h7rs_hal_tim.h"
#include "stm32h7s3xx.h"

#include <stdio.h>

#include "console.h"
#include "system.h"
#include "task_scheduler.h"

#define MAX_TASKS 8

static TIM_Handle        tim2;
static PeriodicTask_t    tasks[MAX_TASKS];
static uint32_t          task_count      = 0;
static volatile uint32_t scheduler_ticks = 0;

// This gets called every 1ms by TIM6
static void scheduler_tick_callback(TIM_Handle *handle)
{
    scheduler_ticks++;

    // Check all tasks
    for (uint32_t i = 0; i < task_count; i++) {
        if (!tasks[i].enabled)
            continue;

        if ((scheduler_ticks - tasks[i].last_run) >= tasks[i].period_ms) {
            tasks[i].last_run = scheduler_ticks;

            // Execute task
            if (tasks[i].func) {
                tasks[i].func();
            }
        }
    }
}

SYS_Status Task_Scheduler_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* Configure TIM Handle */
    tim2.Instance               = TIM2;
    tim2.Init.Prescaler         = 6400 - 1; // 64,000,000 / 6400
    tim2.Init.Period            = 10 - 1;
    tim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    tim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    tim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    tim2.Init.RepetitionCounter = 0x00;

    tim2.periodElapsedCallback = HAL_TIM_periodElapsedCallback;

    if (HAL_TIM_Init(&tim2) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_SetCounter(&tim2, 0);

    if (HAL_TIM_Start_IT(&tim2) != HAL_OK) {
        Error_Handler();
    }

    __NVIC_SetPriority(TIM2_IRQn, 8);
    __NVIC_EnableIRQ(TIM2_IRQn);

    INFO("Starting task scheduler with %u tasks.", task_count);
    return SYS_OK;
}

void TIM2_IRHandler()
{
    HAL_TIM_IRQHandler(&tim2);
}

void HAL_TIM_periodElapsedCallback(TIM_Handle *handle)
{
    scheduler_tick_callback(handle);
}

bool Scheduler_AddTask(TaskFunc_t func, uint32_t period_ms)
{
    if (task_count >= MAX_TASKS || !func) {
        return false;
    }

    tasks[task_count].func      = func;
    tasks[task_count].period_ms = period_ms;
    tasks[task_count].last_run  = 0;
    tasks[task_count].enabled   = true;

    task_count++;
    return true;
}
