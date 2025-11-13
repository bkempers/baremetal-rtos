#include "stm32h7rs_hal.h"
#include "stm32h7s3xx.h"

__IO uint32_t uwTick;
uint32_t uwTickPrio            = (1UL << __NVIC_PRIO_BITS); /* Invalid priority */
HAL_TickFreq uwTickFreq = HAL_TICK_FREQ_DEFAULT;  /* 1KHz */

HAL_Status HAL_Init(void)
{
    HAL_Status ret = HAL_OK;
    if (HAL_InitTick(HAL_TICK_FREQ_DEFAULT) != HAL_OK) {
        ret = HAL_ERROR;
    }
    return ret;
}

HAL_Status HAL_DeInit(void)
{
    // TODO: implement clock reset in RCC
    return HAL_OK;
}

HAL_Status HAL_InitTick(uint32_t tick_priority)
{
    HAL_Status ret = HAL_OK;
    if (SysTick_Config(SystemCoreClock / (1000U / (uint32_t)uwTickFreq)) == 0U)
    {
      /* Configure the SysTick IRQ priority */
      if (tick_priority < (1UL << __NVIC_PRIO_BITS))
      {
        NVIC_SetPriority(SysTick_IRQn, tick_priority);
        uwTickPrio = tick_priority;
      }
      else
      {
        ret = HAL_ERROR;
      }
    }
    else
    {
      ret = HAL_ERROR;
    }
    return ret;
}

__weak void HAL_IncTick(void)
{
    uwTick += (uint32_t)uwTickFreq;
}

__weak uint32_t HAL_GetTick(void)
{
    return uwTick;
}
