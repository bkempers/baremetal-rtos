#include "stm32h7rs_hal.h"
#include "stm32h7s3xx.h"

volatile uint32_t tick;
uint32_t tickPrio            = (1UL << __NVIC_PRIO_BITS); /* Invalid priority */
HAL_TickFreq tickFreq = HAL_TICK_FREQ_DEFAULT;  /* 1KHz */

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
    if (SysTick_Config(SystemCoreClock / (1000U / (uint32_t)tickFreq)) == 0U)
    {
      /* Configure the SysTick IRQ priority */
      if (tick_priority < (1UL << __NVIC_PRIO_BITS))
      {
        NVIC_SetPriority(SysTick_IRQn, tick_priority);
        tickPrio = tick_priority;
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

void SysTick_Handler(void)
{
    tick++;
}

__weak void HAL_IncTick(void)
{
    tick += (uint32_t)tickFreq;
}

__weak uint32_t HAL_GetTick(void)
{
    return tick;
}

void HAL_DelayMS(uint32_t milliseconds)
{
  uint32_t start = tick;
  uint32_t end = start + milliseconds;

  if (end < start) // overflow
  {
    while (tick > start); // wait for ticks to wrap
  }

  while (tick < end);
}
