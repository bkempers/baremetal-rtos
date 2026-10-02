#include "arch.h"
#include "core_cm7.h"

__weak void kernel_tick(void) {}

int arch_systick_start(uint32_t core_hz, uint32_t tick_hz, uint32_t prio)
{
    if (prio >= (1UL << __NVIC_PRIO_BITS)) {
        return -1;
    }
    if (SysTick_Config(core_hz / tick_hz) != 0U) {
        return -1;
    }
    NVIC_SetPriority(SysTick_IRQn, prio);
    return 0;
}

void SysTick_Handler(void)
{
    kernel_tick();
}
