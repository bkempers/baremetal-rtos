#include "time.h"

#include "arch.h"
#include "kernel.h"

static volatile uint32_t tick;

void kernel_tick(void)
{
    tick++;

    /* Round-robin: every tick is a switch point. */
    arch_sched_trigger();
}

uint32_t kernel_get_tick(void)
{
    return tick;
}

void kernel_wait_ms(uint32_t ms)
{
    uint32_t start = tick;
    while ((uint32_t) (tick - start) < ms) {
        /* unsigned subtraction is correct when the counter wraps */
    }
}

void kernel_delay_ms(uint32_t ms)
{
    uint32_t start = tick;
    while ((uint32_t) (tick - start) < ms) {
        kernel_yield();
    }
}
