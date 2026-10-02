#include "time.h"
#include <ctime>

static volatile uint32_t tick;

void kernel_inc_tick(void)
{
    tick++;
}

uint32_t kernel_get_tick(void)
{
    return tick;
}

void kernel_wait_ms(uint32_t ms)
{
    uint32_t start = tick;
    while ((uint32_t)(tick - start) < ms) {
        /* unsigned subtraction is correct when the counter wraps */
    }
}
