#ifndef KERNEL_TIME_H
#define KERNEL_TIME_H

#include <stdint.h>

/* The kernel's single tick source. arch's SysTick_Handler upcalls
 * kernel_tick(); everything time-related reads the counter from here. */

/* Called from the SysTick ISR. Advances the tick and drives the round-robin
 * switch — not meant to be called from thread context. */
void kernel_tick(void);

uint32_t kernel_get_tick(void);

/* Spin until ms ticks have elapsed, without yielding. For use before the
 * scheduler is running; wasteful once it is. */
void kernel_wait_ms(uint32_t ms);

/* Yield to the other threads until ms ticks have elapsed. */
void kernel_delay_ms(uint32_t ms);

#endif // KERNEL_TIME_H
