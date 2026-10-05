#ifndef ARCH_H
#define ARCH_H

#include <stdint.h>

void arch_init(void);
void arch_halt(void);
int arch_systick_start(uint32_t core_hz, uint32_t tick_hz, uint32_t prio);
void arch_irq_disable(void);
void arch_irq_enable(void);
void arch_idle(void);
void arch_sched_prio_set(uint32_t prio);
void arch_sched_trigger(void);
void arch_sched_start(void);

#endif // ARCH_H
