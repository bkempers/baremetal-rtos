#ifndef ARCH_H
#define ARCH_H

#include <stdint.h>

int  arch_systick_start(uint32_t core_hz, uint32_t tick_hz, uint32_t prio);

#endif // ARCH_H
