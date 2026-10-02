#ifndef TIME_H
#define TIME_H

#include <stdint.h>

void kernel_inc_tick(void);
uint32_t kernel_get_tick(void);
void kernel_wait_ms(uint32_t ms);

#endif // TIME_H
