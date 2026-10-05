#ifndef HAL_CLOCK_H
#define HAL_CLOCK_H

#include <stdint.h>

//TODO: this needs to be more generic and dynamic for different clock systems
struct clock_cfg;
struct clock_subsys;

void clock_init(const struct clock_cfg* cfg);
uint32_t clock_get_sysclk(void);
uint32_t clock_get_subsysclk(struct clock_subsys* clk);

#endif // HAL_CLOCK_H
