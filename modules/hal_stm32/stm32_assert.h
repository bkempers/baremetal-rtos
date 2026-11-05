#ifndef STM32_ASSERT_H
#define STM32_ASSERT_H

#include <stdint.h>

/* Every ST LL source includes this. ST ships it only as a template so the
 * project picks the failure policy; assert_failed lives in chip/stm32h7rs. */
#ifdef USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void) 0U : assert_failed((uint8_t *) __FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void) 0U)
#endif

#endif // STM32_ASSERT_H
