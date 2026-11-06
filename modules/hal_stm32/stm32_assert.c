#include "stm32_assert.h"

#ifdef USE_FULL_ASSERT

/* Declared locally rather than including drivers/hal.h, so stm32_ll stays
 * independent of the driver layer. Resolved at the final link. */
void system_error_handle(void);

/* Lives in this archive, not chip's: ld rescans a single archive until it
 * settles, so the LL sources resolve this regardless of link order. */
volatile uint8_t *assert_file;
volatile uint32_t assert_line;

void assert_failed(uint8_t *file, uint32_t line)
{
    assert_file = file;
    assert_line = line;
    system_error_handle();
}

#endif
