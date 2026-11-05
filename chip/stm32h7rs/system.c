#include <stdio.h>
#include <string.h>

#include "system.h"
#include "cmsis_device.h"

__weak void system_error_handle(void)
{
    __disable_irq();
    while (1) {
        __WFI();
    }
}

#ifdef USE_FULL_ASSERT
/* Failure policy for the ST LL drivers' assert_param(). Kept in statics so a
 * debugger can recover the site; nothing here may depend on printf. */
volatile uint8_t  *assert_file;
volatile uint32_t  assert_line;

void assert_failed(uint8_t *file, uint32_t line)
{
    assert_file = file;
    assert_line = line;
    system_error_handle();
}
#endif
