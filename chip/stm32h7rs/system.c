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
