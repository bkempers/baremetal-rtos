#ifndef CMSIS_DEVICE_H
#define CMSIS_DEVICE_H

/* The one place a layer can reach the CMSIS core intrinsics without naming an
 * SoC. CMSIS pulls core_cm7.h in through the device header, which is what
 * supplies IRQn_Type and __NVIC_PRIO_BITS, so arch/ cannot include the core
 * header on its own. Pointing this shim at a different device header is the
 * whole port, and it lives beside the cmsis target because that is already
 * where the device include dir and STM32H7S3xx are selected. */
#include "stm32h7rsxx.h"

#endif
