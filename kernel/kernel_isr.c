/* Both handlers that lived here are Cortex-M specific and moved to
 * arch/arm/cortex_m/cortex_m.c:
 *
 *   SysTick_Handler  - upcalls the weak kernel_tick()
 *   PendSV_Handler   - the context switch asm
 *
 * This file is no longer built (see kernel/CMakeLists.txt) and can be
 * git rm'd; it is left here only so the move is obvious in review. */
