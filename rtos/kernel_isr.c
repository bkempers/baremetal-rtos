#include "kernel.h"

__weak void HAL_IncTick(void) {}

void SysTick_Handler(void) {
    // increment the HAL tick
    HAL_IncTick();

    kernel_tick();
}

__attribute__((naked)) void PendSV_Handler(void) {
    __asm volatile (
        "CPSID   I                  \n"
        "MRS     R0, PSP            \n"
        "STMDB   R0!, {R4-R11}      \n"

        "LDR     R1, =current_tcb   \n"
        "LDR     R2, [R1]           \n"
        "STR     R0, [R2, #0]       \n"

        "LDR     R3, [R2, #4]       \n"  // current_tcb->next
        "STR     R3, [R1]           \n"  // current_tcb = next
        "LDR     R0, [R3, #0]       \n"  // next->stack_ptr

        "LDMIA   R0!, {R4-R11}      \n"
        "MSR     PSP, R0            \n"

        "CPSIE   I                  \n"
        "LDR     LR, =0xFFFFFFFD    \n"
        "BX      LR                 \n"
        ::: "memory"
    );
}
