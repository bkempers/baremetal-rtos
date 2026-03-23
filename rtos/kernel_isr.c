#include "kernel.h"

__weak void HAL_IncTick(void) {}

void SysTick_Handler(void) {
    // increment the HAL tick
    HAL_IncTick();

    kernel_tick();
}

__attribute__((naked)) void PendSV_Handler(void) {
    __asm volatile (
        "CPSID   I                      \n"

        /* ── Skip save on first switch ─────────────────────────────────
           On first entry current_tcb->stack_ptr already has the correct
           initial frame from stack_init — saving PSP here would corrupt it */
        "LDR     R0, =kernel_first_switch   \n"
        "LDRB    R1, [R0]                   \n"
        "CBZ     R1, save_context           \n"  // 0 = not first, go save
        "MOV     R1, #0                     \n"
        "STRB    R1, [R0]                   \n"  // clear flag
        "B       load_next                  \n"  // skip save entirely

        /* ── SAVE CURRENT TASK ──────────────────────────────────────────*/
        "save_context:                  \n"
        "MRS     R0, PSP                \n"

        "TST     LR, #0x10              \n"
        "IT      EQ                     \n"
        "VSTMDBEQ R0!, {S16-S31}        \n"

        "STMDB   R0!, {R4-R11, LR}      \n"

        "LDR     R1, =current_tcb       \n"
        "LDR     R2, [R1]               \n"
        "STR     R0, [R2, #0]           \n"
        "B       switch_tcb             \n"

        /* ── LOAD NEXT TASK ─────────────────────────────────────────────*/
        "load_next:                     \n"
        "LDR     R1, =current_tcb       \n"
        "LDR     R2, [R1]               \n"

        "switch_tcb:                    \n"
        "LDR     R3, [R2, #4]           \n"
        "STR     R3, [R1]               \n"
        "LDR     R0, [R3, #0]           \n"

        "LDMIA   R0!, {R4-R11, LR}      \n"

        "TST     LR, #0x10              \n"
        "IT      EQ                     \n"
        "VLDMIAEQ R0!, {S16-S31}        \n"

        "MSR     PSP, R0                \n"
        "CPSIE   I                      \n"
        "BX      LR                     \n"
        ::: "memory"
    );
}
