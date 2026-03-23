#include "kernel.h"

__weak void HAL_IncTick(void) {}

void SysTick_Handler(void) {
    // increment the HAL tick
    HAL_IncTick();

    kernel_tick();
}

__attribute__((naked)) void PendSV_Handler(void) {
    __asm volatile (
        "CPSID   I                          \n"

        /* ── First switch: restore current_tcb directly, no advance ── */
        "LDR     R0, =kernel_first_switch   \n"
        "LDRB    R1, [R0]                   \n"
        "CBZ     R1, save_context           \n"
        "MOV     R1, #0                     \n"
        "STRB    R1, [R0]                   \n"
        "B       first_load                 \n"  // ← goes to dedicated path

        /* ── Save current task ─────────────────────────────────────── */
        "save_context:                      \n"
        "MRS     R0, PSP                    \n"
        "TST     LR, #0x10                  \n"
        "IT      EQ                         \n"
        "VSTMDBEQ R0!, {S16-S31}            \n"
        "STMDB   R0!, {R4-R11, LR}          \n"
        "LDR     R1, =current_tcb           \n"
        "LDR     R2, [R1]                   \n"
        "STR     R0, [R2, #0]               \n"

        /* ── Advance to next task ───────────────────────────────────── */
        "LDR     R3, [R2, #4]               \n"  // R3 = current_tcb->next
        "STR     R3, [R1]                   \n"  // current_tcb = next
        "LDR     R0, [R3, #0]               \n"  // R0 = next->stack_ptr
        "B       restore_context            \n"

        /* ── First load: restore current_tcb as-is, no next advance ── */
        "first_load:                        \n"
        "LDR     R1, =current_tcb           \n"
        "LDR     R2, [R1]                   \n"  // R2 = &tcbs[0]
        "LDR     R0, [R2, #0]               \n"  // R0 = tcbs[0].stack_ptr

        /* ── Restore ────────────────────────────────────────────────── */
        "restore_context:                   \n"
        "LDMIA   R0!, {R4-R11, LR}          \n"
        "TST     LR, #0x10                  \n"
        "IT      EQ                         \n"
        "VLDMIAEQ R0!, {S16-S31}            \n"
        "MSR     PSP, R0                    \n"
        "CPSIE   I                          \n"
        "BX      LR                         \n"
        ::: "memory"
    );
}
