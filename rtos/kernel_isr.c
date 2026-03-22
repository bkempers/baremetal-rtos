#include "kernel.h"

__weak void HAL_IncTick(void) {}

void SysTick_Handler(void) {
    // increment the HAL tick
    HAL_IncTick();

    kernel_tick();
}

__attribute__((naked)) void PendSV_Handler(void) {
    __asm volatile (

        /* ── SAVE CURRENT TASK ─────────────────────────────────────────
           Hardware already saved: xPSR, PC, LR, R12, R3-R0
           (extended frame also has S0-S15, FPSCR pushed by hardware)  */

        "CPSID   I                  \n"
        "MRS     R0, PSP            \n"

        /* Check EXC_RETURN bit 4 — 0 means FPU was active             */
        "TST     LR, #0x10          \n"
        "IT      EQ                 \n"
        "VSTMDBEQ R0!, {S16-S31}    \n"  // push FPU high regs if needed

        "STMDB   R0!, {R4-R11, LR}  \n"  // push callee-saved + EXC_RETURN
                                          // LR carries the frame type info
                                          // needed on restore

        /* Save updated SP into current TCB->stack_ptr (offset 0)      */
        "LDR     R1, =current_tcb   \n"
        "LDR     R2, [R1]           \n"
        "STR     R0, [R2, #0]       \n"

        /* ── LOAD NEXT TASK ────────────────────────────────────────────
           current_tcb->next is at offset 4                            */

        "LDR     R3, [R2, #4]       \n"  // R3 = next TCB
        "STR     R3, [R1]           \n"  // current_tcb = next
        "LDR     R0, [R3, #0]       \n"  // R0 = next task's stack_ptr

        "LDMIA   R0!, {R4-R11, LR}  \n"  // pop callee-saved + EXC_RETURN
                                          // LR now has next task's frame type

        /* Check if next task needs FPU restore                        */
        "TST     LR, #0x10          \n"
        "IT      EQ                 \n"
        "VLDMIAEQ R0!, {S16-S31}    \n"  // pop FPU high regs if needed

        "MSR     PSP, R0            \n"
        "CPSIE   I                  \n"
        "BX      LR                 \n"  // LR = correct EXC_RETURN for next task
        ::: "memory"
    );
}
