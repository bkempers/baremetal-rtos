#include "arch.h"
#include "cmsis_device.h"

void kernel_tick(void);

void arch_init(void)
{
    /* CP10/CP11 full access — enables the FPU. */
    SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2)));
    __DSB();
    __ISB();
}

void arch_halt(void)
{
    __disable_irq();
    while (1) {
        __WFI();
    }
}

int arch_systick_start(uint32_t core_hz, uint32_t tick_hz, uint32_t prio)
{
    if (prio >= (1UL << __NVIC_PRIO_BITS)) {
        return -1;
    }
    if (SysTick_Config(core_hz / tick_hz) != 0U) {
        return -1;
    }
    NVIC_SetPriority(SysTick_IRQn, prio);
    return 0;
}

void arch_irq_disable(void)
{
    __disable_irq();
}

void arch_irq_enable(void)
{
    __enable_irq();
}

void arch_idle(void)
{
    __WFI();
}

void arch_sched_prio_set(uint32_t prio)
{
    NVIC_SetPriority(PendSV_IRQn, prio);
}

void arch_sched_trigger(void)
{
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
    __DSB(); /* ensure the write lands before the caller enables interrupts */
}

void arch_sched_start(void)
{
    /* Without this every thread would run on MSP — no kernel/thread stack
     * separation. The value is a placeholder; the first PendSV overwrites PSP
     * from the incoming thread's tcb. */
    __set_PSP(__get_MSP());
    __set_CONTROL(__get_CONTROL() | 0x02u); /* SPSEL: Thread mode uses PSP */
    __ISB();                                /* flush pipeline after CONTROL write */

    arch_sched_trigger();
}

void SysTick_Handler(void)
{
    kernel_tick();
}

//TODO: need to better understand this...
/* ─────────────────────────────────────────────────────────────────────────────
 * Context switch.
 *
 * This is pure Cortex-M, so it lives here rather than in the kernel. It is
 * coupled to the kernel only through a layout contract, which the kernel must
 * keep in step with this code:
 *
 *   current_tcb         - pointer to the running thread's tcb
 *   current_tcb[0x0]    - uint32_t *stack_ptr   (saved PSP)
 *   current_tcb[0x4]    - struct tcb *next      (round-robin successor)
 *   kernel_first_switch - uint8_t, nonzero until the first switch completes
 *
 * The symbols are resolved by the linker; no kernel header is included, which
 * is what keeps the dependency pointing one way.
 *
 * Living here also means a strong PendSV_Handler is always linked: something
 * always references arch_init(), so this object is always pulled out of
 * libarch.a and overrides startup.c's weak Default_Handler stub. In a static
 * archive whose only symbol was PendSV_Handler, it silently was not.
 * ──────────────────────────────────────────────────────────────────────────── */
__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile("CPSID   I                          \n"

                   /* ── First switch: restore current_tcb directly, no advance ── */
                   "LDR     R0, =kernel_first_switch   \n"
                   "LDRB    R1, [R0]                   \n"
                   "CBZ     R1, save_context           \n"
                   "MOV     R1, #0                     \n"
                   "STRB    R1, [R0]                   \n"
                   "B       first_load                 \n" // ← goes to dedicated path

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
                   "LDR     R3, [R2, #4]               \n" // R3 = current_tcb->next
                   "STR     R3, [R1]                   \n" // current_tcb = next
                   "LDR     R0, [R3, #0]               \n" // R0 = next->stack_ptr
                   "B       restore_context            \n"

                   /* ── First load: restore current_tcb as-is, no next advance ── */
                   "first_load:                        \n"
                   "LDR     R1, =current_tcb           \n"
                   "LDR     R2, [R1]                   \n" // R2 = &tcbs[0]
                   "LDR     R0, [R2, #0]               \n" // R0 = tcbs[0].stack_ptr

                   /* ── Restore ────────────────────────────────────────────────── */
                   "restore_context:                   \n"
                   "LDMIA   R0!, {R4-R11, LR}          \n"
                   "TST     LR, #0x10                  \n"
                   "IT      EQ                         \n"
                   "VLDMIAEQ R0!, {S16-S31}            \n"
                   "MSR     PSP, R0                    \n"
                   "CPSIE   I                          \n"
                   "BX      LR                         \n" ::
                       : "memory");
}
