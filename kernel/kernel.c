#include "kernel.h"
#include "config.h"

#include "arch.h"
#include "hal.h"
#include "hal_clock.h"

struct tcb     tcbs[NUM_THREADS + 1];
static uint8_t thread_count = 0;

uint8_t kernel_first_switch = 1;

static uint32_t idle_stack[IDLE_STACK_WORDS];

/* Read by arch's PendSV_Handler, which relies on stack_ptr being at offset 0
 * and next at offset 4 of struct tcb. Keep that in step with the contract
 * documented in arch/arm/cortex_m/cortex_m.c. */
struct tcb *current_tcb;

static void task_exit_trap(void)
{
    arch_halt();
}

static void idle_task(void)
{
    while (1) {
        // Check every task's stack bottom for overflow
        for (uint8_t i = 0; i < thread_count; i++) {
            if (tcbs[i].stack_base[0] != STACK_FILL_PATTERN) {
                // Stack overflow — hang visibly
                arch_halt();
            }
        }
        arch_idle(); // sleep until next tick or IRQ
    }
}

void kernel_stack_init(struct tcb *tcb, uint32_t *stack, uint32_t stack_words, void (*task)(void))
{
    // Stamp every word — lets us detect overflow and measure watermark
    for (uint32_t i = 0; i < stack_words; i++) {
        stack[i] = STACK_FILL_PATTERN;
    }

    // Point to the top of the stack, then carve out exactly
    // one frame — no index arithmetic, no magic offsets
    uint32_t *stack_top = stack + stack_words;

    // Step back by the size of our frame struct
    stack_frame_t *frame = ((stack_frame_t *) stack_top) - 1;

    // Hardware frame
    frame->xpsr = (1U << 24);                // Thumb bit — must be set
    frame->pc   = (uint32_t) task;           // where the task starts
    frame->lr   = (uint32_t) task_exit_trap; // called if task fn returns
    frame->r12  = 0;
    frame->r3   = 0;
    frame->r2   = 0;
    frame->r1   = 0;
    frame->r0   = 0;

    // Software frame — debug sentinel values
    frame->exc_return = 0xFFFFFFFDu; // basic frame — no FPU on first switch
    frame->r11        = 0xAAAAAAAA;
    frame->r10        = 0xAAAAAAAA;
    frame->r9         = 0xAAAAAAAA;
    frame->r8         = 0xAAAAAAAA;
    frame->r7         = 0xAAAAAAAA;
    frame->r6         = 0xAAAAAAAA;
    frame->r5         = 0xAAAAAAAA;
    frame->r4         = 0xAAAAAAAA;

    // SP points at the bottom of the frame (r4, lowest address)
    tcb->stack_ptr  = (uint32_t *) frame;
    tcb->stack_base = stack;
}

uint8_t kernel_add_thread(void (*task)(void), uint32_t *stack, uint32_t stack_words, const char *name)
{
    if (thread_count >= NUM_THREADS)
        return 0;

    arch_irq_disable();

    struct tcb *tcb = &tcbs[thread_count];
    tcb->name       = name;

    kernel_stack_init(tcb, stack, stack_words, task);
    thread_count++;

    arch_irq_enable();
    return 1;
}

void kernel_init(void)
{
    /* The board is already up — main() calls board_init() first — so the
     * clock tree is settled and clock_get_sysclk() reports the real core
     * frequency that the tick reload has to be derived from.
     *
     * SysTick sits one level above PendSV, so a tick can pre-empt a switch
     * but a switch can never delay a tick. */
    arch_systick_start(clock_get_sysclk(), TICK_RATE_HZ, TICK_PRIORITY - 1);
    arch_sched_prio_set(TICK_PRIORITY);
}

void kernel_launch(void)
{
    /* kernel_init() is the caller's job — main() runs it before adding
     * threads. Calling it again here re-ran board/clock/SysTick setup. */

    // Add idle task as the last entry — always has a task to run
    struct tcb *idle = &tcbs[thread_count];
    idle->name       = "idle";
    kernel_stack_init(idle, idle_stack, IDLE_STACK_WORDS, idle_task);

    // Wire circular linked list across all tasks including idle
    for (uint8_t i = 0; i < thread_count; i++) {
        tcbs[i].next = &tcbs[i + 1];
    }
    idle->next  = &tcbs[0]; // idle wraps back to first task
    current_tcb = &tcbs[0]; // start with first task

    // Hand Thread mode its own stack and request the first switch
    arch_sched_start();

    arch_irq_enable();
    while (1) {
    }
}

/* kernel_tick() and the tick-based delays live in time.c, which owns the
 * counter. */

void kernel_yield(void)
{
    arch_sched_trigger();
}
