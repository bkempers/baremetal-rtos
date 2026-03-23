#include "include/kernel.h"
#include "config.h"

#include "stm32h7rs_hal.h"

struct tcb tcbs[NUM_THREADS + 1];
static uint8_t thread_count = 0;
static uint32_t tick_count = 0;

uint8_t kernel_first_switch = 1;

static uint32_t idle_stack[IDLE_STACK_WORDS];

struct tcb *current_tcb;

__attribute__((weak)) void board_clock_init(void) {}
__attribute__((weak)) void board_hal_init(void)   {}

static void task_exit_trap(void) {
    __disable_irq();
    while (1) {}
}

static void idle_task(void) {
    while (1) {
        // Check every task's stack bottom for overflow
        for (uint8_t i = 0; i < thread_count; i++) {
            if (tcbs[i].stack_base[0] != STACK_FILL_PATTERN) {
                // Stack overflow — hang visibly
                __disable_irq();
                while (1) {}
            }
        }
        __WFI();    // sleep until next tick or IRQ
    }
}

void kernel_stack_init(struct tcb *tcb, uint32_t *stack, uint32_t stack_words, void (*task)(void)) {
    // Stamp every word — lets us detect overflow and measure watermark
    for (uint32_t i = 0; i < stack_words; i++) {
        stack[i] = STACK_FILL_PATTERN;
    }

    // Point to the top of the stack, then carve out exactly
    // one frame — no index arithmetic, no magic offsets
    uint32_t *stack_top = stack + stack_words;

    // Step back by the size of our frame struct
    stack_frame_t *frame = ((stack_frame_t *)stack_top) - 1;

    // Hardware frame
    frame->xpsr = (1U << 24);              // Thumb bit — must be set
    frame->pc   = (uint32_t)task;          // where the task starts
    frame->lr   = (uint32_t)task_exit_trap;// called if task fn returns
    frame->r12  = 0;
    frame->r3   = 0;
    frame->r2   = 0;
    frame->r1   = 0;
    frame->r0   = 0;

    // Software frame — debug sentinel values
    frame->exc_return = 0xFFFFFFFDu;   // basic frame — no FPU on first switch
    frame->r11  = 0xAAAAAAAA;
    frame->r10  = 0xAAAAAAAA;
    frame->r9   = 0xAAAAAAAA;
    frame->r8   = 0xAAAAAAAA;
    frame->r7   = 0xAAAAAAAA;
    frame->r6   = 0xAAAAAAAA;
    frame->r5   = 0xAAAAAAAA;
    frame->r4   = 0xAAAAAAAA;

    // SP points at the bottom of the frame (r4, lowest address)
    tcb->stack_ptr = (uint32_t *)frame;
}

uint8_t kernel_add_thread(void (*task)(void), uint32_t *stack, uint32_t stack_words, const char* name) {
    if (thread_count >= NUM_THREADS) return 0;

    __disable_irq();

    struct tcb *tcb = &tcbs[thread_count];
    tcb->name = name;

    kernel_stack_init(tcb, stack, stack_words, task);
    thread_count++;

    __enable_irq();
    return 1;
}

void kernel_init(void) {
    board_clock_init();
    board_hal_init();

    // critical for RTOS functionality
    NVIC_SetPriority(SysTick_IRQn, TICK_PRIORITY);
    NVIC_SetPriority(PendSV_IRQn,  TICK_PRIORITY);
}

void kernel_launch(void) {
    kernel_init();

    // Add idle task as the last entry — always has a task to run
    struct tcb *idle = &tcbs[thread_count];
    idle->name  = "idle";
    kernel_stack_init(idle, idle_stack, IDLE_STACK_WORDS, idle_task);

    // Wire circular linked list across all tasks including idle
    for (uint8_t i = 0; i < thread_count; i++) {
        tcbs[i].next = &tcbs[i + 1];
    }
    idle->next   = &tcbs[0];     // idle wraps back to first task
    current_tcb  = &tcbs[0];     // start with first task
    
    // Switch Thread mode to PSP before pending PendSV
    // Without this every task runs on MSP — no kernel/task stack separation
    __set_PSP(__get_MSP());                         // safe placeholder value
    __set_CONTROL(__get_CONTROL() | 0x02u);         // SPSEL bit: MSP→PSP
    __ISB();                                        // flush pipeline after CONTROL write

    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
    __DSB();                                        // ensure write completes before irq enable

    // Hand off to PendSV to run first task — never returns
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
    __enable_irq();

    // Unreachable — PendSV fires immediately after enable
    while (1) {}
}

void kernel_tick(void) {
    tick_count++;
    // Trigger round-robin context switch every tick
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

void kernel_delay_ms(uint32_t ms) {
    uint32_t start = tick_count;
    while ((tick_count - start) < ms) {
        kernel_yield();
    }
}

void kernel_yield(void) {
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
    __DSB();
}
