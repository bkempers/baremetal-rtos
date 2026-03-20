#include "include/kernel.h"
#include "config.h"

struct tcb tcbs[NUM_THREADS];
struct tcb *current_ptr;

static int thread_count = 0;

static void task_exit_trap(void) {
    __disable_irq();
    while (1) {}
}

void kernel_stack_init(struct tcb *tcb, int32_t *stack, uint32_t stack_words, void (*task)(void)) {
    // Point to the top of the stack, then carve out exactly
    // one frame — no index arithmetic, no magic offsets
    int32_t *stack_top = (uint32_t *)stack + stack_words;

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

uint8_t kernel_add_thread(void (*task)(void), int32_t  *stack, uint32_t  stack_words) {
    if (thread_count >= NUM_THREADS) return 0;

    __disable_irq();

    struct tcb *tcb = &tcbs[thread_count++];
    kernel_stack_init(tcb, stack, stack_words, task);
    
    __enable_irq();
    return 1;
}
