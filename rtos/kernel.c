#include "include/kernel.h"
#include "config.h"

struct tcb tcbs[NUM_THREADS];
struct tcb *current_ptr;

// each thread will have a stack size of 100 i.e. 400 bytes
int32_t TCB_STACK[NUM_THREADS][STACK_SIZE];

void kernel_stack_init(int i) {
    tcbs[i].stack_ptr = &TCB_STACK[i][STACK_SIZE - 16]; // stack pointer

    // set bit 21 (T-bit) in PSR to 1 to operate in thumb mode
    TCB_STACK[i][STACK_SIZE - 1] = (1U<<21); // PSR


    //NOTE: for debug puposes only
    TCB_STACK[i][STACK_SIZE - 3] = 0xAAAAAAAA;
    TCB_STACK[i][STACK_SIZE - 4] = 0xAAAAAAAA;
    TCB_STACK[i][STACK_SIZE - 5] = 0xAAAAAAAA;
    TCB_STACK[i][STACK_SIZE - 6] = 0xAAAAAAAA;
    TCB_STACK[i][STACK_SIZE - 7] = 0xAAAAAAAA;
    TCB_STACK[i][STACK_SIZE - 8] = 0xAAAAAAAA;

    TCB_STACK[i][STACK_SIZE - 9] = 0xAAAAAAAA;
    TCB_STACK[i][STACK_SIZE - 10] = 0xAAAAAAAA;
    TCB_STACK[i][STACK_SIZE - 11] = 0xAAAAAAAA;
    TCB_STACK[i][STACK_SIZE - 12] = 0xAAAAAAAA;
    TCB_STACK[i][STACK_SIZE - 13] = 0xAAAAAAAA;
    TCB_STACK[i][STACK_SIZE - 14] = 0xAAAAAAAA;
    TCB_STACK[i][STACK_SIZE - 15] = 0xAAAAAAAA;
    TCB_STACK[i][STACK_SIZE - 16] = 0xAAAAAAAA;
}
