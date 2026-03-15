#include "include/kernel.h"
#include "config.h"

struct tcb tcbs[NUM_THREADS];
struct tcb *current_ptr;

// each thread will have a stack size of 100 i.e. 400 bytes
int32_t TCB_STACK[NUM_THREADS][STACK_SIZE];
