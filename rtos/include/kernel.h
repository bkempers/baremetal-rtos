#ifndef KERNEL_H
#define KERNEL_H

#include "config.h"
#include <stdint.h>

typedef struct tcb {
    int32_t *stack_ptr;
    struct tcb *next;
} TCB_t;

#endif // KERNEL_H
