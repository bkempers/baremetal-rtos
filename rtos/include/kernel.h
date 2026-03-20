#ifndef KERNEL_H
#define KERNEL_H

#include "config.h"
#include <stdint.h>

typedef struct {
    // Software-saved (PendSV)
    uint32_t r4;
    uint32_t r5;
    uint32_t r6;
    uint32_t r7;
    uint32_t r8;
    uint32_t r9;
    uint32_t r10;
    uint32_t r11;

    // Hardware-saved (CPU pushes/pops these automatically)
    uint32_t r0;    // a1 — first argument / return value
    uint32_t r1;    // a2
    uint32_t r2;    // a3
    uint32_t r3;    // a4
    uint32_t r12;
    uint32_t lr;    // r14 — return address
    uint32_t pc;    // r15 — where execution resumes
    uint32_t xpsr;
} __attribute__((packed)) stack_frame_t;

typedef struct tcb {
    uint32_t *stack_ptr;
    struct tcb *next;
};

void kernel_stack_init(struct tcb *tcb, int32_t *stack, uint32_t stack_words, void (*task)(void));
uint8_t kernel_add_thread(void (*task)(void), int32_t *stack, uint32_t stack_words);

#endif // KERNEL_H
