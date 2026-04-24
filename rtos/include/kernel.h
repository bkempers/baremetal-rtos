#ifndef KERNEL_H
#define KERNEL_H

#include <stdint.h>

#include "config.h"

#define KERNEL_STACK_DEFINE(name, words) static uint32_t name[(words)] __attribute__((aligned(8)))

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
    uint32_t exc_return;   // LR value — encodes frame type for restore

    // Hardware-saved (CPU pushes/pops these automatically)
    uint32_t r0;    // a1 — first argument / return value
    uint32_t r1;    // a2
    uint32_t r2;    // a3
    uint32_t r3;    // a4
    uint32_t r12;
    uint32_t lr;    // r14 — return address
    uint32_t pc;    // r15 — where execution resumes
    uint32_t xpsr;
} stack_frame_t;

typedef struct tcb {
    uint32_t *stack_ptr;
    struct tcb *next;

    uint32_t *stack_base;

    const char *name;
} tcb_t;

extern uint8_t kernel_first_switch;

void kernel_stack_init(tcb_t *tcb, uint32_t *stack, uint32_t stack_words, void (*task)(void));
uint8_t kernel_add_thread(void (*task)(void), uint32_t *stack, uint32_t stack_words, const char* name);

void kernel_init(void);

void kernel_launch(void);

// Called from SysTick_Handler
void    kernel_tick(void);

// Called from tasks
void    kernel_delay_ms(uint32_t ms);
void    kernel_yield(void);

#endif // KERNEL_H
