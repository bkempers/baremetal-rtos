#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stdint.h>
#include <stdbool.h>

#define RINGBUFF_SIZE 1024

typedef struct {
    uint8_t buf[RINGBUFF_SIZE];
    uint32_t head;
    uint32_t tail;
    volatile uint32_t count;
} ringbuffer;

void ringbuffer_init(ringbuffer *buf);

bool ringbuffer_is_empty(ringbuffer *buf);
bool ringbuffer_is_full(ringbuffer *buf);

// uint8_t
uint8_t ringbuffer_get(ringbuffer *buf);
void ringbuffer_put(ringbuffer *buf, const uint8_t item);
void ringbuffer_put_many(ringbuffer *buf, const uint8_t* items, const uint32_t size);

// // char
// char ringbuffer_get(ringbuffer *buf);
// void ringbuffer_put(ringbuffer *buf, const char item);
// void ringbuffer_put_many(ringbuffer *buf, const char* items, const uint32_t size);

void ringbuffer_flush(ringbuffer *buf, const bool clear_buf);

#endif
