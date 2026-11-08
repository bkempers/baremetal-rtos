#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct {
    uint8_t *buffer;
    volatile uint32_t head;
    volatile uint32_t tail;
    uint32_t size;
} ringbuffer;

int ringbuffer_init(ringbuffer *rb, uint8_t *data, size_t size);
void ringbuffer_reset(ringbuffer *rb);

bool ringbuffer_empty(ringbuffer *rb);
bool ringbuffer_full(ringbuffer *rb);
uint32_t ringbuffer_size(ringbuffer *rb);

//TODO make these get/put multiple
bool ringbuffer_get(ringbuffer *rb, uint8_t *value);
bool ringbuffer_put(ringbuffer *rb, const uint8_t item);

#endif
