#include <string.h>
#include "ringbuffer.h"

int ringbuffer_init(ringbuffer *rb, uint8_t *data, size_t size)
{
    if (rb == NULL || data == NULL || size == 0) {
        return -1;
    }

    rb->buffer = data;
    rb->head = 0;
    rb->tail = 0;
    rb->size = size;

    return 0;
}

void ringbuffer_reset(ringbuffer *rb)
{
    if (rb->buffer != NULL) {
        memset(rb->buffer, 0, (size_t)ringbuffer_size(rb));
    }

    rb->head = 0;
    rb->tail = 0;
}

bool ringbuffer_empty(ringbuffer *rb)
{
    return (rb->head == rb->tail);
}

bool ringbuffer_full(ringbuffer *rb)
{
    uint32_t next_head = rb->head + 1;
    if (next_head == rb->size) {
        next_head = 0;
    }
    return (next_head == rb->tail);
}

uint32_t ringbuffer_size(ringbuffer *rb)
{
    return rb->size;
}

bool ringbuffer_get(ringbuffer *rb, uint8_t *value)
{
    /* check if ringbuffer is empty */
    if (ringbuffer_empty(rb)) {
        return false;
    }

    *value = rb->buffer[rb->tail];
    uint32_t next_tail = rb->tail + 1;

    /* check for wrap around */
    if (next_tail == ringbuffer_size(rb)) {
        next_tail = 0;
    }

    rb->tail = next_tail;
    return true;
}

bool ringbuffer_put(ringbuffer *rb, const uint8_t item)
{
    /* check for wrap around */
    uint32_t new_head = rb->head + 1;
    if (new_head == ringbuffer_size(rb)) {
        new_head = 0;
    }

    /* check if ringbuffer is full */
    if (new_head == rb->tail) {
        return false;
    }

    rb->buffer[rb->head] = item;
    rb->head = new_head;
    return true;
}
