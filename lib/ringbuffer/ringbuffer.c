#include <string.h>
#include "ringbuffer.h"

int ringbuffer_init(ringbuffer *rb, size_t size)
{
    rb->buffer = malloc(size);
    if (rb->buffer != NULL) {
        memset(rb->buffer, 0, size);
    }

    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;

    return 1;
}

void ringbuffer_reset(ringbuffer *rb)
{
    if (rb->buffer != NULL) {
        memset(rb->buffer, 0, (size_t)ringbuffer_size(rb));
    }

    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

bool ringbuffer_empty(ringbuffer *rb)
{
    return (rb->count == 0);
}

bool ringbuffer_full(ringbuffer *rb)
{
    return ((size_t)rb->count == ringbuffer_size(rb));
}

uint32_t ringbuffer_size(ringbuffer *rb)
{
    return (sizeof(rb->buffer)/sizeof(rb->buffer[0]));
}

bool ringbuffer_get(ringbuffer *rb, uint8_t *value)
{
    /* check if ringbuffer is empty */
    if (rb->head == rb->tail || rb->count == 0) {
        return false;
    }

    value = &rb->buffer[rb->tail];
    uint32_t next_tail = rb->tail + 1;

    /* check for wrap around */
    if (next_tail == ringbuffer_size(rb)) {
        next_tail = 0;
    }

    rb->tail = next_tail;
    rb->count--;
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
    rb->count++;
    return true;
}
