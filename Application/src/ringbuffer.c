#include <string.h>

#include "ringbuffer.h"

static bool ringbuffer_is_free(ringbuffer *_this, const uint32_t size);

static bool ringbuffer_is_free(ringbuffer *_this, const uint32_t size) {
   if (_this->count + size <= RINGBUFF_SIZE) {
       return true;
   } else {
       return false;
   }
}

void ringbuffer_init(ringbuffer *_this) {
     memset(_this, 0, sizeof (*_this));
}

bool ringbuffer_is_empty(ringbuffer *_this) {
    return ( 0 == _this->count );
}

bool ringbuffer_is_full(ringbuffer *_this) {
    return ( _this->count == RINGBUFF_SIZE );
}

uint8_t ringbuffer_get(ringbuffer *_this) {
    uint8_t item;
    if (_this->count > 0) {
        item = _this->buf[_this->tail];
        _this->tail = ((_this->tail + 1) % RINGBUFF_SIZE);
        _this->count--;
    } else {
        item = 0xFF;
    }

    return item;
}

void ringbuffer_put(ringbuffer *_this, const uint8_t item) {
    if (_this->count < RINGBUFF_SIZE) {
        _this->buf[_this->head] = item;
        _this->head = ((_this->head + 1) % RINGBUFF_SIZE);
        _this->count++;
    }
}

void ringbuffer_put_many(ringbuffer *_this, const uint8_t *items, const uint32_t size) {
    for (int i = 0; i < size; i++) {
        ringbuffer_put(_this, items[i]);
    }
}

void ringbuffer_flush(ringbuffer *_this, const bool clear_buf) {
    _this->head = 0;
    _this->tail = 0;
    _this->count = 0;
    if (clear_buf) {
        memset(_this->buf, 0, sizeof(_this->buf));
    }
}
