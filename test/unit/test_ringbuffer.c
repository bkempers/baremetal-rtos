#include "unity.h"

#include "ringbuffer.h"
#include <complex.h>

static ringbuffer rb;
static uint8_t storage[8];

void setUp(void)
{
    ringbuffer_init(&rb, storage, sizeof(storage));
}

void tearDown(void)
{ }

static void rb_dump(const char *label)
{
    char cells[160] = {0};
    int  n = 0;

    for (uint32_t i = 0; i < rb.size; i++) {
        char mark = (i == rb.head && i == rb.tail) ? '*'
        : (i == rb.head)                 ? 'H'
        : (i == rb.tail)                 ? 'T'
        :                                  '.';
        n += snprintf(cells + n, sizeof(cells) - (size_t)n, "%02x%c ", rb.buffer[i], mark);
    }

    char line[256];
    snprintf(line, sizeof(line), "%-18s head=%u tail=%u | %s", label, rb.head, rb.tail, cells);
    TEST_PRINTF("%s", line);
}

void test_ringbuffer_returns_correct_byte(void)
{
    uint8_t out = 0;

    rb_dump("before put");

    TEST_ASSERT_TRUE(ringbuffer_put(&rb, 0xA5));
    TEST_ASSERT_FALSE(ringbuffer_empty(&rb));

    rb_dump("after put");

    TEST_ASSERT_TRUE(ringbuffer_get(&rb, &out));
    TEST_ASSERT_EQUAL_HEX8(0xA5, out);
    TEST_ASSERT_TRUE(ringbuffer_empty(&rb));

    rb_dump("after get");
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_ringbuffer_returns_correct_byte);

    return UNITY_END();
}
