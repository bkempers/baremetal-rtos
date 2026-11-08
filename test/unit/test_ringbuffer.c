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

void test_ringbuffer_wraparound(void)
{
    uint8_t out = 0;

    TEST_ASSERT_TRUE(ringbuffer_empty(&rb));

    // fill up ringbuffer
    TEST_ASSERT_TRUE(ringbuffer_put(&rb, 0x01));
    TEST_ASSERT_TRUE(ringbuffer_put(&rb, 0x02));
    TEST_ASSERT_TRUE(ringbuffer_put(&rb, 0x03));
    TEST_ASSERT_TRUE(ringbuffer_put(&rb, 0x04));
    TEST_ASSERT_TRUE(ringbuffer_put(&rb, 0x05));
    TEST_ASSERT_TRUE(ringbuffer_put(&rb, 0x06));
    TEST_ASSERT_TRUE(ringbuffer_put(&rb, 0x07));

    rb_dump("after writing");

    // don't overwrite tail?
    TEST_ASSERT_FALSE(ringbuffer_put(&rb, 0x08));
    TEST_ASSERT_TRUE(ringbuffer_full(&rb));

    // readout all data
    TEST_ASSERT_TRUE(ringbuffer_get(&rb, &out));
    TEST_ASSERT_EQUAL_HEX8(0x01, out);
    TEST_ASSERT_TRUE(ringbuffer_get(&rb, &out));
    TEST_ASSERT_EQUAL_HEX8(0x02, out);
    TEST_ASSERT_TRUE(ringbuffer_get(&rb, &out));
    TEST_ASSERT_EQUAL_HEX8(0x03, out);
    TEST_ASSERT_TRUE(ringbuffer_get(&rb, &out));
    TEST_ASSERT_EQUAL_HEX8(0x04, out);

    rb_dump("midway reading");

    TEST_ASSERT_TRUE(ringbuffer_get(&rb, &out));
    TEST_ASSERT_EQUAL_HEX8(0x05, out);
    TEST_ASSERT_TRUE(ringbuffer_get(&rb, &out));
    TEST_ASSERT_EQUAL_HEX8(0x06, out);
    TEST_ASSERT_TRUE(ringbuffer_get(&rb, &out));
    TEST_ASSERT_EQUAL_HEX8(0x07, out);

    TEST_ASSERT_TRUE(ringbuffer_empty(&rb));
    TEST_ASSERT_FALSE(ringbuffer_get(&rb, &out));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_ringbuffer_returns_correct_byte);
    RUN_TEST(test_ringbuffer_wraparound);

    return UNITY_END();
}
