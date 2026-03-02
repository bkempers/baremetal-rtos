#include "unity.h"

void setUp(void)    {}
void tearDown(void) {}

void test_hello_world(void) {
    TEST_ASSERT_EQUAL(4, 2 + 2);
}

void test_ceedling_is_wired_up(void) {
    TEST_ASSERT_TRUE(1);
}
