#include "unity.h"

#include "state_types.h"
#include "state.h"

void test_demo(void) {
  TEST_ASSERT_EQUAL_INT(0, 0);
}
int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_demo);
  return UNITY_END();
}
