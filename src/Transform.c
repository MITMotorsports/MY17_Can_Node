#include "Transform.h"

#include "Common.h"

#define ACCEL_1_LOWER_BOUND 105
#define ACCEL_1_UPPER_BOUND 645
#define ACCEL_2_LOWER_BOUND 71
#define ACCEL_2_UPPER_BOUND 340

#define BRAKE_1_LOWER_BOUND 380
#define BRAKE_1_UPPER_BOUND 780
#define BRAKE_2_LOWER_BOUND 220
#define BRAKE_2_UPPER_BOUND 270

uint16_t Transform_accel_1(uint16_t reading, uint16_t desired_width) {
  return Transform_linear_transfer_fn(reading, desired_width, ACCEL_1_LOWER_BOUND, ACCEL_1_UPPER_BOUND);
}

uint16_t Transform_accel_2(uint16_t reading, uint16_t desired_width) {
  return Transform_linear_transfer_fn(reading, desired_width, ACCEL_2_LOWER_BOUND, ACCEL_2_UPPER_BOUND);
}

uint16_t Transform_linear_transfer_fn(uint32_t reading, uint16_t desired_width, uint16_t lower_bound, uint16_t upper_bound) {
  // Ensure reading is within expected range
  reading = max(reading, lower_bound);
  reading = min(reading, upper_bound);

  // Make reading between 0 and diff
  const uint16_t diff = upper_bound - lower_bound;
  reading = reading - lower_bound;

  // Now scale from [0:diff] to [0:desired_width].
  // Note: it's critical that reading be a 32 bit int because otherwise this line will cause overflow!
  reading = reading * desired_width;
  reading = reading / diff;

  // This cast is safe because we have asserted reading is in range [0:desired_width] in previous lines
  uint16_t short_val = (uint16_t)(reading);
  return short_val;
}
