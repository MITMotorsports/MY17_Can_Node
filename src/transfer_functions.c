#include "transfer_functions.h"

#include "common.h"

#define ACCEL_1_LOWER_BOUND 100
#define ACCEL_1_UPPER_BOUND 640
#define ACCEL_2_LOWER_BOUND 72
#define ACCEL_2_UPPER_BOUND 350
#define BRAKE_LOWER_BOUND 104
#define BRAKE_UPPER_BOUND 900
#define STEERING_POT_RIGHT_BOUND 275
#define STEERING_POT_LEFT_BOUND 960

uint16_t accel_1_transfer_fn(uint16_t reading) {
  return linear_transfer_fn(reading, ACCEL_1_LOWER_BOUND, ACCEL_1_UPPER_BOUND);
}

uint16_t accel_2_transfer_fn(uint16_t reading) {
  return linear_transfer_fn(reading, ACCEL_2_LOWER_BOUND, ACCEL_2_UPPER_BOUND);
}

uint16_t brake_1_transfer_fn(uint16_t reading) {
  return linear_transfer_fn(reading, BRAKE_LOWER_BOUND, BRAKE_UPPER_BOUND);
}

uint16_t brake_2_transfer_fn(uint16_t reading) {
  return linear_transfer_fn(reading, BRAKE_LOWER_BOUND, BRAKE_UPPER_BOUND);
}

uint16_t steering_transfer_fn(uint16_t reading) {
  return linear_transfer_fn(reading, STEERING_POT_RIGHT_BOUND, STEERING_POT_LEFT_BOUND);
}

uint16_t linear_transfer_fn(uint32_t reading, uint16_t lower_bound, uint16_t upper_bound) {
  // Ensure reading is within expected range
  reading = max(reading, lower_bound);
  reading = min(reading, upper_bound);

  // Make reading between 0 and diff
  const uint16_t diff = upper_bound - lower_bound;
  reading = reading - lower_bound;

  // Now scale from [0:diff] to [0:1000].
  // Note: it's critical that reading be a 32 bit int because otherwise this line will cause overflow!
  reading = reading * TRAVEL_MAX;
  reading = reading / diff;

  // This cast is safe because we have asserted reading is in range [0:TRAVEL_MAX] in previous lines
  uint16_t short_val = (uint16_t)(reading);
  return short_val;
}
