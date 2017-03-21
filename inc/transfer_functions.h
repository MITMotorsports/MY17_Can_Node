#ifndef TRANSFER_FUNCTIONS_H
#define TRANSFER_FUNCTIONS_H

#include <stdint.h>

#include "state_types.h"

#define TRAVEL_MAX 1000

/**
 * Transfer Function:
 *
 * We define a transfer function as a function that takes an analog reading in
 * [0:1023] and outputs a result in [0:1000] representing the amount of travel
 * of the physical device in units of 0.1 percent.
 *
 * A transfer function need not be linear (although it is for MY16) but it should
 * be monotonically increasing. There should be one transfer function per
 * physical sensor (potentiometer, pressure sensor, etc).
 *
 * @param reading: integer in the range of [0:1023] representing analog value.
 * @return integer in the range of [0:1000] representing 0.1-percent travel
 *    of physical device. Monotonically increasing relative to reading.
 */
uint16_t accel_1_transfer_fn(uint16_t reading);
uint16_t accel_2_transfer_fn(uint16_t reading);
uint16_t brake_1_transfer_fn(uint16_t reading);
uint16_t brake_2_transfer_fn(uint16_t reading);
uint16_t steering_transfer_fn(uint16_t reading);

// PRIVATE, VISIBLE FOR TESTING
uint16_t linear_transfer_fn(uint32_t reading, uint16_t lower_bound, uint16_t upper_bound);

#endif
