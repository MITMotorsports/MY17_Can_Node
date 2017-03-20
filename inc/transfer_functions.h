#ifndef TRANSFER_FUNCTIONS_H
#define TRANSFER_FUNCTIONS_H

#include <stdint.h>

#include "state_types.h"

#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))

#define TRAVEL_MAX 1000

// PUBLIC
uint16_t accel_1_transfer_fn(uint16_t reading);
uint16_t accel_2_transfer_fn(uint16_t reading);
uint16_t brake_1_transfer_fn(uint16_t reading);
uint16_t brake_2_transfer_fn(uint16_t reading);
uint16_t steering_transfer_fn(uint16_t reading);

// PRIVATE
uint16_t linear_transfer_fn(uint32_t reading, uint16_t lower_bound, uint16_t upper_bound);


#endif
