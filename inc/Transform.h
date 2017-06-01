#ifndef _TRANSFORM_H_
#define _TRANSFORM_H_

#include <stdint.h>

uint16_t Transform_accel_1(uint16_t reading, uint16_t desired_width);
uint16_t Transform_accel_2(uint16_t reading, uint16_t desired_width);
uint16_t Transform_linear_transfer_fn(uint32_t reading, uint16_t desired_width, uint16_t lower_bound, uint16_t upper_bound);


#endif // _TRANSFORM_H_
