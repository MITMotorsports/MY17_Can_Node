#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))

uint32_t scale(uint32_t val, uint32_t old_scale, uint32_t new_scale);
#endif
