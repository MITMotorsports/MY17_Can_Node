#include "Common.h"

uint32_t scale(uint32_t val, uint32_t old_scale, uint32_t new_scale) {
  return (val * new_scale) / old_scale;
}

