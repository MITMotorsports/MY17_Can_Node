#ifndef STATE_H
#define STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "Types.h"

void State_initialize(State_T *state);
void State_update_state(Input_T *input, State_T *state, Output_T *output);

#endif // STATE_H
