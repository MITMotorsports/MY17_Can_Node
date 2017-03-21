#ifndef OUTPUT_H
#define OUTPUT_H

#include <stdbool.h>
#include <stdint.h>

#include "state_types.h"

/**
 * Takes in the current state of the system and decides what to send in the
 * next CAN output.
 *
 * TODO flesh out this spec a bit more once we decide how to finish this.
 */
void update_adc_outputs(ADC_STATE_T *adc_state, ADC_OUTPUT_T *adc_output);

#endif

