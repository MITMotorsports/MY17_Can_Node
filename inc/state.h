#ifndef DIGITAL_TRANSFORM_H
#define DIGITAL_TRANSFORM_H

#include <stdbool.h>
#include <stdint.h>

#include "transfer_functions.h"
#include "state_types.h"

void update_adc_inputs(ADC_INPUT_T *adc_input);
void update_adc_state(ADC_INPUT_T *adc_input, ADC_STATE_T *adc_state);
void update_adc_outputs(ADC_INPUT_T *adc_input, ADC_STATE_T *adc_state, ADC_OUTPUT_T *adc_output);

// PRIVATE, VISIBLE FOR TESTING
void observe_plausibility(ADC_INPUT_T *adc_input, ADC_STATE_T *adc_state);
void report_plausibility(ADC_INPUT_T *adc_input, ADC_STATE_T *adc_state);
void check_conflict(ADC_INPUT_T *adc_input, ADC_STATE_T *adc_output);

// PRIVATE
bool check_plausibility(uint16_t accel_1_val, uint16_t accel_2_val);
uint32_t scale(uint32_t val, uint32_t old_scale, uint32_t new_scale);
#endif // DIGITAL_TRANSFORM_H
