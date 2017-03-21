#include "output.h"

#include "common.h"
#include "transfer_functions.h"

#define BYTE_MAX 255

void update_adc_outputs(ADC_STATE_T *adc_state, ADC_OUTPUT_T *adc_output) {
  // TODO more complicated logic for determining torque - for now just use raw throttle
  uint16_t pedal_travel = min(adc_state->accel_1_travel, adc_state->accel_2_travel);
  uint16_t brake_travel = min(adc_state->brake_1_travel, adc_state->brake_2_travel);

  bool should_zero =
      adc_state->has_conflict || adc_state->implausibility_reported;

  adc_output->requested_torque = should_zero ? 0 : scale(pedal_travel, TRAVEL_MAX, BYTE_MAX);
  adc_output->brake_pressure = scale(brake_travel, TRAVEL_MAX, BYTE_MAX);
  adc_output->steering_position = scale(adc_state->steering_travel, TRAVEL_MAX, BYTE_MAX);
}