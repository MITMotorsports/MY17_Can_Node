#include "state.h"

#include "adc.h"
#include "transfer_functions.h"

#define ADC_UPDATE_PERIOD_MS 10
#define IMPLAUSIBILITY_REPORT_MS 100

#define BYTE_MAX 255

/****************************** PUBLIC METHODS BELOW *************************/

void update_adc_inputs(ADC_INPUT_T *adc_input) {
  uint32_t nextUpdate_ms = adc_input->lastUpdate_ms + ADC_UPDATE_PERIOD_MS;
  if (adc_input->msTicks >= nextUpdate_ms) {
    adc_input->accel_1_travel = accel_1_transfer_fn(ADC_Read(ACCEL_1_CHANNEL));
    adc_input->accel_2_travel = accel_2_transfer_fn(ADC_Read(ACCEL_2_CHANNEL));
    adc_input->brake_1_travel = brake_1_transfer_fn(ADC_Read(BRAKE_1_CHANNEL));
    adc_input->brake_2_travel = brake_2_transfer_fn(ADC_Read(BRAKE_2_CHANNEL));
    adc_input->steering_travel = steering_transfer_fn(ADC_Read(STEERING_CHANNEL));
    adc_input->lastUpdate_ms = adc_input->msTicks;
  }
}

void update_adc_state(ADC_INPUT_T *adc_input, ADC_STATE_T *adc_state) {
  observe_plausibility(adc_input, adc_state);
  report_plausibility(adc_input, adc_state);
  check_conflict(adc_input, adc_state);
}

void update_adc_outputs(ADC_INPUT_T *adc_input, ADC_STATE_T *adc_state, ADC_OUTPUT_T *adc_output) {
  // TODO more complicated logic for determining torque - for now just use raw throttle
  uint16_t pedal_travel = min(adc_input->accel_1_travel, adc_input->accel_2_travel);
  uint16_t brake_travel = min(adc_input->brake_1_travel, adc_input->brake_2_travel);

  bool should_zero =
      adc_state->has_conflict || adc_state->implausibility_reported;

  adc_output->requested_torque = should_zero ? 0 : scale(pedal_travel, TRAVEL_MAX, BYTE_MAX);
  adc_output->brake_pressure = scale(brake_travel, TRAVEL_MAX, BYTE_MAX);
  adc_output->steering_position = scale(adc_input->steering_travel, TRAVEL_MAX, BYTE_MAX);
}

/***************** PRIVATE BUT TESTED METHODS BELOW **************************/

void observe_plausibility(ADC_INPUT_T *adc_input, ADC_STATE_T *adc_state) {
  bool has_implausibility = check_plausibility(adc_input->accel_1_travel, adc_input->accel_2_travel);
  if (has_implausibility && !adc_state->implausibility_observed) {
    adc_state->implausibility_time_ms = adc_input->lastUpdate_ms;
  }
  adc_state->implausibility_observed = has_implausibility;
}

void report_plausibility(ADC_INPUT_T *adc_input, ADC_STATE_T *adc_state) {
  uint32_t time_since_implausibility_observed =
      adc_input->msTicks - adc_state->implausibility_time_ms;

  bool time_exceeded_threshold =
      time_since_implausibility_observed > IMPLAUSIBILITY_REPORT_MS;

  bool should_report_implausibility =
      adc_state->implausibility_observed && time_exceeded_threshold;
  if (!adc_state->implausibility_reported && should_report_implausibility) {
      // Implausibility reported for first time,
      // so send urgent message by EV2.3.5
      adc_state->urgent_message = true;
  }
  adc_state->implausibility_reported = should_report_implausibility;
}

void check_conflict(ADC_INPUT_T *adc_input, ADC_STATE_T *adc_state) {
  if (adc_state->implausibility_reported) {
    // Checking conflict is pointless if implausibility
    return;
  }
  uint16_t throttle_travel =
      min(adc_input->accel_1_travel, adc_input->accel_2_travel);

  // TODO figure this out once we know details about front/rear brake pressures
  uint16_t brake_travel = adc_input->brake_1_travel;

  // Conflict state: Remove conflict if throttle < 5% travel (EV2.5.1)
  if (adc_state->has_conflict) {
    bool should_remove_conflict = throttle_travel < 50;
    if (should_remove_conflict) {
      // Conflict was lifted, so send urgent message so we can drive again
      adc_state->urgent_message = true;
      adc_state->has_conflict = false;
    }
  }

  // No conflict state: Add conflict if throttle > 25% travel
  // and brakes are mechanically actuated (EV2.5).
  // TODO figure out what constitutes "mechanically actuated"
  else {
    bool should_add_conflict = throttle_travel < 250 && brake_travel > 250;
    adc_state->has_conflict = should_add_conflict;
  }
}

bool check_plausibility(uint16_t accel_1_travel, uint16_t accel_2_travel) {
  uint16_t max_travel = max(accel_1_travel, accel_2_travel);
  uint16_t min_travel = min(accel_1_travel, accel_2_travel);
  return max_travel - min_travel < 100;
}

uint32_t scale(uint32_t val, uint32_t old_scale, uint32_t new_scale) {
  return (val * new_scale) / old_scale;
}

