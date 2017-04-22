#include "state.h"

#include "common.h"
#include "transfer_functions.h"

#define IMPLAUSIBILITY_REPORT_MS 100
#define IMPLAUSIBILITY_THROTTLE_TRAVEL 100
#define CONFLICT_BEGIN_THROTTLE_TRAVEL 250
#define CONFLICT_BEGIN_BRAKE_TRAVEL 250
#define CONFLICT_END_THROTTLE_TRAVEL 50

void read_input(ADC_INPUT_T *adc_input, ADC_STATE_T *adc_state) {
  adc_state->accel_1_raw = adc_input->accel_1_raw;
  adc_state->accel_2_raw = adc_input->accel_2_raw;
  adc_state->brake_1_raw = adc_input->brake_1_raw;
  adc_state->brake_2_raw = adc_input->brake_2_raw;
  adc_state->steering_raw = adc_input->steering_raw;

  adc_state->accel_1_travel = accel_1_transfer_fn(adc_input->accel_1_raw);
  adc_state->accel_2_travel = accel_2_transfer_fn(adc_input->accel_2_raw);
  adc_state->brake_1_travel = brake_1_transfer_fn(adc_input->brake_1_raw);
  adc_state->brake_2_travel = brake_2_transfer_fn(adc_input->brake_2_raw);
  adc_state->steering_travel = steering_transfer_fn(adc_input->steering_raw);

  adc_state->msTicks = adc_input->msTicks;
}

void observe_implausibility(ADC_STATE_T *adc_state) {
  bool has_implausibility = check_implausibility(adc_state->accel_1_travel, adc_state->accel_2_travel);
  if (has_implausibility && !adc_state->implausibility_observed) {
    adc_state->implausibility_time_ms = adc_state->msTicks;
  }
  adc_state->implausibility_observed = has_implausibility;
}

void report_implausibility(ADC_STATE_T *adc_state) {
  uint32_t time_since_implausibility_observed =
      adc_state->msTicks - adc_state->implausibility_time_ms;

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

void check_conflict(ADC_STATE_T *adc_state) {
  if (adc_state->implausibility_reported) {
    // Checking conflict is pointless if implausibility
    return;
  }
  uint16_t throttle_travel =
      min(adc_state->accel_1_travel, adc_state->accel_2_travel);

  // TODO figure this out once we know details about front/rear brake pressures
  uint16_t brake_travel = adc_state->brake_1_travel;

  // Conflict state: Remove conflict if throttle < 5% travel (EV2.5.1)
  if (adc_state->has_conflict) {
    bool should_remove_conflict =
        throttle_travel < CONFLICT_END_THROTTLE_TRAVEL;
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
    bool should_add_conflict =
        throttle_travel > CONFLICT_BEGIN_THROTTLE_TRAVEL &&
        brake_travel > CONFLICT_BEGIN_BRAKE_TRAVEL;
    adc_state->has_conflict = should_add_conflict;
  }

  // TODO remove this!
  adc_state->has_conflict = false;
}

bool check_implausibility(uint16_t accel_1_travel, uint16_t accel_2_travel) {
  uint16_t max_travel = max(accel_1_travel, accel_2_travel);
  uint16_t min_travel = min(accel_1_travel, accel_2_travel);
  return max_travel - min_travel >= IMPLAUSIBILITY_THROTTLE_TRAVEL;
}
