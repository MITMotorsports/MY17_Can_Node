#include "Rules.h"

#include <stdbool.h>

#include "Common.h"
#include "Transform.h"

#define IMPLAUSIBILITY_REPORT_MS 100
#define IMPLAUSIBILITY_THROTTLE_TRAVEL 100

#define CONFLICT_BEGIN_THROTTLE_TRAVEL 250
#define CONFLICT_END_THROTTLE_TRAVEL 50

#define CONFLICT_BRAKE_RAW 400

bool check_implausibility(uint16_t accel_1, uint16_t accel_2);

void Rules_update_implausibility(Adc_Input_T *adc, Rules_State_T *rules, uint32_t msTicks) {
  uint16_t accel_1 = Transform_accel_1(adc->accel_1_raw, 1000);
  uint16_t accel_2 = Transform_accel_2(adc->accel_2_raw, 1000);
  bool curr_implausible = check_implausibility(accel_1, accel_2);
  bool prev_implausible = rules->implausibility_observed;

  if (!curr_implausible) {
    // No implausibility, so clear everything
    rules->implausibility_observed = false;
    rules->implausibility_reported = false;
    return;
  }

  rules->implausibility_observed = true;

  if (!prev_implausible) {
    // This is the first implausibility
    rules->implausibility_time_ms = msTicks;
    return;
  }

  // See if we have had this implausibility for a while
  uint32_t time_elapsed = msTicks - rules->implausibility_time_ms;
  bool should_report = time_elapsed > IMPLAUSIBILITY_REPORT_MS;
  rules->implausibility_reported = should_report;
}

void Rules_update_conflict(Input_T *input, Rules_State_T *rules) {
  Adc_Input_T *adc = input->adc;
  if (rules->implausibility_reported) {
    // Checking conflict is pointless if implausibility
    return;
  }
  const uint16_t accel_1 = Transform_accel_1(adc->accel_1_raw, 1000);
  const uint16_t accel_2 = Transform_accel_2(adc->accel_2_raw, 1000);
  const uint16_t accel = min(accel_1, accel_2);

  bool curr_conflict = rules->has_conflict;

  if (curr_conflict) {
    // Conflict state: Remove conflict if throttle < 5% travel (EV2.5.1)
    bool next_conflict = accel >= CONFLICT_END_THROTTLE_TRAVEL;
    rules->has_conflict = next_conflict;
    return;
  }

  const uint16_t brake = adc->brake_1_raw;


  // TODO adjust this setting based on LV voltage
  // uint16_t battery_voltage = input->misc->lv_voltage;
  bool brake_engaged = brake > CONFLICT_BRAKE_RAW;

  bool throttle_engaged = accel > CONFLICT_BEGIN_THROTTLE_TRAVEL;

  rules->has_conflict = brake_engaged && throttle_engaged;

}

bool check_implausibility(uint16_t accel_1_travel, uint16_t accel_2_travel) {
  uint16_t max_travel = max(accel_1_travel, accel_2_travel);
  uint16_t min_travel = min(accel_1_travel, accel_2_travel);
  return max_travel - min_travel >= IMPLAUSIBILITY_THROTTLE_TRAVEL;
}
