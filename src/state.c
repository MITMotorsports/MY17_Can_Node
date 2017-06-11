#include "State.h"

#include "Common.h"
#include "Rules.h"
#include "Transform.h"

#define DRIVER_OUTPUT_MSG_MS 20
#define RAW_VALUES_MSG_MS 100
#define WHEEL_SPEED_MSG_MS 20

void update_can_state(Input_T *input, State_T *state, Output_T *output);

bool period_reached(uint32_t start, uint32_t period, uint32_t msTicks);
void update_can_driver_output(Message_State_T *state, Can_Output_T *output, uint32_t msTicks);
void update_can_raw_values(Message_State_T *state, Can_Output_T *output, uint32_t msTicks);
void update_can_wheel_speed(Message_State_T *state, Can_Output_T *output, uint32_t msTicks);

void State_initialize(State_T *state) {
  state->rules->has_conflict = false;
  state->rules->implausibility_observed = false;
  state->rules->implausibility_reported = false;
  state->rules->implausibility_time_ms = 0;

  state->message->can_driver_output_ms = 0;
  state->message->can_raw_values_ms = 0;
  state->message->can_wheel_speed_ms = 0;
  state->message->logging_throttle_ms = 0;
  state->message->logging_brake_ms = 0;
}

void State_update_state(Input_T *input, State_T *state, Output_T *output) {
  Rules_update_implausibility(input->adc, state->rules, input->msTicks);
  Rules_update_conflict(input, state->rules);
  update_can_state(input, state, output);
}

void update_can_state(Input_T *input, State_T *state, Output_T *output) {
  Message_State_T *message = state->message;
  Can_Output_T *can = output->can;
  const uint32_t msTicks = input->msTicks;

  update_can_driver_output(message, can, msTicks);
  update_can_raw_values(message, can, msTicks);
  update_can_wheel_speed(message, can, msTicks);
}

void update_can_driver_output(Message_State_T *message, Can_Output_T *can, uint32_t msTicks) {
  uint32_t *last_msg = &message->can_driver_output_ms;

  if(period_reached(*last_msg, DRIVER_OUTPUT_MSG_MS, msTicks)) {
    *last_msg = msTicks;
    can->send_driver_output_msg = true;
  }
}

void update_can_raw_values(Message_State_T *message, Can_Output_T *can, uint32_t msTicks) {
  uint32_t *last_msg = &message->can_raw_values_ms;

  if(period_reached(*last_msg, RAW_VALUES_MSG_MS, msTicks)) {
    *last_msg = msTicks;
    can->send_raw_values_msg = true;
  }
}

void update_can_wheel_speed(Message_State_T *message, Can_Output_T *can, uint32_t msTicks) {
  uint32_t *last_msg = &message->can_wheel_speed_ms;

  if(period_reached(*last_msg, WHEEL_SPEED_MSG_MS, msTicks)) {
    *last_msg = msTicks;
    can->send_wheel_speed_msg = true;
  }
}

bool period_reached(uint32_t start, uint32_t period, uint32_t msTicks) {
  const uint32_t next_time = start + period;
  return next_time < msTicks;
}

