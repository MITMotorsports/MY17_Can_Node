#include "Output.h"

#include "chip.h"
#include "can.h"

#include <MY17_Can_Library.h>

#include "Common.h"
#include "Serial.h"
#include "Transform.h"

#define TWO_BYTE_MAX 32767
#define TEN_BIT_MAX 1023
#define BYTE_MAX 255

// Determines degree of engagement necessary for RTD
#define BRAKE_ENGAGED_THRESHOLD 150

// Microsecond = 1 millionth of a second
#define MICROSECONDS_PER_SECOND_F 1000000.0
// 1000 millirevs = 1 rev
#define MILLIREVS_PER_REV_F 1000.0
// Pointless comment to not break pattern
#define SECONDS_PER_MINUTE 60

static bool resettingPeripheral = false;

void process_can(Input_T *input, State_T *state, Can_Output_T *can);
void process_logging(Input_T *input, State_T *state, Logging_Output_T *logging);

Can_ErrorID_T write_can_driver_output(Input_T *input, Rules_State_T *rules);
Can_ErrorID_T write_can_raw_values(Adc_Input_T *adc);
Can_ErrorID_T write_can_wheel_speed(Speed_Input_T *speed);
void handle_can_error(Can_ErrorID_T error);
uint32_t click_time_to_mRPM(uint32_t cycles_per_click);

void Output_initialize(Output_T *output) {
  output->can->send_driver_output_msg = false;
  output->can->send_raw_values_msg = false;
  output->can->send_wheel_speed_msg = false;

  output->logging->write_throttle_log = false;
  output->logging->write_brake_log = false;
  uint8_t i;
  for (i = 0; i < CS_VALUES_LENGTH; i++) {
    output->logging->write_cs_log[i] = false;
  }
  output->logging->write_mc_data_log = false;
  output->logging->write_mc_state_log = false;
}

void Output_process_output(Input_T *input, State_T *state, Output_T *output) {
  process_can(input, state, output->can);
  process_logging(input, state, output->logging);
}

void process_can(Input_T *input, State_T *state, Can_Output_T *can) {
  if (can->send_driver_output_msg) {
    can->send_driver_output_msg = false;
    handle_can_error(write_can_driver_output(input, state->rules));
  }
  if (can->send_raw_values_msg) {
    can->send_raw_values_msg = false;
    handle_can_error(write_can_raw_values(input->adc));
  }
  if (can->send_wheel_speed_msg) {
    can->send_wheel_speed_msg = false;
    handle_can_error(write_can_wheel_speed(input->speed));
  }
}

void handle_can_error(Can_ErrorID_T error) {
  if (error != Can_Error_NONE && error != Can_Error_NO_RX) {
    /* Serial_Print("can_write_err: "); */
    /* Serial_PrintlnNumber(error, 16); */
    if (!resettingPeripheral) {
      resettingPeripheral = true;
      // TODO add this to CAN library
      CAN_ResetPeripheral();
      Can_Init(500000);
    }
  } else {
    resettingPeripheral = false;
  }
}

Can_ErrorID_T write_can_driver_output(Input_T *input, Rules_State_T *rules) {
  Adc_Input_T *adc = input->adc;
  uint16_t accel_1 = Transform_accel_1(adc->accel_1_raw, TWO_BYTE_MAX);
  uint16_t accel_2 = Transform_accel_2(adc->accel_2_raw, TWO_BYTE_MAX);
  uint16_t accel = min(accel_1, accel_2);

  uint16_t brake = adc->brake_1_raw;
  bool implausible = rules->implausibility_reported;
  bool conflict = rules->has_conflict;

  bool should_zero = implausible || conflict;

  Can_FrontCanNode_DriverOutput_T msg;

  msg.torque = should_zero ? 0 : accel;

  // TODO when adding launch control
  msg.torque_before_control = msg.torque;

  msg.brake_pressure = scale(brake, TEN_BIT_MAX, BYTE_MAX);
  msg.throttle_implausible = implausible;
  msg.brake_throttle_conflict = conflict;

  uint16_t brake_engaged_threshold;
  if (input->misc->hv_enabled) {
    // TODO if we ever see that lv voltage affects brake after all
    /* uint16_t lv_voltage = input->misc->lv_voltage; */
    /* // 750V is about 350 brake */
    /* // 770V is about 390 brake */
    /* // 810V is about 470 brake */
    /* uint16_t lv_max = 810; */
    /* uint16_t lv_min = 750; */
    /* if (lv_voltage < lv_min) { */
    /*   lv_voltage = lv_min; */
    /* } else if (lv_voltage > lv_max) { */
    /*   lv_voltage = lv_max; */
    /* } */
    /* lv_voltage -= lv_min; */
    /*  */
    /* uint16_t brake_min = 350; */
    /* uint16_t brake_min_scaled = brake_min + lv_voltage * 3 / 2; */
    /* brake_engaged_threshold = brake_min_scaled + 50; */
    brake_engaged_threshold = 350;
  } else {
    brake_engaged_threshold = 220;
  }

  msg.brake_engaged = brake > brake_engaged_threshold;
  msg.steering_position = 0;

  /* Serial_Print("torque: "); */
  /* Serial_PrintNumber(msg.torque, 10); */
  /* Serial_Print(", brake: "); */
  /* Serial_PrintNumber(msg.brake_pressure, 10); */
  /* Serial_Print(", engaged: "); */
  /* Serial_Print(msg.brake_engaged ? "true" : "false"); */
  /* Serial_Print(", implaus: "); */
  /* Serial_Print(msg.throttle_implausible ? "true" : "false"); */
  /* Serial_Print(", conflict: "); */
  /* Serial_Println(msg.brake_throttle_conflict ? "true" : "false"); */
  return Can_FrontCanNode_DriverOutput_Write(&msg);
}

Can_ErrorID_T write_can_raw_values(Adc_Input_T *adc) {
  Can_FrontCanNode_RawValues_T msg;

  msg.accel_1_raw = adc->accel_1_raw;
  msg.accel_2_raw = adc->accel_2_raw;
  msg.brake_1_raw = adc->brake_1_raw;
  msg.brake_2_raw = adc->brake_2_raw;

  return Can_FrontCanNode_RawValues_Write(&msg);
}

Can_ErrorID_T write_can_wheel_speed(Speed_Input_T *speed) {
  Can_FrontCanNode_WheelSpeed_T msg;

  uint8_t wheel;
  for (wheel = 0; wheel < NUM_WHEELS; wheel++) {
    uint32_t *ptr;
    if (wheel == LEFT) {
      ptr = &msg.front_left_wheel_speed_mRPM;
    } else if (wheel == RIGHT) {
      ptr = &msg.front_right_wheel_speed_mRPM;
    }
    if (speed->wheel_stopped[wheel]) {
      *ptr = 0;
      continue;
    }
    if (speed->tick_count[wheel] < NUM_TEETH) {
      *ptr = click_time_to_mRPM(speed->tick_us[wheel]);
    } else {
      *ptr = click_time_to_mRPM(speed->moving_avg_us[wheel]);
    }
  }

  return Can_FrontCanNode_WheelSpeed_Write(&msg);
}

uint32_t click_time_to_mRPM(uint32_t us_per_click) {
  const uint32_t us_per_rev = us_per_click * NUM_TEETH;

  const float s_per_rev = us_per_rev / MICROSECONDS_PER_SECOND_F;

  const uint32_t mrev_per_s = (uint32_t)
    (MILLIREVS_PER_REV_F / s_per_rev);

  const uint32_t mrev_per_min = mrev_per_s * SECONDS_PER_MINUTE;
  return mrev_per_min;
}

void process_logging(Input_T *input, State_T *state, Logging_Output_T *logging) {
  UNUSED(input);
  UNUSED(state);
  UNUSED(logging);
}
