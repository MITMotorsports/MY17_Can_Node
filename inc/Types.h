#ifndef STATE_TYPES_H
#define STATE_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#include <MY17_Can_Library.h>

typedef struct {
  uint16_t accel_1_raw;
  uint16_t accel_2_raw;
  uint16_t brake_1_raw;
  uint16_t brake_2_raw;

  uint32_t last_updated;
} Adc_Input_T;

typedef struct {
  uint32_t wheel_1_click_time;
  uint32_t wheel_2_click_time;
  bool wheel_1_stopped;
  bool wheel_2_stopped;
} Speed_Input_T;

typedef struct {
  Can_MC_RegID_T type;
  int16_t data;
  bool active_current_reduction;
  bool current_reduction_via_igbt_temp;
  bool current_reduction_via_motor_temp;

  uint32_t last_updated;
} Mc_Input_T;

typedef enum {
  CS_Voltage,
  CS_Current,
  CS_Power,
  CS_Energy,
  CS_VALUES_LENGTH
} Current_Sensor_Values_T;

typedef struct {
  int32_t data[CS_VALUES_LENGTH];
  uint32_t last_updated[CS_VALUES_LENGTH];
} Current_Sensor_Input_T;

typedef struct {
  bool hv_enabled;
  uint16_t lv_voltage;
} Misc_Input_T;

typedef struct {
  Adc_Input_T *adc;
  Speed_Input_T *speed;
  Mc_Input_T *mc;
  Current_Sensor_Input_T *current_sensor;
  Misc_Input_T *misc;
  uint32_t msTicks;
} Input_T;

typedef struct {
  // has_conflict is a boolean that is true iff the driver has violated EV2.5
  // and has not yet restored the throttle to less than 5% of pedal travel
  // (EV2.5.1). Note that for this flag we use the lowest reported pedal travel.
  bool has_conflict;

  // implausibility_observed is true iff there is currently an implausibility
  // (difference in reported pedal travel of left and right is > 10%)
  bool implausibility_observed;

  // implausibility_time_ms is set to timestamp of the most recent time that
  // implausibility_observed switched from false to true
  uint32_t implausibility_time_ms;

  // implausibility_reported is true iff implausibility_observed has been true
  // for >100ms (EV2.3.5)
  bool implausibility_reported;

} Rules_State_T;

typedef struct {
  uint32_t can_driver_output_ms;
  uint32_t can_raw_values_ms;
  uint32_t can_wheel_speed_ms;
  uint32_t logging_throttle_ms;
  uint32_t logging_brake_ms;
} Message_State_T;

typedef struct {
  Rules_State_T *rules;
  Message_State_T *message;
} State_T;

typedef struct {
  bool send_driver_output_msg;
  bool send_raw_values_msg;
  bool send_wheel_speed_msg;
} Can_Output_T;

typedef struct {
  bool write_throttle_log;
  bool write_brake_log;
  bool write_cs_log[CS_VALUES_LENGTH];
  bool write_mc_data_log;
  bool write_mc_state_log;
} Logging_Output_T;

typedef struct {
  Can_Output_T *can;
  Logging_Output_T *logging;
} Output_T;

#endif
