#ifndef STATE_TYPES_H
#define STATE_TYPES_H

#include <stdint.h>
#include <stdbool.h>

typedef struct ADC_INPUT {
  // The following are all integers in [0:1023] representing output from some
  // analog sensor on the car.
  uint16_t accel_1_raw;
  uint16_t accel_2_raw;
  uint16_t brake_1_raw;
  uint16_t brake_2_raw;
  uint16_t steering_raw;

  // TODO consider moving timing logic out of this struct.

  // Time in ms that the ADC peripheral was last read from.
  uint32_t lastUpdate_ms;
  // Current time in ms.
  uint32_t msTicks;
} ADC_INPUT_T;

typedef struct ADC_STATE {

  // Copies of values for driver out
  uint16_t accel_1_raw;
  uint16_t accel_2_raw;
  uint16_t brake_1_raw;
  uint16_t brake_2_raw;
  uint16_t steering_raw;

  // steering_val is an integer ranging from [0:1000] representing degree of
  // steering travel, where 0 is fully right and 1000 is fully left.
  uint16_t steering_travel;

  // accel_*_travel is an integer ranging from [0:1000] representing percentage
  // of throttle pedal travel in units of 0.1 percent.
  uint16_t accel_1_travel;
  uint16_t accel_2_travel;

  // brake_travel is an integer ranging from [0:1000] representing percentage
  // of brake pressure in units of 0.1 percent.
  uint16_t brake_1_travel;
  uint16_t brake_2_travel;

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

  // Flag that is true in iterations where one of the following occurs:
  // 1) implausibility_reported switches from false to true.
  // 2) has_conflict switches from true to false.
  //
  // This is because these are two things that require very fast response and
  // indicate that a CAN message should be sent immediately, rather than
  // waiting for the next scheduled sending.
  bool urgent_message;

  // TODO consider moving timing logic out of this struct as well.

  // Current time in ms.
  uint32_t msTicks;
} ADC_STATE_T;

typedef struct ADC_OUTPUT {

  uint16_t accel_1_raw;
  uint16_t accel_2_raw;
  uint16_t brake_1_raw;
  uint16_t brake_2_raw;
  uint16_t steering_raw;

  // Amount of torque that the CAN node decides to output in [-32768:32767]
  // TODO decide endian-ness and whether we will clamp this.
  int16_t requested_torque;

  // Amount of brake pressure in [0:255]
  uint8_t brake_pressure;

  // Steering position in [0:255] with 0 being fully right.
  uint8_t steering_position;

  bool throttle_implausible;
  bool brake_throttle_conflict;

  bool brake_engaged;
} ADC_OUTPUT_T;

#endif
