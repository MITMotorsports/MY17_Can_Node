#ifndef STATE_H
#define STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "state_types.h"

/**
 * Takes in raw input from analog sensors and converts it to percentage of
 * travel by applying the relevant transfer functions.
 *
 * @param adc_input: struct containing analog readings fetched from ADC peripheral.
 * @param adc_state: struct to be populated with abstract travel of each sensor.
 * @effect: adc_state contains the amount of pedal travel indicated by each
 *    analog sensor.
 */
void read_input(ADC_INPUT_T *adc_input, ADC_STATE_T *adc_state);

/**
 * Examines amount of pedal travel to see if an implausibility is observed,
 * recording whether the current readings are implausible. If so,
 * also records time that current implausibilty began.
 *
 * @param adc_state: struct containing populated amount of observed pedal travel
 *    for each sensor, as well as un-populated fields for output and time in ms.
 * @effect: adc_state contains a flag indicating whether or not the current
 *    readings are plausible. If implausible, adc_state also contains the time
 *    that the current implausibility was first observed.
 */
void observe_plausibility(ADC_STATE_T *adc_state);

/**
 * Examines whether there is an implausibility as well as the time that it was
 * first observed, and decides whether to report the implausibility. By rules,
 * implausibilities must stop the car 100ms after first observed.
 *
 * @param adc_state: struct containing fields indicating whether implausibility
 *    has occured and the time of first observation, as well as un-populated
 *    fields for whether implausibility should be reported.
 * @effect: adc_state contains a flag indicating whether or not the system
 *    should report an implausibility.
 */
void report_plausibility(ADC_STATE_T *adc_state);

/**
 * Examines relative position of brake and accelerator and decides whether or
 * not the driver is pressing them at the same time. By EV2.9, the car should
 * shut off if the throttle is >25% engaged while the brakes are actuated, and
 * must not resume applying torque until the throttle is <5% engaged.
 *
 * @param adc_state: struct containing fields indicating pedal travel of
 *    throttle and pressure of brakes, as well as whether the current readings
 *    are plausible.
 * @effect: adc_state contains a flag indicating whether or not the system
 *    should report that EV2.9 is being violated.
 */
void check_conflict(ADC_STATE_T *adc_output);

/**
 * Logic for the actual plausibility check. By EV2.5, the amount of pedal
 * travel observed for each of the APPS must be within 10% of total possible
 * travel. Since pedal travel is in units of 0.1%, this means the two values
 * must be within 100.
 *
 * @param accel_1_val: integer in [0:1000] representing pedal travel
 *    observed by one of the throttle potentiometers.
 * @param accel_2_val: integer in [0:1000] representing pedal travel
 *    observed by the other throttle potentiometer.
 * @return true iff accel_1_val and accel_2_val are plausible.
 */
bool check_plausibility(uint16_t accel_1_val, uint16_t accel_2_val);

#endif // STATE_H
