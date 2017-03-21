#ifndef ADC_H
#define ADC_H

#include "chip.h"
#include "state_types.h"

#define STEERING_CHANNEL ADC_CH0
#define ACCEL_1_CHANNEL ADC_CH2
#define ACCEL_2_CHANNEL ADC_CH3
#define BRAKE_1_CHANNEL ADC_CH4
#define BRAKE_2_CHANNEL ADC_CH5

#define UNUSED_1_CHANNEL ADC_CH1
#define UNUSED_2_CHANNEL ADC_CH6
#define UNUSED_3_CHANNEL ADC_CH7

#define STEERING_PIN IOCON_PIO0_11
#define ACCEL_1_PIN IOCON_PIO1_1
#define ACCEL_2_PIN IOCON_PIO1_2
#define BRAKE_1_PIN IOCON_PIO1_3
#define BRAKE_2_PIN IOCON_PIO1_4

void ADC_Init(void);

uint16_t ADC_Read(ADC_CHANNEL_T channel);
uint8_t ADC_Read_Byte(ADC_CHANNEL_T channel);
void update_adc_inputs(ADC_INPUT_T *adc_inputs);

#endif //ADC_H
