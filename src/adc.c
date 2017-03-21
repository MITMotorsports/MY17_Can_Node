#include "adc.h"
#include "serial.h"

#include <stdint.h>
#include <string.h>

#include "chip.h"
#include "transfer_functions.h"

#define ADC_UPDATE_PERIOD_MS 10

static ADC_CLOCK_SETUP_T adc_setup;

void ADC_Init(void) {
  const uint32_t ADC_PIN_CONFIG = IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_ADMODE_EN;
  Serial_Print("Setting config to ");
  Serial_PrintlnNumber(ADC_PIN_CONFIG, 2);

  Chip_ADC_Init(LPC_ADC, &adc_setup);

  // Steering / AD0
  Chip_IOCON_PinMuxSet(LPC_IOCON, STEERING_PIN, ADC_PIN_CONFIG);
  Chip_ADC_EnableChannel(LPC_ADC, STEERING_CHANNEL, ENABLE);
  // Accel 1 / AD2
  Chip_IOCON_PinMuxSet(LPC_IOCON, ACCEL_1_PIN, ADC_PIN_CONFIG);
  Chip_ADC_EnableChannel(LPC_ADC, ACCEL_1_CHANNEL, ENABLE);
  // Accel 2 / AD3
  Chip_IOCON_PinMuxSet(LPC_IOCON, ACCEL_2_PIN, ADC_PIN_CONFIG);
  Chip_ADC_EnableChannel(LPC_ADC, ACCEL_2_CHANNEL, ENABLE);
  // Brake 1 / AD4
  Chip_IOCON_PinMuxSet(LPC_IOCON, BRAKE_1_PIN, ADC_PIN_CONFIG);
  Chip_ADC_EnableChannel(LPC_ADC, BRAKE_1_CHANNEL, ENABLE);
  // Brake 2 / AD5
  Chip_IOCON_PinMuxSet(LPC_IOCON, BRAKE_2_PIN, ADC_PIN_CONFIG);
  Chip_ADC_EnableChannel(LPC_ADC, BRAKE_2_CHANNEL, ENABLE);

  // Disable all other channels
  // TODO re-enable ones for wheel speed
  Chip_ADC_EnableChannel(LPC_ADC, UNUSED_1_CHANNEL, DISABLE);
  Chip_ADC_EnableChannel(LPC_ADC, UNUSED_2_CHANNEL, DISABLE);
  Chip_ADC_EnableChannel(LPC_ADC, UNUSED_3_CHANNEL, DISABLE);

  // Enable burst
  Chip_ADC_SetBurstCmd(LPC_ADC, ENABLE);
  Chip_ADC_SetStartMode(LPC_ADC, ADC_NO_START, ADC_TRIGGERMODE_RISING);
}

uint16_t ADC_Read(ADC_CHANNEL_T channel) {
  uint16_t result = 0;
  Chip_ADC_ReadValue(LPC_ADC, channel, &result);
  return result;
}

uint8_t ADC_Read_Byte(ADC_CHANNEL_T channel) {
  uint16_t ten_bit_val = ADC_Read(channel);
  uint8_t eight_bit_val = ten_bit_val >> 2;
  return eight_bit_val;
}

void update_adc_inputs(ADC_INPUT_T *adc_input) {
  uint32_t nextUpdate_ms = adc_input->lastUpdate_ms + ADC_UPDATE_PERIOD_MS;
  if (adc_input->msTicks >= nextUpdate_ms) {
    adc_input->accel_1_raw = ADC_Read(ACCEL_1_CHANNEL);
    adc_input->accel_2_raw = ADC_Read(ACCEL_2_CHANNEL);
    adc_input->brake_1_raw = ADC_Read(BRAKE_1_CHANNEL);
    adc_input->brake_2_raw = ADC_Read(BRAKE_2_CHANNEL);
    adc_input->steering_raw = ADC_Read(STEERING_CHANNEL);
    adc_input->lastUpdate_ms = adc_input->msTicks;
  }
}
