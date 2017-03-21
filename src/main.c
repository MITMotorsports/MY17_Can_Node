#include "chip.h"
#include "can.h"
#include "ccand_11xx.h"

#include "adc.h"
#include "can_constants.h"
#include "output.h"
#include "state.h"
#include "serial.h"
#include "transfer_functions.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

const uint32_t OscRateIn = 12000000;

#define SERIAL_BAUDRATE 115200
#define CAN_BAUDRATE 500000

volatile uint32_t msTicks;

const uint32_t ADC_MESSAGE_PERIOD_MS = 1000 / FRONT_CAN_NODE_ANALOG_SENSORS__freq;
const uint32_t RPM_MESSAGE_PERIOD_MS = 1000 / FRONT_CAN_NODE_WHEEL_SPEED__freq;

uint32_t lastAdcMessage_ms = 0;
uint32_t lastRpmMessage_ms = 0;

static ADC_INPUT_T adc_input;
static ADC_STATE_T adc_state;
static ADC_OUTPUT_T adc_output;

/*****************************************************************************
 * Private function
 ****************************************************************************/

void SysTick_Handler(void) {
  msTicks++;
}

void Init_ADC_Structs(void) {
  adc_input.steering_raw = 0;
  adc_input.accel_1_raw = 0;
  adc_input.accel_2_raw = 0;
  adc_input.brake_1_raw = 0;
  adc_input.brake_2_raw = 0;
  adc_input.lastUpdate_ms = 0;
  adc_input.msTicks = msTicks;

  adc_state.steering_travel = 0;
  adc_state.accel_1_travel = 0;
  adc_state.accel_2_travel = 0;
  adc_state.brake_1_travel = 0;
  adc_state.brake_2_travel = 0;
  adc_state.has_conflict = false;
  adc_state.implausibility_observed = false;
  adc_state.implausibility_time_ms = 0;
  adc_state.implausibility_reported = false;
  adc_state.urgent_message = false;
  adc_state.msTicks = msTicks;

  adc_output.requested_torque = 0;
  adc_output.brake_pressure = 0;
  adc_output.steering_position = 0;
}

/**
 * @details reads incoming CAN messages and prints information to the terminal
 */
void update_can_inputs(void) {	
  CCAN_MSG_OBJ_T rx_msg;
  CAN_ERROR_T ret = CAN_Receive(&rx_msg);

  if (ret == NO_CAN_ERROR) {
    Serial_Print("CAN_rcv, id=");
    Serial_PrintlnNumber(rx_msg.mode_id, 10);
  } else if (ret != NO_RX_CAN_MESSAGE) {
    Serial_Println("CAN error state reached on receive attempt:");
    Serial_PrintlnNumber(CAN_GetErrorStatus(), 10);
    Serial_Println("Attempting CAN peripheral reset to clear error...");
    CAN_ResetPeripheral();
  }
}

void send_adc_message(ADC_OUTPUT_T *adc_output) {

  const uint32_t can_out_id = FRONT_CAN_NODE_ANALOG_SENSORS__id;
  const uint8_t requested_torque_idx = 0;
  const uint8_t brake_pressure_idx = 1;
  const uint8_t steering_position_idx = 2;

  const uint8_t can_out_bytes = 3;
  uint8_t data[can_out_bytes];

  data[requested_torque_idx] = adc_output->requested_torque;
  data[brake_pressure_idx] = adc_output->brake_pressure;
  data[steering_position_idx] = adc_output->steering_position;

  uint32_t ret = CAN_Transmit(can_out_id, data, can_out_bytes);
  if (ret != NO_CAN_ERROR) {
    Serial_Println("CAN error state reached on write attempt:");
    Serial_PrintlnNumber(CAN_GetErrorStatus(), 10);
    Serial_Println("Attempting CAN peripheral reset to clear error...");
    CAN_ResetPeripheral();
  }

}

void send_rpm_message(void) {
  // TODO
}

/**
 * Receives CAN messages and reads ADCs
 */
void handle_inputs(void) {
  update_can_inputs();
  update_adc_inputs(&adc_input);
}

void update_state(void) {
  read_input(&adc_input, &adc_state);
  observe_plausibility(&adc_state);
  report_plausibility(&adc_state);
  check_conflict(&adc_state);
}

/**
 * Transmits CAN messages
 */
void handle_outputs(void) {
  if (msTicks - lastAdcMessage_ms > ADC_MESSAGE_PERIOD_MS) {
    lastAdcMessage_ms = msTicks;
    update_adc_outputs(&adc_state, &adc_output);
    send_adc_message(&adc_output);
  }
  if (msTicks - lastRpmMessage_ms > RPM_MESSAGE_PERIOD_MS) {
    lastRpmMessage_ms = msTicks;
    send_rpm_message();
  }
}

int main(void) {

  SystemCoreClockUpdate();

  if (SysTick_Config (SystemCoreClock / 1000)) {
    //Error
    while(1);
  }

  Serial_Init(SERIAL_BAUDRATE);
  CAN_Init(CAN_BAUDRATE);
  ADC_Init();

  Serial_Println("Started up");

  while (1) {
    handle_inputs();
    update_state();
    handle_outputs();
  }
}
