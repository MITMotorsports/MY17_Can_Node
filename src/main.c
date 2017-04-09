#include "chip.h"
#include "can.h"
#include "ccand_11xx.h"

#include "adc.h"
#include "can_constants.h"
#include "output.h"
#include "state.h"
#include "serial.h"
#include "transfer_functions.h"
#include "timer.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

const uint32_t OscRateIn = 12000000;

#define SERIAL_BAUDRATE 115200
#define CAN_BAUDRATE 500000

volatile uint32_t msTicks;

// integers in [0:4294967296] representing the number of clock cycles between
// ticks from wheel speed sensors
volatile uint32_t wheel_1_clock_cycles_between_ticks;
volatile uint32_t wheel_2_clock_cycles_between_ticks;

const uint32_t DRIVER_OUTPUT_MESSAGE_PERIOD_MS = 20;
const uint32_t RPM_MESSAGE_PERIOD_MS = 20;

const uint32_t RAW_VALUES_MESSAGE_PERIOD_MS = 100;

uint32_t lastRawValuesMessage_ms = 0;
uint32_t lastDriverOutputMessage_ms = 0;
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

// Interrupt handler for timer 0 capture pin. This function get called automatically on 
// a rising edge of the signal going into the timer capture pin
void TIMER32_0_IRQHandler(void) {
  Chip_TIMER_Reset(LPC_TIMER32_0);		        /* Reset the timer immediately */
  Chip_TIMER_ClearCapture(LPC_TIMER32_0, 0);	    /* Clear the capture */
  wheel_1_clock_cycles_between_ticks = Chip_TIMER_ReadCapture(LPC_TIMER32_0, 0);
}

// Interrupt handler for timer 1 capture pin. This function get called automatically on 
// a rising edge of the signal going into the timer capture pin
void TIMER32_1_IRQHandler(void) {
  Chip_TIMER_Reset(LPC_TIMER32_1);		        /* Reset the timer immediately */
  Chip_TIMER_ClearCapture(LPC_TIMER32_1, 0);	    /* Clear the capture */
  wheel_2_clock_cycles_between_ticks = Chip_TIMER_ReadCapture(LPC_TIMER32_1, 0);
}

void Initalize_Global_Variables(void) {
  wheel_1_clock_cycles_between_ticks = 0;
  wheel_2_clock_cycles_between_ticks = 0;
}

void Set_Interrupt_Priorities(void) {
  /* Give 32 bit timer capture interrupts the highest priority */
  NVIC_SetPriority(TIMER_32_0_IRQn, 0);
  NVIC_SetPriority(TIMER_32_1_IRQn, 1);
  /* Give the SysTick function a lower priority */
  NVIC_SetPriority(SysTick_IRQn, 2);	
}

void Init_ADC_Structs(void) {
  adc_input.steering_raw = 0;
  adc_input.accel_1_raw = 0;
  adc_input.accel_2_raw = 0;
  adc_input.brake_1_raw = 0;
  adc_input.brake_2_raw = 0;
  adc_input.lastUpdate_ms = 0;
  adc_input.msTicks = msTicks;

  adc_state.steering_raw = 0;
  adc_state.accel_1_raw = 0;
  adc_state.accel_2_raw = 0;
  adc_state.brake_1_raw = 0;
  adc_state.brake_2_raw = 0;
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

  adc_output.steering_raw = 0;
  adc_output.accel_1_raw = 0;
  adc_output.accel_2_raw = 0;
  adc_output.brake_1_raw = 0;
  adc_output.brake_2_raw = 0;
  adc_output.requested_torque = 0;
  adc_output.brake_pressure = 0;
  adc_output.steering_position = 0;
  adc_output.throttle_implausible = false;
  adc_output.brake_throttle_conflict = false;
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

#define TOGGLE(input, test, idx) \
  if (test) {\
    ((input) |= (1 << (7 - (idx)))); \
  } else { \
    ((input) &= ~(1 << (7 - (idx)))); \
  } \

#define CHECK(a,b) (((a) & (1<<(7- (b)))) != 0)

void send_driver_output_message(ADC_OUTPUT_T *adc_output) {
  const uint32_t can_out_id = FRONT_CAN_NODE_ANALOG_SENSORS__id;
  const uint8_t requested_torque_low = 0;
  const uint8_t requested_torque_high = 1;
  const uint8_t brake_pressure_idx = 2;
  const uint8_t steering_position_idx = 3;
  const uint8_t flag_idx = 4;

  // 5 bytes out
  uint8_t data[5] = {0};

  data[requested_torque_low] = adc_output->requested_torque & 0xFF;
  data[requested_torque_high] = (adc_output->requested_torque & 0xFF00) >> 8;
  data[brake_pressure_idx] = adc_output->brake_pressure;
  data[steering_position_idx] = adc_output->steering_position;
  TOGGLE(data[flag_idx], adc_output->throttle_implausible, 0);
  TOGGLE(data[flag_idx], adc_output->brake_throttle_conflict, 1);

  uint32_t ret = CAN_Transmit(can_out_id, data, 5);
  if (ret != NO_CAN_ERROR) {
    Serial_Println("CAN error state reached on write attempt:");
    Serial_PrintlnNumber(CAN_GetErrorStatus(), 10);
    Serial_Println("Attempting CAN peripheral reset to clear error...");
    CAN_ResetPeripheral();
  }
}

void send_raw_values_message(ADC_OUTPUT_T *adc_output) {
  /* const uint32_t can_out_id = 0x230; */

  #define LENGTH 8

  const uint32_t can_out_id = 0x230;
  uint8_t data[LENGTH] = {0};
  data[0] = adc_output->accel_1_raw & 0xFF;
  data[1] = (adc_output->accel_1_raw & 0xFF00) >> 8;
  data[2] = adc_output->accel_2_raw & 0xFF;
  data[3] = (adc_output->accel_2_raw & 0xFF00) >> 8;
  data[4] = adc_output->brake_1_raw & 0xFF;
  data[5] = (adc_output->brake_1_raw & 0xFF00) >> 8;
  data[6] = adc_output->brake_2_raw & 0xFF;
  data[7] = (adc_output->brake_2_raw & 0xFF00) >> 8;

  Serial_PrintNumber(adc_output->accel_1_raw, 10);
  Serial_Print(", ");
  Serial_PrintNumber(adc_output->accel_2_raw, 10);
  Serial_Print(", ");
  Serial_PrintNumber(adc_output->brake_1_raw, 10);
  Serial_Print(", ");
  Serial_PrintNumber(adc_output->brake_2_raw, 10);
  Serial_Print(", ");
  Serial_PrintNumber(adc_output->steering_raw, 10);
  Serial_Println("");

  uint32_t ret = CAN_Transmit(can_out_id, data, LENGTH);
  if (ret != NO_CAN_ERROR) {
    Serial_Println("CAN error state reached on write attempt:");
    Serial_PrintlnNumber(CAN_GetErrorStatus(), 10);
    Serial_Println("Attempting CAN peripheral reset to clear error...");
    CAN_ResetPeripheral();
  }

  #undef LENGTH

}

void send_rpm_message(void) {
  // TODO
}

/**
 * Receives CAN messages and reads ADCs
 */
void handle_inputs(void) {
  adc_input.msTicks = msTicks;
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
  update_adc_outputs(&adc_state, &adc_output);

  if (msTicks - lastDriverOutputMessage_ms > DRIVER_OUTPUT_MESSAGE_PERIOD_MS) {
    lastDriverOutputMessage_ms = msTicks;
    send_driver_output_message(&adc_output);
  }

  if (msTicks - lastRawValuesMessage_ms > RAW_VALUES_MESSAGE_PERIOD_MS) {
    lastRawValuesMessage_ms = msTicks;
    send_raw_values_message(&adc_output);
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

  Initalize_Global_Variables();
  Init_ADC_Structs();
  Serial_Init(SERIAL_BAUDRATE);
  CAN_Init(CAN_BAUDRATE);
  ADC_Init();
  Timer_Init();

  Set_Interrupt_Priorities();

  Timer_Start();

  Serial_Println("Started up");

  while (1) {
    handle_inputs();
    update_state();
    handle_outputs();
  }
}
