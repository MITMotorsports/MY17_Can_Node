#include "chip.h"
// #include "can.h"
// #include "ccand_11xx.h"

#include "adc.h"
#include "output.h"
#include "state.h"
#include "serial.h"
#include "transfer_functions.h"
#include "timer.h"

#include "can.h"

#include "MY17_Can_Library.h"
#include "MY17_Can_Library_Test.h"
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

static bool resettingPeripheral = false;

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
  Can_MsgID_T nextMsg = Can_MsgType();
  if (nextMsg == Can_No_Msg) {
    return;
  } else {
    Frame msg;
    Can_ErrorID_T ok = Can_UnknownRead(&msg);
    if (ok == CAN_ERROR_NONE) {
      /* Serial_Print("CAN_rcv, id="); */
      /* Serial_PrintlnNumber(msg.id, 10); */
    } else {
      Serial_Print("can_read_err: ");
      Serial_PrintlnNumber(ok, 16);
    }
  }
}

void send_driver_output_message(ADC_OUTPUT_T *adc_output) {
  Can_FrontCanNode_DriverOutput_T msg;

  msg.torque = adc_output->requested_torque;

  // TODO when we get brakes
  // msg.brake_pressure = adc_output->brake_pressure;
  msg.brake_pressure = 200;

  msg.steering_position = adc_output->steering_position;
  msg.throttle_implausible = adc_output->throttle_implausible;
  msg.brake_throttle_conflict = adc_output->brake_throttle_conflict;

  /* Serial_Print("torque: "); */
  /* Serial_PrintNumber(msg.torque, 10); */
  /* Serial_Print(", brake_pressure: "); */
  /* Serial_PrintNumber(msg.brake_pressure, 10); */
  /* Serial_Print(", throttle_implausible: "); */
  /* Serial_Print(msg.throttle_implausible ? "true" : "false"); */
  /* Serial_Print(", brake_conflict: "); */
  /* Serial_Print(msg.brake_throttle_conflict ? "true" : "false"); */
  /* Serial_Println(""); */
  Can_ErrorID_T result = Can_FrontCanNode_DriverOutput_Write(&msg);
  if (result != CAN_ERROR_NONE) {
    Serial_Print("driver_write_err: ");
    Serial_PrintlnNumber(result, 16);
    if (!resettingPeripheral) {
      resettingPeripheral = true;
      // TODO add this to CAN library
      CAN_ResetPeripheral();
      CAN_Init(500000);
    }
  } else {
      resettingPeripheral = false;
  }
}

void send_raw_values_message(ADC_OUTPUT_T *adc_output) {
  Can_FrontCanNode_RawValues_T msg;

  msg.accel_1_raw = adc_output->accel_1_raw;
  msg.accel_2_raw = adc_output->accel_2_raw;
  msg.brake_1_raw = adc_output->brake_1_raw;
  msg.brake_2_raw = adc_output->brake_2_raw;

  /* Serial_Print("accel_1: "); */
  /* Serial_PrintNumber(msg.accel_1_raw, 10); */
  /* Serial_Print(", accel_2: "); */
  /* Serial_PrintNumber(msg.accel_2_raw, 10); */
  /* Serial_Print(", brake_1: "); */
  /* Serial_PrintNumber(msg.brake_1_raw, 10); */
  /* Serial_Print(", brake_2: "); */
  /* Serial_PrintNumber(msg.brake_2_raw, 10); */
  /* Serial_Println(""); */
  Can_ErrorID_T result = Can_FrontCanNode_RawValues_Write(&msg);
  if (result != CAN_ERROR_NONE) {
    Serial_Print("raw_write_err: ");
    Serial_PrintlnNumber(result, 16);
    if (!resettingPeripheral) {
      resettingPeripheral = true;
      // TODO add this to CAN library
      CAN_ResetPeripheral();
      CAN_Init(500000);
    }
  } else {
      resettingPeripheral = false;
  }
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
  observe_implausibility(&adc_state);
  report_implausibility(&adc_state);
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
  Can_Init(CAN_BAUDRATE);
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
