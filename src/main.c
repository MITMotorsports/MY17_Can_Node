#include "Adc.h"
#include "Input.h"
#include "Output.h"
#include "Serial.h"
#include "state.h"

#include "timer.h"

#include "MY17_Can_Library.h"
/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

const uint32_t OscRateIn = 12000000;

#define SERIAL_BAUDRATE 115200
#define CAN_BAUDRATE 500000

volatile uint32_t msTicks;

// integers in [0:4294967296] representing the number of clock cycles between
// ticks from wheel speed sensors
volatile uint32_t wheel_1_clock_cycles_between_ticks = 0;
volatile uint32_t wheel_2_clock_cycles_between_ticks = 0;

static Input_T input;
static Adc_Input_T adc_input;
static Speed_Input_T speed_input;
static Mc_Input_T mc_input;
static Current_Sensor_Input_T current_sensor_input;
static Misc_Input_T misc_input;

static State_T state;
static Rules_State_T rules_state;
static Message_State_T message_state;

static Output_T output;
static Logging_Output_T logging_output;
static Can_Output_T can_output;

/*****************************************************************************/

 /* Private function */
void SysTick_Handler(void) {
  msTicks++;
}

/****************************************************************************/


// Interrupt handler for timer 0 capture pin. This function get called automatically on
// a rising edge of the signal going into the timer capture pin
void TIMER32_0_IRQHandler(void) {
  Chip_TIMER_Reset(LPC_TIMER32_0);            /* Reset the timer immediately */
  Chip_TIMER_ClearCapture(LPC_TIMER32_0, 0);      /* Clear the capture */
  wheel_1_clock_cycles_between_ticks = Chip_TIMER_ReadCapture(LPC_TIMER32_0, 0);
  Serial_Println("Wheel 1");
}

// Interrupt handler for timer 1 capture pin. This function get called automatically on
// a rising edge of the signal going into the timer capture pin
void TIMER32_1_IRQHandler(void) {
  Chip_TIMER_Reset(LPC_TIMER32_1);            /* Reset the timer immediately */
  Chip_TIMER_ClearCapture(LPC_TIMER32_1, 0);      /* Clear the capture */
  wheel_2_clock_cycles_between_ticks = Chip_TIMER_ReadCapture(LPC_TIMER32_1, 0);
  Serial_Println("Wheel 2");
}

void Set_Interrupt_Priorities(void) {
  /* Give 32 bit timer capture interrupts the highest priority */
  NVIC_SetPriority(TIMER_32_0_IRQn, 0);
  NVIC_SetPriority(TIMER_32_1_IRQn, 1);
  /* Give the SysTick function a lower priority */
  NVIC_SetPriority(SysTick_IRQn, 2);	
}

void initialize_structs(void) {
  input.adc = &adc_input;
  input.speed = &speed_input;
  input.mc = &mc_input;
  input.current_sensor = &current_sensor_input;
  input.misc = &misc_input;

  state.rules = &rules_state;
  state.message = &message_state;

  output.can = &can_output;
  output.logging = &logging_output;
}

/**
 * Receives CAN messages and reads ADCs
 */
void fill_input(void) {
  input.msTicks = msTicks;
  input.speed->wheel_1_click_time = wheel_1_clock_cycles_between_ticks;
  input.speed->wheel_2_click_time = wheel_2_clock_cycles_between_ticks;
  Input_fill_input(&input);
}

void update_state(void) {
  State_update_state(&input, &state, &output);
}

/**
 * Transmits CAN messages
 */
void process_output(void) {
  Output_process_output(&input, &state, &output);
}

int main(void) {

  SystemCoreClockUpdate();

  if (SysTick_Config (SystemCoreClock / 1000)) {
    //Error
    while(1);
  }

  Serial_Init(SERIAL_BAUDRATE);
  Can_Init(CAN_BAUDRATE);

  ADC_Init();
  Timer_Init();

  Set_Interrupt_Priorities();
  Timer_Start();

  initialize_structs();

  Serial_Println("Started up");

  while (1) {
    fill_input();
    update_state();
    process_output();
  }
}
