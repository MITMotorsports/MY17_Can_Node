#include "Adc.h"
#include "Input.h"
#include "Output.h"
#include "Serial.h"
#include "State.h"

#include "Timer.h"

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
volatile uint32_t last_tick[NUM_WHEELS][NUM_TEETH];

volatile uint32_t num_ticks[NUM_WHEELS];
volatile uint64_t big_sum[NUM_WHEELS];
volatile uint64_t little_sum[NUM_WHEELS];

volatile bool disregard[NUM_WHEELS];

volatile uint32_t last_updated[NUM_WHEELS];

#define WHEEL_SPEED_TIMEOUT_MS 100

uint32_t last_speed_read_ms = 0;
#define WHEEL_SPEED_READ_PERIOD_MS 10

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

void handle_interrupt(LPC_TIMER_T* timer, Wheel_T wheel) {
  Chip_TIMER_Reset(timer);            /* Reset the timer immediately */
  Chip_TIMER_ClearCapture(timer, 0);      /* Clear the capture */
  const uint32_t curr_tick = Chip_TIMER_ReadCapture(timer, 0) / CYCLES_PER_MICROSECOND;

  // Interrupt can now proceed

  if (disregard[wheel]) {
    num_ticks[wheel] = 0;
    big_sum[wheel] = 0;
    little_sum[wheel] = 0;
    last_updated[wheel] = msTicks;
    return;
  }

  const uint32_t count = num_ticks[wheel];
  const uint8_t idx = count % NUM_TEETH;
  const uint32_t this_tooth_last_rev =
    count < NUM_TEETH ? 0 : last_tick[wheel][idx];

  // Register tick
  last_tick[wheel][idx] = curr_tick;
  num_ticks[wheel]++;

  // Update big sum
  big_sum[wheel] += NUM_TEETH * curr_tick;
  big_sum[wheel] -= little_sum[wheel];

  // Update little sum
  little_sum[wheel] += curr_tick;
  little_sum[wheel] -= this_tooth_last_rev;

  // Update timestamp
  last_updated[wheel] = msTicks;
}

// Interrupt handler for timer 0 capture pin. This function get called automatically on
// a rising edge of the signal going into the timer capture pin
void TIMER32_0_IRQHandler(void) {
  handle_interrupt(LPC_TIMER32_0, LEFT);
}

// Interrupt handler for timer 1 capture pin. This function get called automatically on
// a rising edge of the signal going into the timer capture pin
void TIMER32_1_IRQHandler(void) {
  handle_interrupt(LPC_TIMER32_1, RIGHT);
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

  uint8_t wheel;
  for(wheel = 0; wheel < NUM_WHEELS; wheel++) {
    uint8_t tooth;
    for(tooth = 0; tooth < NUM_TEETH; tooth++) {
      last_tick[wheel][tooth] = 0;
    }
    num_ticks[wheel] = 0;
    big_sum[wheel] = 0;
    little_sum[wheel] = 0;
    disregard[wheel] = false;
    last_updated[wheel] = 0;
  }

  Input_initialize(&input);
  State_initialize(&state);
  Output_initialize(&output);
}

/**
 * Receives CAN messages and reads ADCs
 */
void fill_input(void) {
  input.msTicks = msTicks;

  if (last_speed_read_ms + WHEEL_SPEED_READ_PERIOD_MS < msTicks) {
    // Capture values
    last_speed_read_ms = msTicks;
    uint8_t wheel;
    for(wheel = 0; wheel < NUM_WHEELS; wheel++) {
      const uint32_t count = num_ticks[wheel];
      uint8_t idx;
      if (count > 0) {
        // If there are x ticks so far, the last tick is index (x - 1)
        idx = (count - 1) % NUM_TEETH;
      } else {
        idx = 0;
      }
      input.speed->tick_count[wheel] = count;
      input.speed->tick_us[wheel] = last_tick[wheel][idx];
      if (count < NUM_TEETH) {
        input.speed->moving_avg_us[wheel] = 0;
      } else {
        const uint32_t avg = big_sum[wheel] / SUM_ALL_TEETH;
        input.speed->moving_avg_us[wheel] = avg;
      }
      const bool timeout =
        last_updated[wheel] + WHEEL_SPEED_TIMEOUT_MS < msTicks;
      input.speed->wheel_stopped[wheel] = timeout || count == 0;
      disregard[wheel] = timeout;
    }
  }
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
