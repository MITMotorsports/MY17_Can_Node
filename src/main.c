#include "chip.h"
#include "can.h"
#include "ccand_11xx.h"

#include "adc.h"
#include "serial.h"
#include "can_constants.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

const uint32_t OscRateIn = 12000000;

#define SERIAL_BAUDRATE 115200
#define CAN_BAUDRATE 500000

volatile uint32_t msTicks;

const uint32_t adcMessagePeriod_ms = 1000 / FRONT_CAN_NODE_ANALOG_SENSORS__freq;
const uint32_t rpmMessagePeriod_ms = 1000 / FRONT_CAN_NODE_WHEEL_SPEED__freq;

uint32_t lastAdcMessage_ms = 0;
uint32_t lastRpmMessage_ms = 0;

/*****************************************************************************
 * Private function
 ****************************************************************************/

void SysTick_Handler(void) {
  msTicks++;
}

/**
 * @details reads incoming CAN messages and prints information to the terminal
 */
void Process_CAN_Inputs(void) {	
  CCAN_MSG_OBJ_T rx_msg;
  uint32_t ret = CAN_Receive(&rx_msg);

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

void sendADCMessage() {
  uint8_t steering_val = ADC_Read_Byte(STEERING_CHANNEL);
  uint8_t accel_1_val = ADC_Read_Byte(ACCEL_1_CHANNEL);
  uint8_t accel_2_val = ADC_Read_Byte(ACCEL_2_CHANNEL);
  uint8_t brake_1_val = ADC_Read_Byte(BRAKE_1_CHANNEL);
  uint8_t brake_2_val = ADC_Read_Byte(BRAKE_2_CHANNEL);

  /*
  Serial_Print("s: ");
  Serial_PrintNumber(steering_val, 10);
  Serial_Print(", a1: ");
  Serial_PrintNumber(accel_1_val, 10);
  Serial_Print(", a2: ");
  Serial_PrintNumber(accel_2_val, 10);
  Serial_Print(", b1: ");
  Serial_PrintNumber(brake_1_val, 10);
  Serial_Print(", b2: ");
  Serial_PrintlnNumber(brake_2_val, 10);
  */

  const uint32_t can_out_id = FRONT_CAN_NODE_ANALOG_SENSORS__id;
  const uint8_t steering_idx = __FRONT_CAN_NODE_ANALOG_SENSORS__STEERING__start >> 3;
  const uint8_t accel_1_idx = __FRONT_CAN_NODE_ANALOG_SENSORS__RIGHT_ACCEL__start >> 3;
  const uint8_t accel_2_idx = __FRONT_CAN_NODE_ANALOG_SENSORS__LEFT_ACCEL__start >> 3;
  const uint8_t brake_1_idx = __FRONT_CAN_NODE_ANALOG_SENSORS__FRONT_BRAKE__start >> 3;
  const uint8_t brake_2_idx = __FRONT_CAN_NODE_ANALOG_SENSORS__REAR_BRAKE__start >> 3;

  const uint8_t can_out_bytes = 5;
  uint8_t data[can_out_bytes];

  data[steering_idx] = steering_val;
  data[accel_1_idx] = accel_1_val;
  data[accel_2_idx] = accel_2_val;
  data[brake_1_idx] = brake_1_val;
  data[brake_2_idx] = brake_2_val;

  uint32_t ret = CAN_Transmit(can_out_id, data, can_out_bytes);
  if (ret != NO_CAN_ERROR) {
    Serial_Println("CAN error state reached on write attempt:");
    Serial_PrintlnNumber(CAN_GetErrorStatus(), 10);
    Serial_Println("Attempting CAN peripheral reset to clear error...");
    CAN_ResetPeripheral();
  }
}

void sendRpmMessage() {
  // TODO
}

/**
 * Transmits CAN messages
 */
void Process_CAN_Outputs(void) {
  if (msTicks - lastAdcMessage_ms > adcMessagePeriod_ms) {
    lastAdcMessage_ms = msTicks;
    sendADCMessage();
  }
  if (msTicks - lastRpmMessage_ms > rpmMessagePeriod_ms) {
    lastRpmMessage_ms = msTicks;
    sendRpmMessage();
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
    Process_CAN_Inputs();
    Process_CAN_Outputs();
  }
}
