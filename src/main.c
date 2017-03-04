#include "chip.h"
#include "can.h"
#include "ccand_11xx.h"
#include "serial.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

const uint32_t OscRateIn = 12000000;

#define SERIAL_BAUDRATE 115200
#define CAN_BAUDRATE 500000

volatile uint32_t msTicks;

const uint32_t adcMessagePeriod_ms = 50;
const uint32_t rpmMessagePeriod_ms = 20;

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

  if (ret == NO_RX_CAN_MESSAGE) {
    // No message received.
    // TODO really need a way to distinguish no message available
    // from an error while receiving messages...
  } else if (ret == NO_CAN_ERROR) {
    Serial_Print("CAN_rcv, id=");
    Serial_PrintlnNumber(rx_msg.mode_id, 10);
  } else {
    // Should never happen
    Serial_Println("Unexpected CAN error state!!");
  }
}

void sendAdcMessage() {
  const uint8_t can_out_id = 1;
  const uint8_t can_out_bytes = 8;

  uint8_t data[can_out_bytes];
  data[0] = 0x01;
  data[1] = 0x12;
  data[2] = 0x23;
  data[3] = 0x34;
  data[4] = 0x45;
  data[5] = 0x56;
  data[6] = 0x67;
  data[7] = 0x78;

  CAN_Transmit(can_out_id, data, can_out_bytes);
  Serial_Println("Sent CAN message");
}

void sendRpmMessage() {

}

/**
 * Transmits CAN messages
 */
void Process_CAN_Outputs(void) {
  //Send CAN message every second
  if (msTicks - lastAdcMessage_ms > adcMessagePeriod_ms) {
    lastAdcMessage_ms = msTicks;
    sendAdcMessage();
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

  Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_6, (IOCON_FUNC1 | IOCON_MODE_INACT)); /* RXD */
  Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_7, (IOCON_FUNC1 | IOCON_MODE_INACT)); /* TXD */

  Chip_UART_Init(LPC_USART);
  Chip_UART_SetBaud(LPC_USART, SERIAL_BAUDRATE);
  // Configure data width, parity, and stop bits
  Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS));
  Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
  Chip_UART_TXEnable(LPC_USART);

  Serial_Println("Started up");

  CAN_Init(CAN_BAUDRATE);

  while (1) {
    Process_CAN_Inputs();
    Process_CAN_Outputs();
  }
}
