#include "chip.h"
#include "can.h"
#include "ccand_11xx.h"
#include "board.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define BAUDRATE 115200

const uint32_t OscRateIn = 12000000;

volatile uint32_t msTicks;

uint32_t last_can_message_sent_time = 0; 

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
		Board_Println("Received CAN message.");

		Board_Print("ID: ");
		Board_Println_Int(rx_msg.mode_id, 10);

		Board_Println("Data: ");
		Board_Print("data[0]: ");
		Board_Println_Int(rx_msg.data[0], 2);
		Board_Print("data[1]: ");
		Board_Println_Int(rx_msg.data[1], 2);
		Board_Print("data[2]: ");
		Board_Println_Int(rx_msg.data[2], 2);
		Board_Print("data[3]: ");
		Board_Println_Int(rx_msg.data[3], 2);
		Board_Print("data[4]: ");
		Board_Println_Int(rx_msg.data[4], 2);
		Board_Print("data[5]: ");
		Board_Println_Int(rx_msg.data[5], 2);
		Board_Print("data[6]: ");
		Board_Println_Int(rx_msg.data[6], 2);
		Board_Print("data[7]: ");
		Board_Println_Int(rx_msg.data[7], 2);

	}
}

/**
 * Transmits CAN messages
 */
void Process_CAN_Outputs(void) {
	//Send CAN message every second
	const uint16_t one_second = 1000;
	if (msTicks - last_can_message_sent_time > one_second) {	
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
		Board_Println("Sent CAN message");
		last_can_message_sent_time = msTicks;
	}
}

int main(void) {

	SystemCoreClockUpdate();

	uint32_t reset_can_peripheral_time = 0;
	bool reset_can_peripheral = false;

	if (SysTick_Config (SystemCoreClock / 1000)) {
		//Error
		while(1);
	}

	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_6, (IOCON_FUNC1 | IOCON_MODE_INACT)); /* RXD */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_7, (IOCON_FUNC1 | IOCON_MODE_INACT)); /* TXD */
	
	Chip_UART_Init(LPC_USART);
	Chip_UART_SetBaud(LPC_USART, BAUDRATE);
	// Configure data width, parity, and stop bits
	Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS));
	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(LPC_USART);

	Board_Println("Started up");

	CAN_Init(500000);

	while (1) {
       		 if( reset_can_peripheral && (msTicks > reset_can_peripheral_time) ) {
            		Board_Println("Attempting to reset CAN peripheral...");
            		CAN_ResetPeripheral();
            		CAN_Init(500000);
            		Board_Println("Reset CAN peripheral.");
            		reset_can_peripheral = false;
        	}

		Process_CAN_Inputs();
		Process_CAN_Outputs();

	}
}
