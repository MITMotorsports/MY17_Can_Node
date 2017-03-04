#include "can_utils.h"

#include <stdlib.h>

#include "ccand_11xx.h"
#include "can_constants.h"
#include "board.h"
#include "util.h"

#define DEBUG 1

uint64_t construct_64_bit_can_message(CCAN_MSG_OBJ_T * msg_obj) {
	const uint8_t bytes_in_can_message = 8;
	const uint8_t bits_in_byte = 8;
	uint64_t can_message = 0;
	Board_Println("In construct_64_bit_can_message");
	char can_message_str[100];
	char i_str[100];
	uint64_t data_i;
	char data_i_str[100];
	uint32_t left_shift;
	char left_shift_str[10];
	uint64_t shifted_data_i;
	uint8_t i;
	for (i=0; i<bytes_in_can_message; i++) {
		data_i = msg_obj->data[i];
		left_shift = (bytes_in_can_message - 1 - i)*bits_in_byte;
		shifted_data_i = data_i << left_shift;
		can_message = can_message | shifted_data_i;
		
		Board_Print("i: ");
		itoa(i, i_str, 10);
		Board_Println(i_str);

		Board_Print("left_shift: ");
		Board_Println_Int(left_shift, 10);

		Board_Print("shifted_data_i ");
		Board_Println_Int(shifted_data_i, 2);
		
		Board_Print("data_i: ");
		itoa(data_i, data_i_str, 2);
		Board_Println(data_i_str);
		
		itoa(can_message, can_message_str, 2);
		Board_Print("can message: ");
		Board_Println(can_message_str);
	}
	Board_Println("n");
	uint32_t n = 0;
	char n_str[10];
	for (i=0; i<10; i++) {
		Board_Print("left_shift: ");
		left_shift = 10-i-1;
		itoa(left_shift, left_shift_str, 10);
		Board_Println(left_shift_str);

		n = n | (1 << left_shift);
		itoa(n, n_str, 2);
		Board_Println(n_str);
	}
	uint64_t my_int = 1;
	uint64_t my_shifted_int = my_int << (32);
	Board_Print("my_shifted_int: ");
	Board_Println_Int(my_shifted_int, 2);
	return can_message;
}

void CAN_MakeBMSHeartbeat(BMS_HEARTBEAT_T * bms_heartbeat, CCAN_MSG_OBJ_T * msg_obj) {
	uint64_t can_message = construct_64_bit_can_message(msg_obj);
	if (DEBUG) {
		char can_message_64_bits[100];
		Board_Println("Inside CAN_MakeBMSHeartbeat. Just before itoa. Expect new message soon");
		itoa(can_message, can_message_64_bits, 2);
		Board_Print("BMS Heartbeat CAN Message (64 bits): ");
		Board_Print(can_message_64_bits);
		Board_Print("\r\n");
	}

	//get state and soc
	const uint8_t CAN_message_highest_bit = 63;
	//TODO: don't hard code the masks
	const uint64_t state_mask =          0xE000000000000000;
	const uint64_t soc_percentage_mask = 0x1FF8000000000000;
	uint64_t state = (can_message & state_mask) >> 
		(CAN_message_highest_bit - __BMS_HEARTBEAT__STATE__start);
	uint64_t soc_percentage = (can_message & soc_percentage_mask) >> 
		(CAN_message_highest_bit - __BMS_HEARTBEAT__SOC_PERCENTAGE__start);
	
	//construct BMS_HEARTBEAT_T
	bms_heartbeat->state = state;
	bms_heartbeat->soc_percentage = soc_percentage;
}

void CAN_MakeBMSDischargeResponse(BMS_DISCHARGE_RESPONSE_T * bms_discharge_response, CCAN_MSG_OBJ_T * msg_obj) {
        const uint32_t CAN_message_highest_bit = 63;
        const uint64_t bms_discharge_response_mask = 0x8000000000000000;
        uint64_t discharge_response = (msg_obj->data_64 & bms_discharge_response_mask) >> 
		(CAN_message_highest_bit - __BMS_DISCHARGE_RESPONSE__DISCHARGE_RESPONSE__end);

        bms_discharge_response->discharge_response = discharge_response;
}
