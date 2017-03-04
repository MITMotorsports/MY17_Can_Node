#include "board.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "chip.h"

uint32_t Board_Print(const char *str) {
	//char * str1 = "Entered Board_Print ";
	//Chip_UART_SendBlocking(LPC_USART, str1, strlen(str));
	uint32_t count = Chip_UART_SendBlocking(LPC_USART, str, strlen(str));
	//char * str2 = " Left Board_Print ";
	//return count + Chip_UART_SendBlocking(LPC_USART, str2, strlen(str2));
	return count;
}

uint32_t Board_Println(const char *str) {
	uint32_t count = Board_Print(str);
	return count + Board_Print("\r\n");
}

uint32_t Board_Println_Int(uint64_t n, uint32_t base) {
	const uint8_t max_digits = 100;
	char n_str[max_digits];
	itoa(n, n_str, base);
	return Board_Println(n_str);
}
