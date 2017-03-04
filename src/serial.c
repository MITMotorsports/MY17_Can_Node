#include "serial.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "chip.h"

uint32_t Serial_Print(const char *str) {
  return Chip_UART_SendBlocking(LPC_USART, str, strlen(str));
}

uint32_t Serial_Println(const char *str) {
  uint32_t count = Serial_Print(str);
  return count + Serial_Print("\r\n");
}

uint32_t Serial_PrintNumber(uint32_t n, uint32_t base) {
  const uint8_t max_digits = 33;
  char n_str[max_digits];
  itoa(n, n_str, base);
  return Serial_Print(n_str);
}

uint32_t Serial_PrintlnNumber(uint32_t n, uint32_t base) {
  uint32_t count = Serial_PrintNumber(n, base);
  return count + Serial_Print("\r\n");
}
