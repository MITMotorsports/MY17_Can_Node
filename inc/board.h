#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

uint32_t Board_Print(const char *str);

uint32_t Board_Println(const char *str);

uint32_t Board_Println_Int(uint64_t n, uint32_t base);

#endif //BOARD_H
