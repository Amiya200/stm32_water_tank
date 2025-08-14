#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdint.h>
#include "stm32f1xx_hal.h"
extern UART_HandleTypeDef huart1;

// Declare motorStatus as an extern variable
extern uint8_t motorStatus; // 0: Off, 1: On

#endif // GLOBAL_H
