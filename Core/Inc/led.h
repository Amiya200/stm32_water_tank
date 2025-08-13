#ifndef LED_H
#define LED_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

void LED_Init(void);
void LED_Status_On(void);
void LED_Status_Off(void);
void LED_Status_Toggle(void);

void LED_Func_On(void);
void LED_Func_Off(void);
void LED_Func_Blink(uint32_t times, uint32_t delay_ms);

#endif /* LED_H */
