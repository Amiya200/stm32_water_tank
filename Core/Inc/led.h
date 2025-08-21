#ifndef __LED_H
#define __LED_H

#include "stm32f1xx_hal.h"

/* Initialize LEDs */
void LED_Init(void);

/* Basic Controls */
void LED_On(uint8_t led);
void LED_Off(uint8_t led);
void LED_Toggle(uint8_t led);
void LED_All_On(void);
void LED_All_Off(void);

/* Patterns */
void LED_Blink_All(uint32_t times, uint32_t delay_ms);
void LED_Running(uint32_t delay_ms);
void LED_KnightRider(uint32_t delay_ms, uint32_t cycles);
void LED_Alternate(uint32_t times, uint32_t delay_ms);
void LED_Wave(uint32_t delay_ms);

#endif
