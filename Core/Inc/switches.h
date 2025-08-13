#ifndef SWITCHES_H
#define SWITCHES_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

void Switches_Init(void);
bool Switch_Read(uint8_t idx); /* idx = 1..4 */
bool Switch_WasPressed(uint8_t idx); /* returns true once when pressed (debounced) */

#endif /* SWITCHES_H */
