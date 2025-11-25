#ifndef EEPROM_I2C_H
#define EEPROM_I2C_H

#include "stm32f1xx_hal.h"

HAL_StatusTypeDef EEPROM_WriteByte(uint16_t addr, uint8_t data);
HAL_StatusTypeDef EEPROM_ReadByte(uint16_t addr, uint8_t *data);
HAL_StatusTypeDef EEPROM_WriteBuffer(uint16_t addr, uint8_t *buf, uint16_t len);
HAL_StatusTypeDef EEPROM_ReadBuffer(uint16_t addr, uint8_t *buf, uint16_t len);

#endif
