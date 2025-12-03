#ifndef EEPROM_I2C_H
#define EEPROM_I2C_H

#include "stm32f1xx_hal.h"
#define EE_ADDR_GAP_TIME        0x00   // uint16
#define EE_ADDR_RETRY_COUNT     0x02   // uint8
#define EE_ADDR_UV_LIMIT        0x03   // uint16
#define EE_ADDR_OV_LIMIT        0x05   // uint16
#define EE_ADDR_OVERLOAD        0x07   // float (4 bytes)
#define EE_ADDR_UNDERLOAD       0x0B   // float (4 bytes)
#define EE_ADDR_MAXRUN          0x0F   // uint16
#define EE_ADDR_SIGNATURE       0x20   // uint16 (EEPROM valid marker)
#define SETTINGS_SIGNATURE      0x55AA

HAL_StatusTypeDef EEPROM_WriteByte(uint16_t addr, uint8_t data);
HAL_StatusTypeDef EEPROM_ReadByte(uint16_t addr, uint8_t *data);
HAL_StatusTypeDef EEPROM_WriteBuffer(uint16_t addr, uint8_t *buf, uint16_t len);
HAL_StatusTypeDef EEPROM_ReadBuffer(uint16_t addr, uint8_t *buf, uint16_t len);

#endif
