#include "eeprom_i2c.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c2;

#define EEPROM_ADDR  (0x50 << 1)  // adjust for A0/A1/A2 pins

HAL_StatusTypeDef EEPROM_WriteByte(uint16_t memAddr, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c2, EEPROM_ADDR,
                             memAddr, I2C_MEMADD_SIZE_16BIT,
                             &data, 1, 10);
}

HAL_StatusTypeDef EEPROM_ReadByte(uint16_t memAddr, uint8_t* data)
{
    return HAL_I2C_Mem_Read(&hi2c2, EEPROM_ADDR,
                            memAddr, I2C_MEMADD_SIZE_16BIT,
                            data, 1, 10);
}

HAL_StatusTypeDef EEPROM_WriteBuffer(uint16_t memAddr, uint8_t* buf, uint16_t len)
{
    return HAL_I2C_Mem_Write(&hi2c2, EEPROM_ADDR,
                             memAddr, I2C_MEMADD_SIZE_16BIT,
                             buf, len, 50);
}

HAL_StatusTypeDef EEPROM_ReadBuffer(uint16_t memAddr, uint8_t* buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c2, EEPROM_ADDR,
                            memAddr, I2C_MEMADD_SIZE_16BIT,
                            buf, len, 50);
}
