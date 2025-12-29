#include "eeprom_i2c.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c2;

/* ============================================================
   EEPROM CONFIG
   ============================================================ */

#define EEPROM_I2C_ADDR       0xA0
#define EEPROM_TIMEOUT       100


/* ============================================================
   LOW LEVEL EEPROM DRIVER (UNCHANGED API)
   ============================================================ */

HAL_StatusTypeDef EEPROM_WriteByte(uint16_t addr, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c2, EEPROM_I2C_ADDR, addr,
                             I2C_MEMADD_SIZE_16BIT, &data, 1, EEPROM_TIMEOUT);
}

HAL_StatusTypeDef EEPROM_ReadByte(uint16_t addr, uint8_t *data)
{
    return HAL_I2C_Mem_Read(&hi2c2, EEPROM_I2C_ADDR, addr,
                            I2C_MEMADD_SIZE_16BIT, data, 1, EEPROM_TIMEOUT);
}

HAL_StatusTypeDef EEPROM_WriteBuffer(uint16_t addr, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Write(&hi2c2, EEPROM_I2C_ADDR, addr,
                             I2C_MEMADD_SIZE_16BIT, buf, len, EEPROM_TIMEOUT);
}

HAL_StatusTypeDef EEPROM_ReadBuffer(uint16_t addr, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c2, EEPROM_I2C_ADDR, addr,
                            I2C_MEMADD_SIZE_16BIT, buf, len, EEPROM_TIMEOUT);
}


/* ============================================================
   SAFE MODE STORAGE (NEW, POWER-FAIL SAFE)
   ============================================================ */

static uint8_t ee_crc(uint8_t m, uint8_t mot)
{
    return (m ^ mot ^ EE_COMMIT_FLAG);
}

void EEPROM_SaveMode(uint8_t mode, uint8_t motor)
{
    uint8_t crc = ee_crc(mode, motor);
    uint8_t zero = 0;

    /* Invalidate */
    EEPROM_WriteByte(EE_MODE_BLOCK_ADDR, zero);

    EEPROM_WriteByte(EE_MODE_BLOCK_ADDR + 1, mode);
    EEPROM_WriteByte(EE_MODE_BLOCK_ADDR + 2, motor);
    EEPROM_WriteByte(EE_MODE_BLOCK_ADDR + 3, crc);

    EEPROM_WriteByte(EE_MODE_BLOCK_ADDR + 4, EE_COMMIT_FLAG);
    EEPROM_WriteByte(EE_MODE_BLOCK_ADDR, EE_VALID_FLAG);
}

uint8_t EEPROM_LoadMode(uint8_t *mode, uint8_t *motor)
{
    uint8_t v, c, m, mot, crc;

    EEPROM_ReadByte(EE_MODE_BLOCK_ADDR, &v);
    EEPROM_ReadByte(EE_MODE_BLOCK_ADDR + 4, &c);

    if(v != EE_VALID_FLAG || c != EE_COMMIT_FLAG)
        return 0;

    EEPROM_ReadByte(EE_MODE_BLOCK_ADDR + 1, &m);
    EEPROM_ReadByte(EE_MODE_BLOCK_ADDR + 2, &mot);
    EEPROM_ReadByte(EE_MODE_BLOCK_ADDR + 3, &crc);

    if(crc != ee_crc(m, mot))
        return 0;

    *mode = m;
    *motor = mot;
    return 1;
}
