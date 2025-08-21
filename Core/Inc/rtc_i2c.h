#ifndef __RTC_I2C_H
#define __RTC_I2C_H

#include "main.h" // Assuming main.h includes HAL definitions and other necessary headers

// DS3231 I2C Slave Address (7-bit = 0x68, shifted left by 1 = 0xD0 for HAL)
#define DS3231_ADDRESS (0x68 << 1)

// Structure to hold time and date information
typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hour;
    uint8_t dayofweek;   // 1=Sunday, 2=Monday, ..., 7=Saturday
    uint8_t dayofmonth;
    uint8_t month;
    uint8_t year;        // Last two digits of the year (e.g., 23 for 2023)
} TIME;

extern TIME time; // Global TIME structure

// Function prototypes
uint8_t decToBcd(int val);
int bcdToDec(uint8_t val);
void Set_Time (uint8_t sec, uint8_t min, uint8_t hour,
               uint8_t dow, uint8_t dom, uint8_t month, uint8_t year);
void Get_Time (void);
float Get_Temp (void);
void force_temp_conv (void);

#endif /* __RTC_I2C_H */
