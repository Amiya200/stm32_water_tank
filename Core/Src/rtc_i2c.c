#include "rtc_i2c.h"
#include "main.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c2; // I2C handle from main.c

TIME time; // Global time struct

uint8_t decToBcd(int val) {
    return (uint8_t)((val / 10 * 16) + (val % 10));
}

int bcdToDec(uint8_t val) {
    return (int)((val / 16 * 10) + (val % 16));
}

void Set_Time(uint8_t sec, uint8_t min, uint8_t hour,
              uint8_t dow, uint8_t dom, uint8_t month, uint8_t year) {
    uint8_t set_time[7];
    set_time[0] = decToBcd(sec);
    set_time[1] = decToBcd(min);
    set_time[2] = decToBcd(hour);
    set_time[3] = decToBcd(dow);
    set_time[4] = decToBcd(dom);
    set_time[5] = decToBcd(month);
    set_time[6] = decToBcd(year);

    if (HAL_I2C_Mem_Write(&hi2c2, DS3231_ADDRESS, 0x00,
                          I2C_MEMADD_SIZE_8BIT, set_time, 7, 1000) != HAL_OK) {
        char err[50];
        sprintf(err, "RTC Set Error: %lu\r\n", HAL_I2C_GetError(&hi2c2));
        Debug_Print(err);
    }
}

void Get_Time(void) {
    uint8_t get_time[7];

    if (HAL_I2C_Mem_Read(&hi2c2, DS3231_ADDRESS, 0x00,
                         I2C_MEMADD_SIZE_8BIT, get_time, 7, 1000) != HAL_OK) {
        char err[50];
        sprintf(err, "RTC Read Error: %lu\r\n", HAL_I2C_GetError(&hi2c2));
        Debug_Print(err);

        time.seconds = time.minutes = time.hour =
        time.dayofweek = time.dayofmonth =
        time.month = time.year = 0xFF;
        return;
    }

    char dbg[60];
    sprintf(dbg, "Raw RTC Data: %02X %02X %02X %02X %02X %02X %02X\r\n",
            get_time[0], get_time[1], get_time[2],
            get_time[3], get_time[4], get_time[5], get_time[6]);
    Debug_Print(dbg);

    time.seconds    = bcdToDec(get_time[0]);
    time.minutes    = bcdToDec(get_time[1]);
    time.hour       = bcdToDec(get_time[2]);
    time.dayofweek  = bcdToDec(get_time[3]);
    time.dayofmonth = bcdToDec(get_time[4]);
    time.month      = bcdToDec(get_time[5]);
    time.year       = bcdToDec(get_time[6]);
}

float Get_Temp(void) {
    uint8_t temp[2];

    if (HAL_I2C_Mem_Read(&hi2c2, DS3231_ADDRESS, 0x11,
                         I2C_MEMADD_SIZE_8BIT, temp, 2, 1000) != HAL_OK) {
        char err[50];
        sprintf(err, "RTC Temp Read Error: %lu\r\n", HAL_I2C_GetError(&hi2c2));
        Debug_Print(err);
        return -999.0f;
    }

    return (float)temp[0] + ((temp[1] >> 6) * 0.25f);
}

void force_temp_conv(void) {
    uint8_t status = 0;
    uint8_t control = 0;

    if (HAL_I2C_Mem_Read(&hi2c2, DS3231_ADDRESS, 0x0F,
                         I2C_MEMADD_SIZE_8BIT, &status, 1, 100) != HAL_OK) {
        Debug_Print("RTC Status Read Fail\r\n");
        return;
    }

    if (!(status & 0x04)) {
        if (HAL_I2C_Mem_Read(&hi2c2, DS3231_ADDRESS, 0x0E,
                             I2C_MEMADD_SIZE_8BIT, &control, 1, 100) != HAL_OK) {
            Debug_Print("RTC Ctrl Read Fail\r\n");
            return;
        }

        control |= 0x20; // Set CONV bit

        if (HAL_I2C_Mem_Write(&hi2c2, DS3231_ADDRESS, 0x0E,
                              I2C_MEMADD_SIZE_8BIT, &control, 1, 100) != HAL_OK) {
            Debug_Print("RTC Temp Conv Fail\r\n");
        }
    }
}
