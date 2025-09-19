#ifndef __RTC_I2C_H
#define __RTC_I2C_H

#include "stm32f1xx_hal.h"

typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hour;
    uint8_t dayofweek;
    uint8_t dayofmonth;
    uint8_t month;
    uint16_t year;
} TIME;

extern TIME time;

void RTC_InitWithBackup(void);
void RTC_SetTimeDate(uint8_t sec, uint8_t min, uint8_t hour,
                     uint8_t dow, uint8_t dom, uint8_t month, uint16_t year);
void RTC_GetTimeDate(void);

#endif
