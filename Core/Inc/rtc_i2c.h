#ifndef __RTC_I2C_H
#define __RTC_I2C_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* External I2C handle from your project */
extern I2C_HandleTypeDef hi2c2;

/* Time structure exposed to the app */
typedef struct {
    uint8_t  seconds;     /* 0–59  */
    uint8_t  minutes;     /* 0–59  */
    uint8_t  hour;        /* 0–23  */
    uint8_t  dayofweek;   /* 1–7   (Mon=1 … Sun=7) */
    uint8_t  dayofmonth;  /* 1–31  */
    uint8_t  month;       /* 1–12  */
    uint16_t year;        /* 2000–2099 */
} RTC_Time_t;

extern RTC_Time_t time;

/* Init / probe / basic IO */
void RTC_Init(void);
void RTC_GetTimeDate(void);

/* Set time/date (pass DOW yourself) */
void RTC_SetTimeDate(uint8_t sec, uint8_t min, uint8_t hour,
                     uint8_t dow, uint8_t dom, uint8_t month, uint16_t year);

/* Set time/date (auto-computes DOW = Monday(1) … Sunday(7)) */
void RTC_SetTimeDate_AutoDOW(uint8_t sec, uint8_t min, uint8_t hour,
                             uint8_t dom, uint8_t month, uint16_t year);

/* Optional: dump first 7 registers for debug */
void RTC_DumpRegisters(void);

#endif /* __RTC_I2C_H */
