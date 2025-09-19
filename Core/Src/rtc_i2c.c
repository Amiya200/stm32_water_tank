#include "rtc_i2c.h"
#include "main.h"
#include <stdio.h>

extern RTC_HandleTypeDef hrtc;
TIME time;

extern void Debug_Print(char *msg);

/* ------------------ RTC Init with Backup ------------------ */
void RTC_InitWithBackup(void)
{
    HAL_PWR_EnableBkUpAccess();

    __HAL_RCC_LSI_ENABLE();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET);

    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    __HAL_RCC_RTC_ENABLE();

    hrtc.Instance = RTC;
    hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
    hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;

    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
        Debug_Print("RTC Init failed\r\n");
        return;
    }

    if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == 0x32F2)
        Debug_Print("RTC already initialized\r\n");
    else
        Debug_Print("RTC not initialized. Please set time/date now.\r\n");
}

/* ------------------ Set Time + Date ------------------ */
void RTC_SetTimeDate(uint8_t sec, uint8_t min, uint8_t hour,
                     uint8_t dow, uint8_t dom, uint8_t month, uint16_t year)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours   = hour;
    sTime.Minutes = min;
    sTime.Seconds = sec;
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    sDate.WeekDay = dow;
    sDate.Date    = dom;
    sDate.Month   = month;
    sDate.Year    = (uint8_t)(year - 2000);
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);

    Debug_Print("RTC time/date set successfully\r\n");
}

/* ------------------ Get Time + Date ------------------ */
void RTC_GetTimeDate(void)
{
    RTC_TimeTypeDef gTime = {0};
    RTC_DateTypeDef gDate = {0};

    HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

    time.seconds    = gTime.Seconds;
    time.minutes    = gTime.Minutes;
    time.hour       = gTime.Hours;
    time.dayofweek  = gDate.WeekDay;
    time.dayofmonth = gDate.Date;
    time.month      = gDate.Month;
    time.year       = 2000 + gDate.Year;
}
