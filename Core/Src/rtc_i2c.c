#include "rtc_i2c.h"
#include <stdio.h>

/* Probe both popular addresses:
   - DS3231 genuine: 7-bit 0x68  -> HAL 8-bit = 0xD0
   - Your module reported: 0x57   -> HAL 8-bit = 0xAE
*/
#define DS3231_ADDR_68   (0x68u << 1)
#define DS3231_ADDR_57   (0x57u << 1)

/* Global time object */
RTC_Time_t time;

/* Selected I2C address (HAL 8-bit) after probe */
static uint16_t s_rtc_addr = 0;

/* ---------- helpers ---------- */
static uint8_t dec2bcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }
static uint8_t bcd2dec(uint8_t v) { return (uint8_t)(((v >> 4) * 10) + (v & 0x0F)); }

/* Zeller/Sakamoto-style weekday (Mon=1 … Sun=7) */
static uint8_t dow_from_ymd(uint16_t y, uint8_t m, uint8_t d)
{
    static const uint8_t t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
    if (m < 3) y--;
    /* 0=Sunday … 6=Saturday */
    uint8_t w = (uint8_t)((y + y/4 - y/100 + y/400 + t[m-1] + d) % 7);
    /* Map to Mon=1 … Sun=7 */
    return (w == 0) ? 7 : w;          /* 7=Sunday */
}

/* ---------- probe + init ---------- */
void RTC_Init(void)
{
    /* Probe 0x68 first, then 0x57 */
    if (HAL_I2C_IsDeviceReady(&hi2c2, DS3231_ADDR_68, 2, 50) == HAL_OK) {
        s_rtc_addr = DS3231_ADDR_68;
    } else if (HAL_I2C_IsDeviceReady(&hi2c2, DS3231_ADDR_57, 2, 50) == HAL_OK) {
        s_rtc_addr = DS3231_ADDR_57;
    } else {
        s_rtc_addr = 0; /* not found */
        return;
    }

    /* Clear CH (clock halt) bit if set (seconds @ reg 0x00, bit7) */
    uint8_t sec;
    if (HAL_I2C_Mem_Read(&hi2c2, s_rtc_addr, 0x00, 1, &sec, 1, HAL_MAX_DELAY) == HAL_OK) {
        if (sec & 0x80u) {
            sec &= 0x7Fu;
            (void)HAL_I2C_Mem_Write(&hi2c2, s_rtc_addr, 0x00, 1, &sec, 1, HAL_MAX_DELAY);
        }
    }
}

/* ---------- set time/date ---------- */
void RTC_SetTimeDate(uint8_t sec, uint8_t min, uint8_t hour,
                     uint8_t dow, uint8_t dom, uint8_t month, uint16_t year)
{
    if (!s_rtc_addr) return;

    /* Force valid ranges (defensive) */
    if (sec  > 59) sec = 0;
    if (min  > 59) min = 0;
    if (hour > 23) hour = 0;
    if (dow   < 1 || dow > 7)    dow = 1;
    if (dom   < 1 || dom > 31)   dom = 1;
    if (month < 1 || month > 12) month = 1;
    if (year  < 2000) year = 2000; else if (year > 2099) year = 2099;

    /* Write 7 bytes starting at 0x00:
       00: seconds (bit7 = CH, must be 0 to run)
       01: minutes
       02: hours (24h: bit6=0, bits5:4 tens of hours)
       03: day-of-week (1..7)
       04: day-of-month
       05: month (bit7=century; keep 0)
       06: year (00..99)
    */
    uint8_t buf[7];
    buf[0] = dec2bcd(sec)  & 0x7Fu;  /* ensure CH=0 */
    buf[1] = dec2bcd(min)  & 0x7Fu;
    buf[2] = dec2bcd(hour) & 0x3Fu;  /* force 24h: bit6=0 */
    buf[3] = dec2bcd(dow)  & 0x07u;
    buf[4] = dec2bcd(dom)  & 0x3Fu;
    buf[5] = dec2bcd(month)& 0x1Fu;  /* century bit=0 */
    buf[6] = dec2bcd((uint8_t)(year - 2000));

    (void)HAL_I2C_Mem_Write(&hi2c2, s_rtc_addr, 0x00, 1, buf, 7, HAL_MAX_DELAY);
}

void RTC_SetTimeDate_AutoDOW(uint8_t sec, uint8_t min, uint8_t hour,
                             uint8_t dom, uint8_t month, uint16_t year)
{
    uint8_t dow = dow_from_ymd(year, month, dom); /* 1..7 */
    RTC_SetTimeDate(sec, min, hour, dow, dom, month, year);
}

/* ---------- read time/date ---------- */
void RTC_GetTimeDate(void)
{
    if (!s_rtc_addr) return;

    uint8_t r[7];
    if (HAL_I2C_Mem_Read(&hi2c2, s_rtc_addr, 0x00, 1, r, 7, HAL_MAX_DELAY) != HAL_OK) {
        return;
    }

    /* Seconds / Minutes */
    time.seconds = bcd2dec(r[0] & 0x7Fu);
    time.minutes = bcd2dec(r[1] & 0x7Fu);

    /* Hours: handle 12h or 24h formats robustly */
    if (r[2] & 0x40u) {
        /* 12-hour mode: bit5 = AM/PM (1=PM), bits4..0 = 1..12 */
        uint8_t hr12 = bcd2dec(r[2] & 0x1Fu);
        uint8_t pm   = (r[2] & 0x20u) ? 1u : 0u;
        if (hr12 == 12) {
            time.hour = pm ? 12 : 0;      /* 12AM -> 0, 12PM -> 12 */
        } else {
            time.hour = pm ? (hr12 + 12) : hr12;
        }
    } else {
        /* 24-hour mode: bits5..0 are BCD hour */
        time.hour = bcd2dec(r[2] & 0x3Fu);
    }

    /* DOW / Date / Month / Year */
    time.dayofweek  = bcd2dec(r[3] & 0x07u);              /* 1..7 */
    time.dayofmonth = bcd2dec(r[4] & 0x3Fu);              /* 1..31 */
    time.month      = bcd2dec(r[5] & 0x1Fu);              /* 1..12 (ignore century) */
    time.year       = 2000u + bcd2dec(r[6]);              /* 2000..2099 */
}

/* ---------- debug dump (first 7 registers) ---------- */
void RTC_DumpRegisters(void)
{
    if (!s_rtc_addr) return;
    uint8_t r[7];
    if (HAL_I2C_Mem_Read(&hi2c2, s_rtc_addr, 0x00, 1, r, 7, HAL_MAX_DELAY) != HAL_OK) return;

    printf("RTC regs: ");
    for (int i = 0; i < 7; i++) printf("%02X ", r[i]);
    printf("\r\n");
}
