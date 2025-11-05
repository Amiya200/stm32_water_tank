#include "rtc_i2c.h"
#include <stdio.h>
#include <string.h>

/* =====================================================
   EEPROM (AT24C32 / 24C02) INTEGRATION ON RTC MODULE
   Uses same I2C2 bus, address 0x57
   ===================================================== */

#define RTC_EEPROM_ADDR       (0x57u << 1)  // typical 24C32 EEPROM on DS3231 board
#define RTC_EEPROM_PAGE_SIZE  32
#define RTC_EEPROM_TOTAL_SIZE 4096          // 4K bytes typical
#define RTC_EEPROM_WRITE_DELAY_MS 10

/* =====================================================
   EEPROM BASIC READ/WRITE
   ===================================================== */

bool RTC_EEPROM_Write(uint16_t memAddr, const uint8_t *data, uint16_t len)
{
    if (memAddr + len > RTC_EEPROM_TOTAL_SIZE) return false;

    while (len > 0)
    {
        uint16_t chunk = RTC_EEPROM_PAGE_SIZE - (memAddr % RTC_EEPROM_PAGE_SIZE);
        if (chunk > len) chunk = len;

        if (HAL_I2C_Mem_Write(&hi2c2, RTC_EEPROM_ADDR,
                              memAddr, I2C_MEMADD_SIZE_16BIT,
                              (uint8_t *)data, chunk, HAL_MAX_DELAY) != HAL_OK)
        {
            return false;
        }

        HAL_Delay(RTC_EEPROM_WRITE_DELAY_MS);
        memAddr += chunk;
        data += chunk;
        len -= chunk;
    }

    return true;
}

bool RTC_EEPROM_Read(uint16_t memAddr, uint8_t *data, uint16_t len)
{
    if (memAddr + len > RTC_EEPROM_TOTAL_SIZE) return false;

    if (HAL_I2C_Mem_Read(&hi2c2, RTC_EEPROM_ADDR,
                         memAddr, I2C_MEMADD_SIZE_16BIT,
                         data, len, HAL_MAX_DELAY) != HAL_OK)
    {
        return false;
    }

    return true;
}

/* =====================================================
   CONTROLLER STATE SAVE / LOAD
   ===================================================== */

/* Simple CRC16 (X25) */
static uint16_t rtc_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

/* Save structure to EEPROM at fixed address */
bool RTC_SavePersistentState(const RTC_PersistState *s)
{
    RTC_PersistState tmp;
    memcpy(&tmp, s, sizeof(tmp));
    tmp.crc = rtc_crc16((uint8_t *)&tmp, sizeof(tmp) - 2);
    return RTC_EEPROM_Write(0x0100, (uint8_t *)&tmp, sizeof(tmp));
}

/* Read structure back from EEPROM */
bool RTC_LoadPersistentState(RTC_PersistState *s)
{
    if (!RTC_EEPROM_Read(0x0100, (uint8_t *)s, sizeof(*s)))
        return false;
    uint16_t c = rtc_crc16((uint8_t *)s, sizeof(*s) - 2);
    return (c == s->crc);
}

/* =====================================================
   DS3231 CORE CLOCK ROUTINES
   ===================================================== */

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
       02: hours (24h: bit6=0)
       03: day-of-week (1..7)
       04: day-of-month
       05: month (bit7=century; keep 0)
       06: year (00..99)
    */
    uint8_t buf[7];
    buf[0] = dec2bcd(sec)  & 0x7Fu;
    buf[1] = dec2bcd(min)  & 0x7Fu;
    buf[2] = dec2bcd(hour) & 0x3Fu;
    buf[3] = dec2bcd(dow)  & 0x07u;
    buf[4] = dec2bcd(dom)  & 0x3Fu;
    buf[5] = dec2bcd(month)& 0x1Fu;
    buf[6] = dec2bcd((uint8_t)(year - 2000));

    (void)HAL_I2C_Mem_Write(&hi2c2, s_rtc_addr, 0x00, 1, buf, 7, HAL_MAX_DELAY);
}

void RTC_SetTimeDate_AutoDOW(uint8_t sec, uint8_t min, uint8_t hour,
                             uint8_t dom, uint8_t month, uint16_t year)
{
    uint8_t dow = dow_from_ymd(year, month, dom);
    RTC_SetTimeDate(sec, min, hour, dow, dom, month, year);
}

/* ---------- read time/date ---------- */
void RTC_GetTimeDate(void)
{
    if (!s_rtc_addr) return;

    uint8_t r[7];
    if (HAL_I2C_Mem_Read(&hi2c2, s_rtc_addr, 0x00, 1, r, 7, HAL_MAX_DELAY) != HAL_OK)
        return;

    /* Seconds / Minutes */
    time.seconds = bcd2dec(r[0] & 0x7Fu);
    time.minutes = bcd2dec(r[1] & 0x7Fu);

    /* Hours: handle 12h or 24h formats robustly */
    if (r[2] & 0x40u) {
        uint8_t hr12 = bcd2dec(r[2] & 0x1Fu);
        uint8_t pm   = (r[2] & 0x20u) ? 1u : 0u;
        if (hr12 == 12)
            time.hour = pm ? 12 : 0;
        else
            time.hour = pm ? (hr12 + 12) : hr12;
    } else {
        time.hour = bcd2dec(r[2] & 0x3Fu);
    }

    /* DOW / Date / Month / Year */
    time.dayofweek  = bcd2dec(r[3] & 0x07u);
    time.dayofmonth = bcd2dec(r[4] & 0x3Fu);
    time.month      = bcd2dec(r[5] & 0x1Fu);
    time.year       = 2000u + bcd2dec(r[6]);
}

/* ---------- debug dump (first 7 registers) ---------- */
void RTC_DumpRegisters(void)
{
    if (!s_rtc_addr) return;
    uint8_t r[7];
    if (HAL_I2C_Mem_Read(&hi2c2, s_rtc_addr, 0x00, 1, r, 7, HAL_MAX_DELAY) != HAL_OK)
        return;

//    printf("RTC regs: ");
    for (int i = 0; i < 7; i++) printf("%02X ", r[i]);
//    printf("\r\n");
}
