#ifndef __RTC_I2C_H
#define __RTC_I2C_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h" // For I2C_HandleTypeDef and HAL functions, which includes stm32f1xx_hal_rtc.h
#include <stdbool.h>
#include <stdint.h>

// Define the I2C address of your RTC module (if using an external I2C RTC)
// For internal STM32 RTC, this define is not directly used by HAL RTC functions.
// It's kept here for consistency if you later switch to an external I2C RTC.
#define RTC_I2C_ADDRESS (0x68 << 1) // Example for DS1307/DS3231

// RTC Register Addresses (Example for DS1307/DS3231 - not directly used for internal HAL RTC)
#define DS1307_REG_SECONDS      0x00
#define DS1307_REG_MINUTES      0x01
#define DS1307_REG_HOURS        0x02
#define DS1307_REG_DAYOFWEEK    0x03
#define DS1307_REG_DAY          0x04
#define DS1307_REG_MONTH        0x05
#define DS1307_REG_YEAR         0x06
#define DS1307_REG_CONTROL      0x07

// Control Register Bits (Example for DS1307 - not directly used for internal HAL RTC)
#define DS1307_CH_BIT           (1 << 7) // Clock Halt bit
#define DS1307_SQWE_BIT         (1 << 4) // Square Wave Enable bit
#define DS1307_RS_MASK          (0x03)   // Rate Select bits

// NOTE: RTC_TimeTypeDef and RTC_DateTypeDef are already defined by the STM32 HAL library
// in stm32f1xx_hal_rtc.h, which is included via main.h -> stm32f1xx_hal.h.
// Do NOT redefine them here, as it causes conflicting types errors.

// Function Prototypes

/**
  * @brief  Initializes the RTC module. For internal RTC, this typically means
  *         initializing the HAL_RTC_HandleTypeDef. For external I2C RTC, it
  *         would involve I2C initialization and checking the RTC chip.
  * @param  hrtc: Pointer to the RTC handle (e.g., &hrtc from main.c)
  * @retval true if initialization is successful, false otherwise.
  */
bool RTC_I2C_Init(RTC_HandleTypeDef *hrtc); // Changed parameter to RTC_HandleTypeDef

/**
  * @brief  Sets the current time on the RTC module.
  * @param  hrtc: Pointer to the RTC handle.
  * @param  sTime: Pointer to a RTC_TimeTypeDef structure containing the time to set.
  * @retval true if time is set successfully, false otherwise.
  */
bool RTC_I2C_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime);

/**
  * @brief  Gets the current time from the RTC module.
  * @param  hrtc: Pointer to the RTC handle.
  * @param  sTime: Pointer to a RTC_TimeTypeDef structure to store the retrieved time.
  * @retval true if time is read successfully, false otherwise.
  */
bool RTC_I2C_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime);

/**
  * @brief  Sets the current date on the RTC module.
  * @param  hrtc: Pointer to the RTC handle.
  * @param  sDate: Pointer to a RTC_DateTypeDef structure containing the date to set.
  * @retval true if date is set successfully, false otherwise.
  */
bool RTC_I2C_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate); // Changed to RTC_DateTypeDef

/**
  * @brief  Gets the current date from the RTC module.
  * @param  hrtc: Pointer to the RTC handle.
  * @param  sDate: Pointer to a RTC_DateTypeDef structure to store the retrieved date.
  * @retval true if date is read successfully, false otherwise.
  */
bool RTC_I2C_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate); // Changed to RTC_DateTypeDef

/**
  * @brief  Converts a decimal value to BCD format.
  * @param  val: Decimal value to convert.
  * @retval BCD value.
  */
uint8_t RTC_I2C_DecToBcd(uint8_t val);

/**
  * @brief  Converts a BCD value to decimal format.
  * @param  val: BCD value to convert.
  * @retval Decimal value.
  */
uint8_t RTC_I2C_BcdToDec(uint8_t val);

#ifdef __cplusplus
}
#endif

#endif /* __RTC_I2C_H */
