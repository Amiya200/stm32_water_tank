#include "rtc_i2c.h"

// External RTC_HandleTypeDef declaration from main.c (or wherever it's defined)
extern RTC_HandleTypeDef hrtc;

/**
  * @brief  Converts a decimal value to BCD format.
  * @param  val: Decimal value to convert.
  * @retval BCD value.
  */
uint8_t RTC_I2C_DecToBcd(uint8_t val) {
    return (uint8_t)((val / 10 * 16) + (val % 10));
}

/**
  * @brief  Converts a BCD value to decimal format.
  * @param  val: BCD value to convert.
  * @retval Decimal value.
  */
uint8_t RTC_I2C_BcdToDec(uint8_t val) {
    return (uint8_t)((val / 16 * 10) + (val % 16));
}

/**
  * @brief  Initializes the RTC module. This function now wraps the HAL RTC Init.
  * @param  hrtc: Pointer to the RTC handle.
  * @retval true if initialization is successful, false otherwise.
  */
bool RTC_I2C_Init(RTC_HandleTypeDef *hrtc) {
    // The actual RTC peripheral initialization (clock, prescalers) is typically
    // done in MX_RTC_Init in main.c. This function can be used to check status
    // or perform initial configuration if needed for an external RTC.
    // For internal RTC, this function might just return true if HAL_RTC_Init was successful.
    // Since MX_RTC_Init handles the HAL_RTC_Init, this function might be redundant
    // if only using the internal RTC. However, it's kept for API consistency.
    if (HAL_RTC_GetState(hrtc) == HAL_RTC_STATE_READY) {
        return true;
    }
    return false; // RTC not ready, likely not initialized by MX_RTC_Init
}

/**
  * @brief  Sets the current time on the RTC module using HAL functions.
  * @param  hrtc: Pointer to the RTC handle.
  * @param  sTime: Pointer to a RTC_TimeTypeDef structure containing the time to set.
  * @retval true if time is set successfully, false otherwise.
  */
bool RTC_I2C_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime) {
    // For STM32 internal RTC, we typically use RTC_FORMAT_BIN for setting time
    // and the HAL driver handles the conversion to BCD for the RTC registers.
    if (HAL_RTC_SetTime(hrtc, sTime, RTC_FORMAT_BIN) != HAL_OK) {
        return false;
    }
    return true;
}

/**
  * @brief  Gets the current time from the RTC module using HAL functions.
  * @param  hrtc: Pointer to the RTC handle.
  * @param  sTime: Pointer to a RTC_TimeTypeDef structure to store the retrieved time.
  * @retval true if time is read successfully, false otherwise.
  */
bool RTC_I2C_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime) {
    // For STM32 internal RTC, we typically use RTC_FORMAT_BIN for getting time.
    if (HAL_RTC_GetTime(hrtc, sTime, RTC_FORMAT_BIN) != HAL_OK) {
        return false;
    }
    return true;
}

/**
  * @brief  Sets the current date on the RTC module using HAL functions.
  * @param  hrtc: Pointer to the RTC handle.
  * @param  sDate: Pointer to a RTC_DateTypeDef structure containing the date to set.
  * @retval true if date is set successfully, false otherwise.
  */
bool RTC_I2C_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate) {
    // For STM32 internal RTC, we typically use RTC_FORMAT_BIN for setting date.
    if (HAL_RTC_SetDate(hrtc, sDate, RTC_FORMAT_BIN) != HAL_OK) {
        return false;
    }
    return true;
}

/**
  * @brief  Gets the current date from the RTC module using HAL functions.
  * @param  hrtc: Pointer to the RTC handle.
  * @param  sDate: Pointer to a RTC_DateTypeDef structure to store the retrieved date.
  * @retval true if date is read successfully, false otherwise.
  */
bool RTC_I2C_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate) {
    // For STM32 internal RTC, we typically use RTC_FORMAT_BIN for getting date.
    if (HAL_RTC_GetDate(hrtc, sDate, RTC_FORMAT_BIN) != HAL_OK) {
        return false;
    }
    return true;
}
