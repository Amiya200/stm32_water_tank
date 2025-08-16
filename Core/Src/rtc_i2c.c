#include "rtc_i2c.h"

extern I2C_HandleTypeDef hi2c2; // External declaration for the I2C handle initialized in main.c

TIME time; // Definition of the global TIME structure (only defined once here)

/**
  * @brief Converts a normal decimal number to Binary Coded Decimal (BCD).
  * @param val: The decimal value to convert.
  * @retval The BCD representation of the value.
  */
uint8_t decToBcd(int val)
{
  return (uint8_t)( (val/10*16) + (val%10) );
}

/**
  * @brief Converts a Binary Coded Decimal (BCD) number to a normal decimal number.
  * @param val: The BCD value to convert.
  * @retval The decimal representation of the value.
  */
int bcdToDec(uint8_t val)
{
  return (int)( (val/16*10) + (val%16) );
}

/**
  * @brief Sets the time and date on the DS3231 RTC module.
  * @param sec: Seconds (0-59)
  * @param min: Minutes (0-59)
  * @param hour: Hour (0-23 for 24-hour format, or 1-12 for 12-hour format with AM/PM bit)
  * @param dow: Day of Week (1-7, e.g., 1 for Sunday)
  * @param dom: Day of Month (1-31)
  * @param month: Month (1-12)
  * @param year: Year (0-99, representing 2000-2099)
  * @retval None
  */
void Set_Time (uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year)
{
	uint8_t set_time[7];
	set_time[0] = decToBcd(sec);
	set_time[1] = decToBcd(min);
	set_time[2] = decToBcd(hour);
	set_time[3] = decToBcd(dow);
	set_time[4] = decToBcd(dom);
	set_time[5] = decToBcd(month);
	set_time[6] = decToBcd(year);

	// Write 7 bytes starting from address 0x00 (seconds register)
	HAL_I2C_Mem_Write(&hi2c2, DS3231_ADDRESS, 0x00, 1, set_time, 7, 1000);
}

/**
  * @brief Reads the current time and date from the DS3231 RTC module.
  *        The read values are stored in the global 'time' structure.
  * @param None
  * @retval None
  */
void Get_Time (void)
{
	uint8_t get_time[7];
	// Read 7 bytes starting from address 0x00 (seconds register)
	HAL_I2C_Mem_Read(&hi2c2, DS3231_ADDRESS, 0x00, 1, get_time, 7, 1000);
	time.seconds = bcdToDec(get_time[0]);
	time.minutes = bcdToDec(get_time[1]);
	time.hour = bcdToDec(get_time[2]);
	time.dayofweek = bcdToDec(get_time[3]);
	time.dayofmonth = bcdToDec(get_time[4]);
	time.month = bcdToDec(get_time[5]);
	time.year = bcdToDec(get_time[6]);
}

/**
  * @brief Reads the temperature from the DS3231 RTC module.
  * @param None
  * @retval The temperature in Celsius as a float.
  */
float Get_Temp (void)
{
	uint8_t temp[2];

	// Read 2 bytes starting from address 0x11 (temperature MSB register)
	HAL_I2C_Mem_Read(&hi2c2, DS3231_ADDRESS, 0x11, 1, temp, 2, 1000);
	// Temperature calculation: MSB + (LSB >> 6) * 0.25
	return ((float)temp[0]) + ((float)(temp[1] >> 6) / 4.0f);
}

/**
  * @brief Forces a temperature conversion on the DS3231 RTC module.
  *        This function checks the busy bit in the status register and
  *        initiates a conversion if not already in progress.
  * @param None
  * @retval None
  */
void force_temp_conv (void)
{
	uint8_t status=0;
	uint8_t control=0;
	// Read status register (0x0F)
	HAL_I2C_Mem_Read(&hi2c2, DS3231_ADDRESS, 0x0F, 1, &status, 1, 100);
	// Check if the busy bit (bit 2, OSF) is not set (0x04)
	if (!(status & 0x04))
	{
		// Read control register (0x0E)
		HAL_I2C_Mem_Read(&hi2c2, DS3231_ADDRESS, 0x0E, 1, &control, 1, 100);
		// Set the CONV bit (bit 5, CONV) in the control register to force a temperature conversion
		HAL_I2C_Mem_Write(&hi2c2, DS3231_ADDRESS, 0x0E, 1, (uint8_t *)(control | 0x20), 1, 100);
	}
}
