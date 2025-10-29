#ifndef __ACS712_H__
#define __ACS712_H__

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 *  ACS712-30A  +  Voltage Divider (1.8 k / 10 k + Diode)
 *  MCU       : STM32F1 Series (3.3 V ADC)
 * ============================================================
 */

/* -------------------------------
 *  Global values (for display)
 * ------------------------------- */
extern float g_currentA;   // Amperes
extern float g_voltageV;   // Volts

/* -------------------------------
 *  ACS712-30A Current Sensor
 * ------------------------------- */
#define ACS712_ADC_CHANNEL        ADC_CHANNEL_7
#define ACS712_NUM_SAMPLES        10
#define ACS712_SENSITIVITY_RAW    0.066f            // 66 mV per Amp
#define ACS712_VREF_ADC           3.3f
#define ACS712_ADC_RESOLUTION     4095.0f

// response & filter tuning
#define ACS712_FILTER_ALPHA       0.3f              // 0.1 = very smooth, 0.6 = fast
#define ACS712_NOISE_DEADZONE     0.03f             // ±30 mA ignore region

/* -------------------------------
 *  Voltage Measurement Divider
 * -------------------------------
 *  Rtop = 1.8 kΩ, Rbottom = 10 kΩ
 *  Divider ratio = 10 / (1.8 + 10) = 0.847
 */
#define VOLTAGE_ADC_CHANNEL       ADC_CHANNEL_6
#define VOLT_DIVIDER_RATIO        (10.0f / (10.0f + 1.8f))   // ≈ 0.847 f

/* -------------------------------
 *  Function Prototypes
 * ------------------------------- */
void  ACS712_Init(ADC_HandleTypeDef *hadc);
void  ACS712_CalibrateZero(void);
float ACS712_ReadCurrent(void);
float Voltage_ReadInput(void);
void  ACS712_Update(void);

#endif /* __ACS712_H__ */
