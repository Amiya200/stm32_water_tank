#ifndef __ACS712_H__
#define __ACS712_H__

#include "stm32f1xx_hal.h"
#include <stdint.h>

extern float g_currentA;
extern float g_voltageV;

/* -----------------------------------------
   ADC CONFIG
------------------------------------------ */
#define ADC_VREF   3.3f
#define ADC_RES    4095.0f

/* -----------------------------------------
   ACS712 CURRENT SENSOR
------------------------------------------ */
#define ACS712_ADC_CHANNEL     ADC_CHANNEL_7
#define ACS712_ZERO_SAMPLES    300
#define ACS712_FILTER_ALPHA    0.05f
#define ACS712_SENS_30A        0.066f      // 30A version

/* -----------------------------------------
   ZMPT101B VOLTAGE SENSOR
------------------------------------------ */
#define ZMPT_ADC_CHANNEL       ADC_CHANNEL_6
#define ZMPT_OFFSET_SAMPLES    300
#define ZMPT_RMS_SAMPLES       800
#define ZMPT_FILTER_ALPHA      0.12f

/* --------------------------------------------------
   CALIBRATION FACTOR
   Change this value so that:
   g_voltageV == Your Multimeter Voltage

   NEW_FACTOR = Multimeter_Voltage / ADC_RMS
--------------------------------------------------- */
#define ZMPT_CALIBRATION       245.0f      // <-- CHANGE THIS ONLY

void ACS712_Init(ADC_HandleTypeDef *hadc);
void ACS712_Update(void);

float ACS712_ReadCurrent(void);
float ZMPT_ReadVoltageRMS(void);

#endif
