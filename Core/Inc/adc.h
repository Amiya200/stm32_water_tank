#ifndef ADC_H
#define ADC_H

#include "stm32f1xx_hal.h"

// Define ADC channels
#define ADC_CHANNEL_COUNT 6
#define ADC_MAX_CHANNELS 6

typedef enum {
    ADC_CH0 = 0,  // PA0
    ADC_CH1,      // PA1
    ADC_CH2,      // PA2
    ADC_CH3,      // PA3
    ADC_CH4,      // PA4
    ADC_CH5,      // PA5
} ADC_Channel;

typedef struct {
    uint32_t rawValues[ADC_MAX_CHANNELS];
    float voltages[ADC_MAX_CHANNELS];
    uint8_t maxReached[ADC_MAX_CHANNELS];
} ADC_Data;

// Function prototypes
void ADC_Init(ADC_HandleTypeDef* hadc);
void ADC_ReadAllChannels(ADC_HandleTypeDef* hadc, ADC_Data* data);
uint8_t ADC_CheckMaxVoltage(ADC_Data* data, float threshold);

#endif /* ADC_H */
