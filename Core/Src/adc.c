#include "adc.h"
#include "main.h"

// ADC channel configuration
static const uint32_t adcChannels[ADC_CHANNEL_COUNT] = {
    ADC_CHANNEL_0,  // PA0
    ADC_CHANNEL_1,  // PA1
    ADC_CHANNEL_2,  // PA2
    ADC_CHANNEL_3,  // PA3
    ADC_CHANNEL_4,  // PA4
    ADC_CHANNEL_5   // PA5
};

/**
  * @brief Initializes the ADC hardware
  * @param hadc: Pointer to ADC handle
  */
void ADC_Init(ADC_HandleTypeDef* hadc)
{
    // Calibration
    if (HAL_ADCEx_Calibration_Start(hadc) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief Reads all ADC channels
  * @param hadc: Pointer to ADC handle
  * @param data: Pointer to ADC_Data struct to store results
  */
void ADC_ReadAllChannels(ADC_HandleTypeDef* hadc, ADC_Data* data)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        // Configure channel
        sConfig.Channel = adcChannels[i];
        HAL_ADC_ConfigChannel(hadc, &sConfig);

        // Start conversion
        HAL_ADC_Start(hadc);
        if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK)
        {
            data->rawValues[i] = HAL_ADC_GetValue(hadc);
            data->voltages[i] = (data->rawValues[i] * 3.3f) / 4095.0f;
            data->maxReached[i] = (data->voltages[i] >= 3.2f) ? 1 : 0; // 3.2V threshold
        }
    }
}

/**
  * @brief Checks if any channel reached maximum voltage
  * @param data: Pointer to ADC_Data struct
  * @param threshold: Voltage threshold (e.g., 3.2V)
  * @return 1 if any channel reached max, 0 otherwise
  */
uint8_t ADC_CheckMaxVoltage(ADC_Data* data, float threshold)
{
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        if (data->voltages[i] >= threshold)
        {
            return 1;
        }
    }
    return 0;
}
