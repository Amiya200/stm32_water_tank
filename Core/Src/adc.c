#include "adc.h"
#include "main.h"
#include "uart.h" // Include the new UART header
#include "global.h" // Include the global header for motorStatus

// ADC channel configuration
static const uint32_t adcChannels[ADC_CHANNEL_COUNT] = {
    ADC_CHANNEL_0,  // PA0
    ADC_CHANNEL_1,  // PA1
    ADC_CHANNEL_2,  // PA2
    ADC_CHANNEL_3,  // PA3
    ADC_CHANNEL_4,  // PA4
    ADC_CHANNEL_5   // PA5
};

// Buffer to hold the data packet for transmission
// This buffer should be large enough to hold the longest message + null terminator
char dataPacketTx[10]; // Increased size to accommodate messages like "@1:W#" + null

// Define the motorStatus variable
//volatile uint8_t motorStatus = 0; // 0: Off, 1: On

// ... (rest of the ADC code remains unchanged)

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
  * @brief Reads all ADC channels and transmits UART messages if thresholds are met.
  * @param hadc: Pointer to ADC handle
  * @param data: Pointer to ADC_Data struct to store results
  */
void ADC_ReadAllChannels(ADC_HandleTypeDef* hadc, ADC_Data* data)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

    // Define the voltage threshold for UART transmission
    const float UART_TRANSMIT_THRESHOLD = 3.0f; // Proper 3V or above
    const float DRY_RUN_THRESHOLD = .0f; // Threshold for dry run detection
    const float GROUND_THRESHOLD = 0.1f; // Threshold to consider as ground (0V)

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
            data->maxReached[i] = (data->voltages[i] >= 3.2f) ? 1 : 0; // Original 3.2V threshold for maxReached flag

            // Check if the voltage is close to ground
            if (data->voltages[i] < GROUND_THRESHOLD)
            {
                data->rawValues[i] = 0; // Set raw value to 0
                data->voltages[i] = 0.0f; // Set voltage to 0.0V
            }

            // Check voltage for UART transmission
            if (data->voltages[i] >= UART_TRANSMIT_THRESHOLD)
            {
                switch (i)
                {
                    case 0: // IN0
                        sprintf(dataPacketTx, "@10W#"); // Use sprintf to format the string
                        UART_TransmitString(&huart1, dataPacketTx);
                        motorStatus = 1; // Motor is on
                        break;
                    case 1: // IN1
                        sprintf(dataPacketTx, "@30W#");
                        UART_TransmitString(&huart1, dataPacketTx);
                        motorStatus = 1; // Motor is on
                        break;
                    case 2: // IN2
                        sprintf(dataPacketTx, "@70W#");
                        UART_TransmitString(&huart1, dataPacketTx);
                        motorStatus = 1; // Motor is on
                        break;
                    case 3: // IN3
                        sprintf(dataPacketTx, "@1:W#");
                        UART_TransmitString(&huart1, dataPacketTx);
                        motorStatus = 1; // Motor is on
                        break;
                    case 4: // IN4
                        sprintf(dataPacketTx, "@DRY#");
                        UART_TransmitString(&huart1, dataPacketTx);
                        motorStatus = 1; // Set motor status to on for dry run
                        break;
                    // Cases for IN5 are not specified for UART transmission
                    default:
                        break;
                }
            }
            else if (data->voltages[i] < DRY_RUN_THRESHOLD && motorStatus == 1)
            {
                // If the voltage is below the dry run threshold and the motor is on
                // This part was commented out in the original, keeping it commented for consistency
                // sprintf(dataPacketTx, "@MT0#");
                // UART_TransmitString(&huart1, dataPacketTx);
                motorStatus = 0; // Set motor status to off
            }
        }
        else
        {
            // Handle ADC conversion timeout or error if necessary
            // For simplicity, we'll just set values to 0 on error
            data->rawValues[i] = 0;
            data->voltages[i] = 0.0f;
            data->maxReached[i] = 0;
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
