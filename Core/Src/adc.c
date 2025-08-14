#include "adc.h"
#include "main.h"
#include "uart.h"   // for UART_TransmitString / UART_ReceiveString
extern UART_HandleTypeDef huart1;

#include "global.h"

// ---------- CONFIGURATION ----------
#define ADC_RESOLUTION       4095.0f
#define ADC_REF_VOLTAGE      3.3f

#define UART_TRANSMIT_THRESHOLD   3.0f   // V
#define MAX_REACHED_THRESHOLD     3.2f   // V
#define DRY_RUN_THRESHOLD         0.0f   // V
#define GROUND_THRESHOLD          0.1f   // V

// ---------- GLOBAL VARIABLES ----------
static const uint32_t adcChannels[ADC_CHANNEL_COUNT] = {
    ADC_CHANNEL_0,  // PA0
    ADC_CHANNEL_1,  // PA1
    ADC_CHANNEL_2,  // PA2
    ADC_CHANNEL_3,  // PA3
    ADC_CHANNEL_4,  // PA4
    ADC_CHANNEL_5   // PA5
};

char dataPacket[16];          // Buffer for outgoing UART packet
uint8_t motorStatus = 0;      // 0 = Off, 1 = On

// ---------- HELPER: Build and send packet ----------
static void SendDataPacket(const char* msg)
{
    UART_ReadDataPacket(dataPacket, msg, strlen(msg)); // prepare packet
    UART_TransmitString(&huart1, dataPacket);          // transmit packet
}

// ---------- ADC Initialization ----------
void ADC_Init(ADC_HandleTypeDef* hadc)
{
    if (HAL_ADCEx_Calibration_Start(hadc) != HAL_OK)
    {
        Error_Handler();
    }
}

// ---------- Read All ADC Channels ----------
void ADC_ReadAllChannels(ADC_HandleTypeDef* hadc, ADC_Data* data)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        // Configure channel
        sConfig.Channel = adcChannels[i];
        if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
            data->rawValues[i] = 0;
            data->voltages[i] = 0.0f;
            data->maxReached[i] = 0;
            continue;
        }

        // Start conversion
        HAL_ADC_Start(hadc);
        if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK)
        {
            // Get raw and voltage
            data->rawValues[i] = HAL_ADC_GetValue(hadc);
            data->voltages[i] = (data->rawValues[i] * ADC_REF_VOLTAGE) / ADC_RESOLUTION;
            data->maxReached[i] = (data->voltages[i] >= MAX_REACHED_THRESHOLD);

            // If close to ground, clamp to 0
            if (data->voltages[i] < GROUND_THRESHOLD) {
                data->rawValues[i] = 0;
                data->voltages[i] = 0.0f;
            }

            // UART packet handling
            if (data->voltages[i] >= UART_TRANSMIT_THRESHOLD)
            {
                switch (i)
                {
                    case 0: SendDataPacket("@10W#"); motorStatus = 1; break;
                    case 1: SendDataPacket("@30W#"); motorStatus = 1; break;
                    case 2: SendDataPacket("@70W#"); motorStatus = 1; break;
                    case 3: SendDataPacket("@1:W#"); motorStatus = 1; break;
                    case 4: SendDataPacket("@DRY#"); motorStatus = 1; break;
                    default: break;
                }
            }
            else if (data->voltages[i] <= DRY_RUN_THRESHOLD && motorStatus == 1)
            {
                // Dry run detected
                SendDataPacket("@MT0#");
                motorStatus = 0;
            }
        }
        else
        {
            // Conversion timeout/error
            data->rawValues[i] = 0;
            data->voltages[i] = 0.0f;
            data->maxReached[i] = 0;
        }
    }

    // ---------- Handle incoming UART ----------
    char receivedData[20] = {0};

    UART_ReceiveString(&huart1, receivedData, sizeof(receivedData));

    if (receivedData[0] != '\0') {   // only process if something received
        UART_ProcessReceivedData(receivedData);
    }

}

// ---------- Max Voltage Check ----------
uint8_t ADC_CheckMaxVoltage(ADC_Data* data, float threshold)
{
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        if (data->voltages[i] >= threshold) {
            return 1;
        }
    }
    return 0;
}
