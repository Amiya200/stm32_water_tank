#include "adc.h"
#include "main.h"
#include "uart.h"
#include "global.h"
#include "led.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "model_handle.h"

#ifndef THR
#define THR                       1.0f
#endif

#define EMA_ALPHA                 0.3f
#define HYST_DELTA                0.10f
#define GROUND_THRESHOLD           0.5f
#define DRY_VOLTAGE_THRESHOLD     0.05f
#define DRY_COUNT_THRESHOLD          3
#define PRINT_DELTA                0.05f
#define VREF                      3.3f
#define ADC_RES                   4095.0f
float g_adcVoltages[ADC_CHANNEL_COUNT] = {0};

float g_acVoltage_raw = 0.0f;
float g_acCurrent_raw = 0.0f;
float g_acVoltage_avg = 0.0f;
float g_acCurrent_avg = 0.0f;
bool  g_overload  = false;
static float    s_filtered[ADC_CHANNEL_COUNT] = {0};
static uint8_t  s_level_flags[ADC_CHANNEL_COUNT] = {0};
static uint8_t  s_low_counts[ADC_CHANNEL_COUNT] = {0};
static float    s_prev_volt[ADC_CHANNEL_COUNT] = {0};
static const uint32_t adcChannels[ADC_CHANNEL_COUNT] = {
    ADC_CHANNEL_0,
    ADC_CHANNEL_1,
    ADC_CHANNEL_2,
    ADC_CHANNEL_3,
    ADC_CHANNEL_4,
    ADC_CHANNEL_5
};
#define ADC_CHANNEL_AC_VOLTAGE    ADC_CHANNEL_6
#define ADC_CHANNEL_AC_CURRENT    ADC_CHANNEL_7
static char dataPacketTx[16];
static float readChannelVoltage(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
        return 0.0f;
    if (HAL_ADC_Start(hadc) != HAL_OK)
        return 0.0f;
    float v = 0.0f;
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK) {
        uint32_t raw = HAL_ADC_GetValue(hadc);
        v = (raw * VREF) / ADC_RES;
    }
    HAL_ADC_Stop(hadc);
    return v;
}
static float readChannelAveraged(ADC_HandleTypeDef *hadc, uint32_t channel, uint8_t samples)
{
    float sum = 0.0f;
    for (uint8_t i = 0; i < samples; i++) {
        sum += readChannelVoltage(hadc, channel);
    }
    return sum / samples;
}
void ADC_Init(ADC_HandleTypeDef* hadc)
{
    if (HAL_ADCEx_Calibration_Start(hadc) != HAL_OK) {
        Error_Handler();
    }
}
void ADC_ReadAllChannels(ADC_HandleTypeDef* hadc, ADC_Data* data)
{
    bool changed = false;
    char loraPacket[32];
    loraPacket[0] = '\0';
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        float v = readChannelVoltage(hadc, adcChannels[i]);
        if (s_filtered[i] == 0.0f)
            s_filtered[i] = v;
        else
            s_filtered[i] = EMA_ALPHA * v + (1.0f - EMA_ALPHA) * s_filtered[i];
        v = s_filtered[i];
        data->voltages[i]   = v;
        data->rawValues[i]  = (uint16_t)((v * ADC_RES) / VREF);
        data->maxReached[i] = (v >= 3.2f);
        g_adcVoltages[i]    = v;
        if (fabsf(v - s_prev_volt[i]) > PRINT_DELTA)
        {
            changed = true;
            s_prev_volt[i] = v;
        }
        if (i <= 3)
        {
            if (!s_level_flags[i] && v >= THR)
            {
                s_level_flags[i] = 1;
                switch (i)
                {
                    case 0: snprintf(dataPacketTx, sizeof(dataPacketTx), "@L1#"); break;
                    case 1: snprintf(dataPacketTx, sizeof(dataPacketTx), "@L2#"); break;
                    case 2: snprintf(dataPacketTx, sizeof(dataPacketTx), "@L3#"); break;
                    case 3: snprintf(dataPacketTx, sizeof(dataPacketTx), "@FULL#"); break;
                    default: dataPacketTx[0] = '\0'; break;
                }
                if (dataPacketTx[0])
                {
                    strncat(loraPacket, dataPacketTx,
                            sizeof(loraPacket) - strlen(loraPacket) - 1);
                    strncat(loraPacket, ";",
                            sizeof(loraPacket) - strlen(loraPacket) - 1);
                }
            }
            else if (s_level_flags[i] && v < (THR - HYST_DELTA))
            {
                s_level_flags[i] = 0;
            }
            continue;
        }
        if (i == 4)
        {
            if (!s_level_flags[i] && v >= GROUND_THRESHOLD)
            {
                s_level_flags[i] = 1;
                snprintf(dataPacketTx, sizeof(dataPacketTx), "@GW#");

                strncat(loraPacket, dataPacketTx,
                        sizeof(loraPacket) - strlen(loraPacket) - 1);
                strncat(loraPacket, ";",
                        sizeof(loraPacket) - strlen(loraPacket) - 1);
            }
            else if (s_level_flags[i] && v < (GROUND_THRESHOLD - HYST_DELTA))
            {
                s_level_flags[i] = 0;
            }
            continue;
        }
        if (i == 5)
        {
            if (!s_level_flags[i] && v >= DRY_VOLTAGE_THRESHOLD)
            {
                s_level_flags[i] = 1;
                snprintf(dataPacketTx, sizeof(dataPacketTx), "@DRY#");

                strncat(loraPacket, dataPacketTx,
                        sizeof(loraPacket) - strlen(loraPacket) - 1);
                strncat(loraPacket, ";",
                        sizeof(loraPacket) - strlen(loraPacket) - 1);
            }
            else if (s_level_flags[i] && v < (DRY_VOLTAGE_THRESHOLD - HYST_DELTA))
            {
                s_level_flags[i] = 0;
            }

            continue;
        }
    }
//    if (changed && loraPacket[0] != '\0')
//    {
//        // LoRa_SendPacket((uint8_t*)loraPacket, strlen(loraPacket));
//    }
}
uint8_t ADC_CheckMaxVoltage(ADC_Data* data, float threshold)
{
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
        if (data->voltages[i] >= threshold) return 1;
    return 0;
}
