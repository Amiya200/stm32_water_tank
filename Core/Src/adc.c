#include "adc.h"
#include "main.h"
#include "uart.h"
#include "global.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#ifndef THR
#define THR                       1.0f
#endif

#define EMA_ALPHA                 0.3f
#define HYST_DELTA                0.10f
#define GROUND_THRESHOLD           0.5f
#define DRY_VOLTAGE_THRESHOLD     0.05f
#define DRY_COUNT_THRESHOLD          3
#define PRINT_DELTA                0.05f  // only print if change > 0.05V

// ✅ NEW: Exported for Live Expressions
float g_adcVoltages[ADC_CHANNEL_COUNT] = {0};

static float    s_filtered[ADC_CHANNEL_COUNT] = {0};
static uint8_t  s_level_flags[ADC_CHANNEL_COUNT] = {0};
static uint8_t  s_low_counts[ADC_CHANNEL_COUNT] = {0};
static float    s_prev_volt[ADC_CHANNEL_COUNT] = {0};

static const uint32_t adcChannels[ADC_CHANNEL_COUNT] = {
    ADC_CHANNEL_0,//dry run
    ADC_CHANNEL_1,//water level
    ADC_CHANNEL_2,//water level
    ADC_CHANNEL_3,//water level
    ADC_CHANNEL_4,//water level
    ADC_CHANNEL_5//water level
};

static char dataPacketTx[16];

/* --- helper: sample one channel --- */
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
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK)
    {
        uint32_t raw = HAL_ADC_GetValue(hadc);
        v = (raw * 3.3f) / 4095.0f;
    }
    HAL_ADC_Stop(hadc);
    return v;
}

/* --- Public API --- */
void ADC_Init(ADC_HandleTypeDef* hadc)
{
    if (HAL_ADCEx_Calibration_Start(hadc) != HAL_OK)
    {
        Error_Handler();
    }
}

void ADC_ReadAllChannels(ADC_HandleTypeDef* hadc, ADC_Data* data)
{
    bool changed = false;

    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        float v = readChannelVoltage(hadc, adcChannels[i]);

        if (s_filtered[i] == 0.0f)
            s_filtered[i] = v;
        else
            s_filtered[i] = EMA_ALPHA * v + (1 - EMA_ALPHA) * s_filtered[i];

        v = s_filtered[i];

        if (v < GROUND_THRESHOLD)
            v = 0.0f;

        data->voltages[i] = v;
        data->rawValues[i] = (uint16_t)((v * 4095.0f) / 3.3f);
        data->maxReached[i] = (v >= 3.2f);

        // ✅ NEW: export to debugger live
        g_adcVoltages[i] = v;

        if (fabsf(v - s_prev_volt[i]) > PRINT_DELTA) {
            changed = true;
            s_prev_volt[i] = v;
        }

        /* --- threshold and debounce logic --- */
        if (!s_level_flags[i] && v >= THR)
        {
            s_level_flags[i] = 1;
            switch (i)
            {
                case 0: snprintf(dataPacketTx, sizeof(dataPacketTx), "@10W#"); break;
                case 1: snprintf(dataPacketTx, sizeof(dataPacketTx), "@30W#"); break;
                case 2: snprintf(dataPacketTx, sizeof(dataPacketTx), "@70W#"); break;
                case 3: snprintf(dataPacketTx, sizeof(dataPacketTx), "@1:W#"); break;
                case 4: snprintf(dataPacketTx, sizeof(dataPacketTx), "@DRY#"); break;
                default: dataPacketTx[0] = '\0'; break;
            }
            if (dataPacketTx[0])
                UART_TransmitString(&huart1, dataPacketTx);

            motorStatus = 1;
            s_low_counts[i] = 0;
        }
        else if (s_level_flags[i] && v < (THR - HYST_DELTA))
        {
            s_level_flags[i] = 0;
        }

        if (v < DRY_VOLTAGE_THRESHOLD)
        {
            if (s_low_counts[i] < 0xFF) s_low_counts[i]++;
        }
        else
        {
            s_low_counts[i] = 0;
        }

        if (motorStatus == 1 && s_low_counts[i] >= DRY_COUNT_THRESHOLD)
        {
            motorStatus = 0;
            memset(s_low_counts, 0, sizeof(s_low_counts));
        }
    }

    /* print only if changed */
    if (changed)
    {
        char dbg[128];
        int pos = snprintf(dbg, sizeof(dbg), "[ADC] ");
        for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
        {
//            pos += snprintf(dbg+pos, sizeof(dbg)-pos,
//                            "CH%d:%4u(%.2fV)%s ",
//                            i,
//                            data->rawValues[i],
//                            data->voltages[i],
//                            data->maxReached[i] ? "!" : "");
        }
        dbg[sizeof(dbg)-1] = '\0';
        UART_TransmitString(&huart1, dbg);
        UART_TransmitString(&huart1, "\r\n");
    }
}

uint8_t ADC_CheckMaxVoltage(ADC_Data* data, float threshold)
{
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
        if (data->voltages[i] >= threshold) return 1;
    return 0;
}
