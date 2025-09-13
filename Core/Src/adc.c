#include "adc.h"
#include "main.h"
#include "uart.h"     // UART_TransmitString
#include "global.h"   // motorStatus

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* ======================== Tunables ========================= */
#ifndef THR
#define THR                       1.0f   /* threshold (V) */
#endif

#define EMA_ALPHA                 0.3f   /* low = smoother */
#define HYST_DELTA                0.10f  /* hysteresis margin */
#define GROUND_THRESHOLD           0.5f  /* below this => 0 */
#define DRY_VOLTAGE_THRESHOLD     0.05f  /* "almost 0" for OFF detect */
#define DRY_COUNT_THRESHOLD          3   /* consecutive low samples */

/* ======================== Internal State ========================= */
static float    s_filtered[ADC_CHANNEL_COUNT] = {0};
static uint8_t  s_level_flags[ADC_CHANNEL_COUNT] = {0};
static uint8_t  s_low_counts[ADC_CHANNEL_COUNT] = {0};

static const uint32_t adcChannels[ADC_CHANNEL_COUNT] = {
    ADC_CHANNEL_0,  // PA0
    ADC_CHANNEL_1,  // PA1
    ADC_CHANNEL_2,  // PA2
    ADC_CHANNEL_3,  // PA3
    ADC_CHANNEL_4,  // PA4
    ADC_CHANNEL_5   // PA5
};

static char dataPacketTx[16];

/* ======================== Helper ========================= */
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

    float voltage = 0.0f;

    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK)
    {
        uint32_t raw = HAL_ADC_GetValue(hadc);
        voltage = (raw * 3.3f) / 4095.0f;
    }

    HAL_ADC_Stop(hadc);
    return voltage;
}

/* ======================== Public API ========================= */

void ADC_Init(ADC_HandleTypeDef* hadc)
{
    if (HAL_ADCEx_Calibration_Start(hadc) != HAL_OK)
    {
        Error_Handler();
    }
}

void ADC_ReadAllChannels(ADC_HandleTypeDef* hadc, ADC_Data* data)
{
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        float voltage = readChannelVoltage(hadc, adcChannels[i]);

        /* apply EMA smoothing */
        if (s_filtered[i] == 0.0f)
            s_filtered[i] = voltage;
        else
            s_filtered[i] = (EMA_ALPHA * voltage) + (1.0f - EMA_ALPHA) * s_filtered[i];

        voltage = s_filtered[i];

        /* handle ground-level clipping */
        if (voltage < GROUND_THRESHOLD)
            voltage = 0.0f;

        data->voltages[i] = voltage;
        data->rawValues[i] = (uint16_t)((voltage * 4095.0f) / 3.3f);
        data->maxReached[i] = (voltage >= 3.2f) ? 1 : 0;

        /* ================== Threshold detection with hysteresis ================== */
        if (!s_level_flags[i] && voltage >= THR)
        {
            s_level_flags[i] = 1;

            /* send UART based on channel */
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
        else if (s_level_flags[i] && voltage < (THR - HYST_DELTA))
        {
            s_level_flags[i] = 0;
        }

        /* ================== Debounce for motor off ================== */
        if (voltage < DRY_VOLTAGE_THRESHOLD)
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
            // Optional: UART_TransmitString(&huart1, "@MT0#");
            memset(s_low_counts, 0, sizeof(s_low_counts));
        }
    }

    /* ===== Debug print all channel data ===== */
    char dbg[128];
    int pos = 0;
    pos += snprintf(dbg + pos, sizeof(dbg) - pos, "[ADC] ");
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        pos += snprintf(dbg + pos, sizeof(dbg) - pos,
                        "CH%d:%4u(%.2fV)%s ",
                        i,
                        data->rawValues[i],
                        data->voltages[i],
                        data->maxReached[i] ? "!" : "");
    }
    dbg[sizeof(dbg)-1] = '\0';
    UART_TransmitString(&huart1, dbg);
    UART_TransmitString(&huart1, "\r\n");
}

uint8_t ADC_CheckMaxVoltage(ADC_Data* data, float threshold)
{
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
        if (data->voltages[i] >= threshold) return 1;

    return 0;
}
