#include "adc.h"
#include "main.h"
#include "uart.h"
#include "global.h"
#include "led.h"     // ✅ for LED control

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

// === AC / Current sensing config ===
#define VREF                      3.3f
#define ADC_RES                   4095.0f
#define VOLTAGE_DIV_RATIO         66.67f   // ⚠️ adjust for your divider (e.g. 220V -> 3.3V)
#define ACS712_SENSITIVITY        0.066f   // V/A for ACS712-30A (185mV/A=5A, 100mV/A=20A, 66mV/A=30A)
#define ACS712_ZERO_OFFSET        (VREF / 2.0f)  // ~1.65V at 0A

#define AC_OVERLOAD_CURRENT       5.0f     // Ampere threshold (adjust to motor rating)
#define AC_OVERVOLTAGE            260.0f   // Volt threshold (adjust as needed)

// === Averaging and smoothing ===
#define AC_AVG_SAMPLES            20      // number of samples to average
#define AC_EMA_ALPHA              0.2f    // smoothing factor for EMA

// === Exported for monitoring (Live Expressions) ===
float g_adcVoltages[ADC_CHANNEL_COUNT] = {0};

float g_acVoltage_raw = 0.0f;     // instantaneous sampled AC voltage
float g_acCurrent_raw = 0.0f;     // instantaneous sampled AC current
float g_acVoltage_avg = 0.0f;     // smoothed AC voltage
float g_acCurrent_avg = 0.0f;     // smoothed AC current

bool  g_overload  = false;

static float    s_filtered[ADC_CHANNEL_COUNT] = {0};
static uint8_t  s_level_flags[ADC_CHANNEL_COUNT] = {0};
static uint8_t  s_low_counts[ADC_CHANNEL_COUNT] = {0};
static float    s_prev_volt[ADC_CHANNEL_COUNT] = {0};

// Existing channels (water sensors etc.)
static const uint32_t adcChannels[ADC_CHANNEL_COUNT] = {
    ADC_CHANNEL_0,//dry run
    ADC_CHANNEL_1,//water level
    ADC_CHANNEL_2,//water level
    ADC_CHANNEL_3,//water level
    ADC_CHANNEL_4,//water level
    ADC_CHANNEL_5 //water level
};

// ⚠️ Add dedicated channels for AC voltage/current (configure in CubeMX!)
#define ADC_CHANNEL_AC_VOLTAGE    ADC_CHANNEL_6
#define ADC_CHANNEL_AC_CURRENT    ADC_CHANNEL_7

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
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK) {
        uint32_t raw = HAL_ADC_GetValue(hadc);
        v = (raw * VREF) / ADC_RES;
    }
    HAL_ADC_Stop(hadc);
    return v;
}

/* --- helper: average multiple samples for better stability --- */
static float readChannelAveraged(ADC_HandleTypeDef *hadc, uint32_t channel, uint8_t samples)
{
    float sum = 0.0f;
    for (uint8_t i = 0; i < samples; i++) {
        sum += readChannelVoltage(hadc, channel);
    }
    return sum / samples;
}

/* --- Public API --- */
void ADC_Init(ADC_HandleTypeDef* hadc)
{
    if (HAL_ADCEx_Calibration_Start(hadc) != HAL_OK) {
        Error_Handler();
    }
}

void ADC_ReadAllChannels(ADC_HandleTypeDef* hadc, ADC_Data* data)
{
    bool changed = false;

    // === Process water sensor channels ===
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
        data->rawValues[i] = (uint16_t)((v * ADC_RES) / VREF);
        data->maxReached[i] = (v >= 3.2f);

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

        if (v < DRY_VOLTAGE_THRESHOLD) {
            if (s_low_counts[i] < 0xFF) s_low_counts[i]++;
        } else {
            s_low_counts[i] = 0;
        }

        if (motorStatus == 1 && s_low_counts[i] >= DRY_COUNT_THRESHOLD) {
            motorStatus = 0;
            memset(s_low_counts, 0, sizeof(s_low_counts));
        }
    }

    // === NEW: read AC voltage & current with averaging + smoothing ===
    g_acVoltage_raw = readChannelAveraged(hadc, ADC_CHANNEL_AC_VOLTAGE, AC_AVG_SAMPLES) * VOLTAGE_DIV_RATIO;
    g_acCurrent_raw = (readChannelAveraged(hadc, ADC_CHANNEL_AC_CURRENT, AC_AVG_SAMPLES) - ACS712_ZERO_OFFSET) / ACS712_SENSITIVITY;

    // Apply EMA smoothing
    g_acVoltage_avg = (AC_EMA_ALPHA * g_acVoltage_raw) + ((1.0f - AC_EMA_ALPHA) * g_acVoltage_avg);
    g_acCurrent_avg = (AC_EMA_ALPHA * g_acCurrent_raw) + ((1.0f - AC_EMA_ALPHA) * g_acCurrent_avg);

    // === Overload detection ===
    g_overload = false;
    if (fabsf(g_acCurrent_avg) > AC_OVERLOAD_CURRENT || g_acVoltage_avg > AC_OVERVOLTAGE) {
        g_overload = true;
        LED_SetIntent(LED_COLOR_RED, LED_MODE_STEADY, 0);  // 🔴 show overload
    } else {
        LED_SetIntent(LED_COLOR_RED, LED_MODE_OFF, 0);
    }

    // === Debug print (only if water level changed) ===
    if (changed) {
        char dbg[128];
//        snprintf(dbg, sizeof(dbg),
//                 "[AC] V=%.1fV I=%.2fA (raw V=%.1f I=%.2f) %s\r\n",
//                 g_acVoltage_avg, g_acCurrent_avg,
//                 g_acVoltage_raw, g_acCurrent_raw,
//                 g_overload ? "OVERLOAD" : "OK");
        UART_TransmitString(&huart1, dbg);
    }
}

uint8_t ADC_CheckMaxVoltage(ADC_Data* data, float threshold)
{
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
        if (data->voltages[i] >= threshold) return 1;
    return 0;
}
