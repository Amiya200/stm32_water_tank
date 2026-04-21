/* ====================================================================
 * adc.c  —  RECEIVER (motor-controller node)
 *
 * What changed vs the old version:
 *
 * After every local ADC scan, ADC_ReadAllChannels() checks whether
 * a valid LoRa wireless level is available.  If so, it synthesises
 * probe voltages for CH0–CH3 (the tank level probes) from the
 * remote reading.
 *
 * CH4 (ground water) and CH5 (dry run) are at the MOTOR SIDE and
 * continue to be read from local ADC — unchanged.
 *
 * Voltage synthesis logic:
 *   model_handle.c uses  PROBE_THRESHOLD = 0.50 V
 *   voltage < 0.50 V  →  probe is submerged (water has reached it)
 *   voltage > 0.50 V  →  probe is above water
 *
 *   Receiver probe layout (4 probes):
 *     voltages[0] = 100 % probe   (submerged when tank ≥ 100 %)
 *     voltages[1] =  75 % probe   (submerged when tank ≥  75 %)
 *     voltages[2] =  50 % probe   (submerged when tank ≥  50 %)
 *     voltages[3] =  25 % probe   (submerged when tank ≥  25 %)
 *
 *   Transmitter → Receiver level mapping:
 *     TX 100 %  →  RX 100 %
 *     TX  80 %  →  RX  75 %  (80 ≥ 75)
 *     TX  60 %  →  RX  50 %
 *     TX  40 %  →  RX  25 %
 *     TX  20 %  →  RX   0 %  (below lowest probe)
 *     TX   0 %  →  RX   0 %
 *
 * If no valid wireless data (LoRa timed out > 60 s), local ADC
 * probes are used as fall-back — safe default for wired testing.
 * ==================================================================== */

#include "adc.h"
#include "lora.h"           /* LoRa_IsWirelessDataValid / LoRa_GetWirelessTankLevel */
#include "main.h"
#include "uart.h"
#include "global.h"
#include "led.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "model_handle.h"

/* ── Configuration ─────────────────────────────────────────────────── */
#ifndef THR
#define THR                       1.0f
#endif
#define EMA_ALPHA                 0.3f
#define HYST_DELTA                0.10f
#define GROUND_THRESHOLD          0.5f
#define DRY_VOLTAGE_THRESHOLD     0.05f
#define DRY_COUNT_THRESHOLD       3
#define PRINT_DELTA               0.05f
#define VREF                      3.3f
#define ADC_RES                   4095.0f

/* ── Probe voltage levels used when injecting wireless data ─────────── */
#define PROBE_SUBMERGED   0.0f   /* < PROBE_THRESHOLD (0.50 V) = water  */
#define PROBE_DRY         1.0f   /* > PROBE_THRESHOLD          = no water*/

/* ── Module state ───────────────────────────────────────────────────── */
float g_adcVoltages[ADC_CHANNEL_COUNT] = {0};
float g_acVoltage_raw = 0.0f;
float g_acCurrent_raw = 0.0f;
float g_acVoltage_avg = 0.0f;
float g_acCurrent_avg = 0.0f;
bool  g_overload  = false;

static float   s_filtered[ADC_CHANNEL_COUNT]   = {0};
static uint8_t s_level_flags[ADC_CHANNEL_COUNT] = {0};
static uint8_t s_low_counts[ADC_CHANNEL_COUNT]  = {0};
static float   s_prev_volt[ADC_CHANNEL_COUNT]   = {0};

static const uint32_t adcChannels[ADC_CHANNEL_COUNT] = {
    ADC_CHANNEL_0,
    ADC_CHANNEL_1,
    ADC_CHANNEL_2,
    ADC_CHANNEL_3,
    ADC_CHANNEL_4,
    ADC_CHANNEL_5
};

#define ADC_CHANNEL_AC_VOLTAGE  ADC_CHANNEL_6
#define ADC_CHANNEL_AC_CURRENT  ADC_CHANNEL_7

static char dataPacketTx[16];

/* ── Low-level single-channel read ─────────────────────────────────── */
static float readChannelVoltage(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = channel;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) return 0.0f;
    if (HAL_ADC_Start(hadc)                   != HAL_OK) return 0.0f;

    float v = 0.0f;
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK)
    {
        uint32_t raw = HAL_ADC_GetValue(hadc);
        v = (raw * VREF) / ADC_RES;
    }
    HAL_ADC_Stop(hadc);
    return v;
}

/* ── Init ───────────────────────────────────────────────────────────── */
void ADC_Init(ADC_HandleTypeDef *hadc)
{
    if (HAL_ADCEx_Calibration_Start(hadc) != HAL_OK)
        Error_Handler();
}

/* ── Inject wireless level into tank-probe channels (CH0–CH3) ────────
 *
 * Called inside ADC_ReadAllChannels() when valid LoRa data exists.
 * Overwrites data->voltages[0..3] and s_filtered[0..3] so the EMA
 * state stays consistent with what model_handle.c will read.
 * ──────────────────────────────────────────────────────────────────── */
static void inject_wireless_level(ADC_Data *data, uint8_t lvlPct)
{
    /* Clamp to multiples of 20 that the transmitter can send */
    if (lvlPct > 100) lvlPct = 100;

    /* Synthesise probe voltages based on level thresholds */
    float v0 = (lvlPct >= 100) ? PROBE_SUBMERGED : PROBE_DRY;   /* 100 % */
    float v1 = (lvlPct >=  75) ? PROBE_SUBMERGED : PROBE_DRY;   /*  75 % */
    float v2 = (lvlPct >=  50) ? PROBE_SUBMERGED : PROBE_DRY;   /*  50 % */
    float v3 = (lvlPct >=  25) ? PROBE_SUBMERGED : PROBE_DRY;   /*  25 % */

    data->voltages[0] = v0;
    data->voltages[1] = v1;
    data->voltages[2] = v2;
    data->voltages[3] = v3;

    /* Keep EMA state in sync so the next local-ADC cycle doesn't
     * slam the filter back to a stale physical reading               */
    s_filtered[0] = v0;
    s_filtered[1] = v1;
    s_filtered[2] = v2;
    s_filtered[3] = v3;

    /* Update rawValues for any diagnostic code that uses them */
    for (int i = 0; i < 4; i++)
        data->rawValues[i] = (uint16_t)((data->voltages[i] * ADC_RES) / VREF);
}

/* ── Read all channels ──────────────────────────────────────────────── */
void ADC_ReadAllChannels(ADC_HandleTypeDef *hadc, ADC_Data *data)
{
    bool changed = false;
    char loraPacket[32];
    loraPacket[0] = '\0';

    /* ── Step 1: Read all 6 local ADC channels (EMA filtered) ─────── */
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

        /* --- Level flags (CH0–CH3) for LoRa / debug events --- */
        if (i <= 3)
        {
            if (!s_level_flags[i] && v >= THR)
            {
                s_level_flags[i] = 1;
                switch (i)
                {
                    case 0: snprintf(dataPacketTx, sizeof(dataPacketTx), "@L1#");   break;
                    case 1: snprintf(dataPacketTx, sizeof(dataPacketTx), "@L2#");   break;
                    case 2: snprintf(dataPacketTx, sizeof(dataPacketTx), "@L3#");   break;
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

        /* --- Ground water (CH4) --- */
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

        /* --- Dry run sensor (CH5) — stays local at motor side --- */
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

    /* ── Step 2: Wireless override for tank-level probes (CH0–CH3) ──
     *
     * If the LoRa receiver has a valid, non-expired reading from the
     * transmitter at the tank, synthesise probe voltages from it.
     *
     * CH4 (ground water) and CH5 (dry run) are ALWAYS taken from the
     * local ADC because those sensors are physically at the motor site.
     * ──────────────────────────────────────────────────────────────── */
    if (LoRa_IsWirelessDataValid())
    {
        uint8_t wirelessLevel = LoRa_GetWirelessTankLevel();
        inject_wireless_level(data, wirelessLevel);
    }
    /* If wireless data is not valid, local ADC voltages[0-3] are used
     * unchanged — this is the safe fall-back for wired bench testing  */
}

/* ── Threshold check (unchanged) ───────────────────────────────────── */
uint8_t ADC_CheckMaxVoltage(ADC_Data *data, float threshold)
{
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
        if (data->voltages[i] >= threshold) return 1;
    return 0;
}
