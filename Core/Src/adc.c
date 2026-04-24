/* ====================================================================
 * adc.c  —  RECEIVER  (motor-controller node)
 *
 * NODE TYPE: LORA_RECEIVER_NODE
 *
 * ── Two-channel source selection ────────────────────────────────────
 *
 *  Every call to ADC_ReadAllChannels() does:
 *
 *    Step 1 — Read ALL 6 local ADC channels through the EMA filter.
 *             This keeps s_filtered[] current even when LoRa is active,
 *             so the fallback path has fresh values instantly.
 *
 *    Step 2 — Check LoRa_IsWirelessDataValid().
 *              If VALID   → call inject_wireless_level() which
 *                           OVERWRITES:
 *                             voltages[0..3]  from TL (tank level)
 *                             voltages[5]     from WD (well-dry flag)
 *                           CH4 (ground water) is ALWAYS from local ADC.
 *              If INVALID → local ADC values used for ALL channels
 *                           (safe bench / fallback mode).
 *
 * ── Voltage synthesis for CH0–CH3 (tank level probes) ───────────────
 *
 *   model_handle.c uses  PROBE_THRESHOLD = 0.50 V
 *   voltage < 0.50 V  →  probe submerged  (water has reached it)
 *   voltage ≥ 0.50 V  →  probe above water
 *
 *   Receiver probe layout:
 *     voltages[0] = 100 % probe
 *     voltages[1] =  75 % probe
 *     voltages[2] =  50 % probe
 *     voltages[3] =  25 % probe
 *
 *   Wireless level → probe synthesis:
 *     TX 100 % → RX all 4 submerged  (FULL)
 *     TX  80 % → RX 75 % + below submerged
 *     TX  60 % → RX 50 % + below submerged
 *     TX  40 % → RX 25 % probe submerged only
 *     TX  20 % → RX all 4 dry       (EMPTY/LOW)
 *     TX   0 % → RX all 4 dry
 *
 * ── Voltage synthesis for CH5 (dry-run / well-dry sensor) ───────────
 *
 *   model_handle.c ModelHandle_CheckDryRun():
 *     senseDryRun = (voltages[5] < 0.30 V)
 *
 *   WD=0 (transmitter: well has water) →  inject 1.0 V  →  senseDryRun=false
 *   WD=1 (transmitter: well DRY)       →  inject 0.0 V  →  senseDryRun=true
 *                                          → motor protection will activate
 *
 * ── Offline fallback ─────────────────────────────────────────────────
 *
 *   When LoRa link is down (no packet for >60 s):
 *     CH0–CH3 → physical ADC (local level probes at motor side)
 *     CH5     → physical ADC (local dry-run sensor at motor side)
 *     CH4     → physical ADC (always — ground water is local)
 * ==================================================================== */

#include "adc.h"
#include "lora.h"       /* LoRa_IsWirelessDataValid / LoRa_GetWireless*() */
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

/* ── Injected probe voltages used when writing wireless data ─────────
 *
 *  PROBE_SUBMERGED  must be < PROBE_THRESHOLD (0.50 V)
 *  PROBE_DRY        must be > PROBE_THRESHOLD
 *  SENSOR_WATER     must be ≥ 0.30 V  (senseDryRun = false)
 *  SENSOR_DRY       must be <  0.30 V (senseDryRun = true  → protection)
 * ──────────────────────────────────────────────────────────────────── */
#define PROBE_SUBMERGED  0.0f   /* < PROBE_THRESHOLD → water present    */
#define PROBE_DRY        1.0f   /* > PROBE_THRESHOLD → no water         */
#define SENSOR_WATER     1.0f   /* CH5 voltage when WD=0 (well OK)      */
#define SENSOR_DRY       0.0f   /* CH5 voltage when WD=1 (well dry)     */

/* ── Module state ───────────────────────────────────────────────────── */
float g_adcVoltages[ADC_CHANNEL_COUNT] = {0};
float g_acVoltage_raw = 0.0f;
float g_acCurrent_raw = 0.0f;
float g_acVoltage_avg = 0.0f;
float g_acCurrent_avg = 0.0f;
bool  g_overload      = false;

static float   s_filtered[ADC_CHANNEL_COUNT]   = {0};
static uint8_t s_level_flags[ADC_CHANNEL_COUNT] = {0};
static float   s_prev_volt[ADC_CHANNEL_COUNT]   = {0};

static const uint32_t adcChannels[ADC_CHANNEL_COUNT] = {
    ADC_CHANNEL_0,
    ADC_CHANNEL_1,
    ADC_CHANNEL_2,
    ADC_CHANNEL_3,
    ADC_CHANNEL_4,
    ADC_CHANNEL_5
};

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

/* ── inject_wireless_level ──────────────────────────────────────────
 *
 *  Called from ADC_ReadAllChannels() when LoRa_IsWirelessDataValid()
 *  returns true.  Overwrites:
 *    data->voltages[0..3]  — synthesised from lvlPct (tank level)
 *    data->voltages[5]     — synthesised from wellDry (WD flag)
 *
 *  CH4 (ground water) is left unchanged (read from local ADC above).
 *
 *  s_filtered[] is also updated for CH0–CH3 and CH5 so that the EMA
 *  state does not snap back to stale physical values if the LoRa link
 *  momentarily drops and then recovers.
 *
 *  Parameters
 *    lvlPct   : wireless tank level  0–100 %
 *    wellDry  : 0 = well has water  |  1 = well DRY alarm from TX
 * ──────────────────────────────────────────────────────────────────── */
static void inject_wireless_level(ADC_Data *data,
                                   uint8_t   lvlPct,
                                   uint8_t   wellDry)
{
    /* Clamp to valid range */
    if (lvlPct > 100) lvlPct = 100;

    /* ── CH0–CH3: Tank level probe synthesis ─────────────────────
     *  Each probe is submerged once the water level reaches or
     *  exceeds its percentage threshold:
     *    voltages[0] = 100 % probe
     *    voltages[1] =  75 % probe
     *    voltages[2] =  50 % probe
     *    voltages[3] =  25 % probe
     * ─────────────────────────────────────────────────────────── */
    float v0 = (lvlPct >= 100) ? PROBE_SUBMERGED : PROBE_DRY;
    float v1 = (lvlPct >=  75) ? PROBE_SUBMERGED : PROBE_DRY;
    float v2 = (lvlPct >=  50) ? PROBE_SUBMERGED : PROBE_DRY;
    float v3 = (lvlPct >=  25) ? PROBE_SUBMERGED : PROBE_DRY;

    data->voltages[0] = v0;
    data->voltages[1] = v1;
    data->voltages[2] = v2;
    data->voltages[3] = v3;

    /* Sync EMA state so fallback re-entry is smooth */
    s_filtered[0] = v0;
    s_filtered[1] = v1;
    s_filtered[2] = v2;
    s_filtered[3] = v3;

    /* Update raw values for diagnostic code */
    for (int i = 0; i < 4; i++)
        data->rawValues[i] = (uint16_t)((data->voltages[i] * ADC_RES) / VREF);

    /* ── CH5: Dry-run sensor synthesis from WD flag ──────────────
     *
     *  model_handle.c: senseDryRun = (voltages[5] < 0.30 V)
     *
     *  WD=0 (well has water) → SENSOR_WATER (1.0 V) → senseDryRun=false
     *  WD=1 (well DRY)       → SENSOR_DRY   (0.0 V) → senseDryRun=true
     *                          → dry-run FSM will fire → motor stops
     * ─────────────────────────────────────────────────────────── */
    float v5 = (wellDry != 0) ? SENSOR_DRY : SENSOR_WATER;

    data->voltages[5]  = v5;
    s_filtered[5]      = v5;   /* keep EMA in sync */
    data->rawValues[5] = (uint16_t)((v5 * ADC_RES) / VREF);

    /* CH4 (ground water) is intentionally NOT touched here —
     * it is always read from the local ADC on the receiver side    */
}

/* ── ADC_ReadAllChannels ────────────────────────────────────────────
 *
 *  Step 1: Read all 6 physical ADC channels through EMA filter.
 *          This always runs so s_filtered[] stays current regardless
 *          of LoRa state.
 *
 *  Step 2: If LoRa wireless data is valid, overwrite CH0–CH3 and
 *          CH5 with synthesised values from the last received packet.
 *          CH4 (ground water) is always kept from local ADC.
 *
 *  Step 3: CH0–CH3 / CH4 / CH5 level-flag events are evaluated on
 *          the FINAL voltages (after any wireless injection), which
 *          keeps event detection consistent regardless of source.
 * ──────────────────────────────────────────────────────────────────── */
void ADC_ReadAllChannels(ADC_HandleTypeDef *hadc, ADC_Data *data)
{
    char loraPacket[32];
    loraPacket[0] = '\0';

    /* ── Step 1: Sample and EMA-filter every local channel ─────── */
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
            s_prev_volt[i] = v;
    }

    /* ── Step 2: Wireless override ──────────────────────────────
     *
     *  When the LoRa link is healthy, synthesise CH0–CH3 (tank level)
     *  and CH5 (dry-run) from the last received packet.
     *
     *  CH4 is NEVER overridden — ground-water detection is a local
     *  physical sensor permanently wired to the motor-controller PCB.
     *
     *  If the link has timed out (>60 s silent) or was never established,
     *  LoRa_IsWirelessDataValid() returns false and the local ADC values
     *  from Step 1 are used unchanged — safe bench / wired test mode.
     * ─────────────────────────────────────────────────────────── */
    if (LoRa_IsWirelessDataValid())
    {
        uint8_t wirelessLevel  = LoRa_GetWirelessTankLevel();
        uint8_t wirelessWellDry = LoRa_GetWirelessWellDry();
        inject_wireless_level(data, wirelessLevel, wirelessWellDry);
    }
    /* else: local ADC values for all channels — fallback / offline mode */

    /* ── Step 3: Level-flag events on final voltages ────────────
     *
     *  These flags drive LoRa event packets on the transmitter side
     *  but are kept here for debug / UART diagnostic symmetry.
     *  Evaluated AFTER wireless injection so they reflect the true
     *  state seen by model_handle.c.
     * ─────────────────────────────────────────────────────────── */
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        float v = data->voltages[i];

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

        if (i == 4)   /* Ground water — always local ADC */
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

        if (i == 5)   /* Dry-run sensor — local when offline, injected when online */
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
}

/* ── Threshold check (unchanged) ───────────────────────────────────── */
uint8_t ADC_CheckMaxVoltage(ADC_Data *data, float threshold)
{
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
        if (data->voltages[i] >= threshold) return 1;
    return 0;
}
