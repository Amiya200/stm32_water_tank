/* ====================================================================
 * adc.c  —  RECEIVER  (motor-controller node)
 *
 * NODE TYPE: LORA_RECEIVER_NODE
 *
 * ── Three-tier wireless source selection ────────────────────────────
 *
 *  Every call to ADC_ReadAllChannels() does:
 *
 *    Step 1 — Read ALL 6 local ADC channels through the EMA filter.
 *             This keeps s_filtered[] current regardless of link state,
 *             so the fallback path has fresh values instantly.
 *
 *    Step 2 — Wireless source priority (checked in order):
 *
 *              TIER 1 — LoRa   (LoRa_IsWirelessDataValid())
 *                        Bidirectional, ACK'd — highest confidence.
 *                        Overwrites voltages[0..3] + voltages[5].
 *
 *              TIER 2 — RF433  (RF_IsWirelessDataValid())
 *                        Simplex OOK broadcast — used when LoRa is
 *                        absent/timed-out.  Same overwrite as Tier 1.
 *
 *              TIER 3 — Local ADC  (both wireless links invalid)
 *                        Physical sensors on the motor-controller PCB.
 *                        Safe bench / offline / wired-test mode.
 *
 *             CH4 (ground water) is ALWAYS from local ADC regardless
 *             of which wireless tier is active.
 *
 * v6.1 fix:
 *   Added Tier-2 RF433 fallback.  Previously only LoRa was checked,
 *   so when the TX was in RF433 mode the receiver always fell through
 *   to local ADC even though RF_Task() was successfully decoding and
 *   storing valid RF packets.
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
 *     TX  75 % → RX [1..3] submerged
 *     TX  50 % → RX [2..3] submerged
 *     TX  25 % → RX [3]    submerged only
 *     TX   0 % → RX all 4  dry       (EMPTY)
 *
 * ── Voltage synthesis for CH5 (dry-run / well-dry sensor) ───────────
 *
 *   model_handle.c ModelHandle_CheckDryRun():
 *     senseDryRun = (voltages[5] < 0.30 V)
 *
 *   WD=0 (well has water) → inject 1.0 V → senseDryRun=false
 *   WD=1 (well DRY)       → inject 0.0 V → senseDryRun=true
 *                            → motor protection will activate
 *
 * ── Offline fallback (Tier 3) ────────────────────────────────────────
 *
 *   When both LoRa and RF433 links are down / timed out:
 *     CH0–CH3 → physical ADC (local level probes at motor side)
 *     CH4     → physical ADC (always — ground water is local)
 *     CH5     → physical ADC (local dry-run sensor at motor side)
 * ==================================================================== */

#include "adc.h"
#include "lora.h"   /* LoRa_IsWirelessDataValid / LoRa_GetWireless*() */
#include "rf.h"     /* RF_IsWirelessDataValid  / RF_GetWireless*()    */
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

/* ── Injected probe voltages ─────────────────────────────────────────
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

/* ── Source tracking for diagnostics ───────────────────────────────── */
typedef enum {
    ADC_SRC_LOCAL = 0,
    ADC_SRC_LORA,
    ADC_SRC_RF433
} ADC_WirelessSrc_t;

static ADC_WirelessSrc_t s_lastSrc     = ADC_SRC_LOCAL;
static uint32_t          s_srcChangeTick = 0u;

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
 *  Shared by both Tier 1 (LoRa) and Tier 2 (RF433) paths.
 *  Overwrites data->voltages[0..3] from lvlPct and voltages[5] from
 *  wellDry.  CH4 (ground water) is intentionally NOT touched.
 *
 *  s_filtered[] is also updated for CH0–CH3 and CH5 so that EMA
 *  state does not snap when switching between wireless and local ADC.
 *
 *  Parameters
 *    lvlPct   : tank level 0–100 %
 *    wellDry  : 0 = well has water  |  1 = well DRY alarm from TX
 * ──────────────────────────────────────────────────────────────────── */
static void inject_wireless_level(ADC_Data *data,
                                   uint8_t   lvlPct,
                                   uint8_t   wellDry)
{
    if (lvlPct > 100u) lvlPct = 100u;

    /* ── CH0–CH3: tank level → probe voltages ────────────────────── */
    float v0 = (lvlPct >= 100u) ? PROBE_SUBMERGED : PROBE_DRY;
    float v1 = (lvlPct >=  75u) ? PROBE_SUBMERGED : PROBE_DRY;
    float v2 = (lvlPct >=  50u) ? PROBE_SUBMERGED : PROBE_DRY;
    float v3 = (lvlPct >=  25u) ? PROBE_SUBMERGED : PROBE_DRY;

    data->voltages[0] = v0;
    data->voltages[1] = v1;
    data->voltages[2] = v2;
    data->voltages[3] = v3;

    s_filtered[0] = v0;    /* keep EMA in sync for smooth fallback */
    s_filtered[1] = v1;
    s_filtered[2] = v2;
    s_filtered[3] = v3;

    for (int i = 0; i < 4; i++)
        data->rawValues[i] = (uint16_t)((data->voltages[i] * ADC_RES) / VREF);

    /* ── CH5: well-dry → dry-run sensor voltage ──────────────────── *
     *
     *  model_handle.c: senseDryRun = (voltages[5] < 0.30 V)
     *
     *  WD=0 → SENSOR_WATER (1.0 V) → senseDryRun = false
     *  WD=1 → SENSOR_DRY   (0.0 V) → senseDryRun = true → motor stops
     * ─────────────────────────────────────────────────────────────── */
    float v5 = (wellDry != 0u) ? SENSOR_DRY : SENSOR_WATER;

    data->voltages[5]  = v5;
    s_filtered[5]      = v5;
    data->rawValues[5] = (uint16_t)((v5 * ADC_RES) / VREF);

    /* CH4 is intentionally NOT touched — always local ADC */
}

/* ── ADC_ReadAllChannels ────────────────────────────────────────────
 *
 *  Step 1 : Sample every local ADC channel through EMA.
 *  Step 2 : Apply wireless override (LoRa → RF433 → local).
 *  Step 3 : Evaluate level-flag events on final voltages.
 * ──────────────────────────────────────────────────────────────────── */
void ADC_ReadAllChannels(ADC_HandleTypeDef *hadc, ADC_Data *data)
{
    char loraPacket[32];
    loraPacket[0] = '\0';

    /* ── Step 1: EMA-filter every local channel ─────────────────── */
    for (uint8_t i = 0u; i < ADC_CHANNEL_COUNT; i++)
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

    /* ── Step 2: Wireless source priority ──────────────────────────
     *
     *  TIER 1 — LoRa  (bidirectional, ACK'd — highest confidence)
     *  TIER 2 — RF433 (simplex OOK broadcast — fallback)
     *  TIER 3 — local ADC (both links invalid / timed out)
     *
     *  CH4 is NEVER overridden — ground-water sensor is physically
     *  wired to the motor-controller PCB.
     *
     *  v6.1 fix: RF433 tier added.  Previously only LoRa was checked,
     *  causing the receiver to ignore all RF packets and always use
     *  local ADC when the transmitter was in RF433 mode.
     * ─────────────────────────────────────────────────────────────── */
    if (LoRa_IsWirelessDataValid())
    {
        /* ── Tier 1: LoRa ──────────────────────────────────────────── */
        uint8_t level   = LoRa_GetWirelessTankLevel();
        uint8_t wellDry = LoRa_GetWirelessWellDry();
        inject_wireless_level(data, level, wellDry);

        if (s_lastSrc != ADC_SRC_LORA)
        {
            s_lastSrc     = ADC_SRC_LORA;
            s_srcChangeTick = HAL_GetTick();
            /* Log source switch only on transitions */
            char dbg[64];
            snprintf(dbg, sizeof(dbg),
                     "[ADC] Source → LoRa   Level:%u%%  WD:%u",
                     (unsigned)level, (unsigned)wellDry);
            /* Route through whatever UART function is available */
            HAL_UART_Transmit(
                &huart1,
                (uint8_t *)dbg, (uint16_t)strlen(dbg), 500u);
            HAL_UART_Transmit(
                &huart1,
                (uint8_t *)"\r\n", 2u, 500u);
        }
    }
    else if (RF_IsWirelessDataValid())
    {
        /* ── Tier 2: RF433 ─────────────────────────────────────────── *
         *  Root-cause fix (v6.1):                                       *
         *    This branch was missing entirely.  RF_Task() was decoding  *
         *    and storing RF packets correctly, but they were never       *
         *    applied to the ADC data, so the receiver always ran on      *
         *    local ADC even with a healthy RF433 link.                   */
        uint8_t level   = RF_GetWirelessTankLevel();
        uint8_t wellDry = RF_GetWirelessWellDry();
        inject_wireless_level(data, level, wellDry);

        if (s_lastSrc != ADC_SRC_RF433)
        {
            s_lastSrc     = ADC_SRC_RF433;
            s_srcChangeTick = HAL_GetTick();

            char dbg[80];
            snprintf(dbg, sizeof(dbg),
                     "[ADC] Source → RF433  Level:%u%%  WD:%u  DID:%08lX",
                     (unsigned)level,
                     (unsigned)wellDry,
                     (unsigned long)RF_GetLastPacketDID());
            HAL_UART_Transmit(
                &huart1,
                (uint8_t *)dbg, (uint16_t)strlen(dbg), 500u);
            HAL_UART_Transmit(
                &huart1,
                (uint8_t *)"\r\n", 2u, 500u);
        }
    }
    else
    {
        /* ── Tier 3: local ADC (offline / bench / fallback) ────────── */
        if (s_lastSrc != ADC_SRC_LOCAL)
        {
            s_lastSrc     = ADC_SRC_LOCAL;
            s_srcChangeTick = HAL_GetTick();

            HAL_UART_Transmit(
                &huart1,
                (uint8_t *)"[ADC] Source → LOCAL ADC (no wireless link)\r\n",
                44u, 500u);
        }
        /* local ADC values from Step 1 are already in data → nothing to do */
    }

    /* ── Step 3: Level-flag events on final voltages ───────────────
     *
     *  Evaluated AFTER wireless injection so events are consistent
     *  regardless of whether the source is LoRa, RF433, or local ADC.
     * ─────────────────────────────────────────────────────────────── */
    for (uint8_t i = 0u; i < ADC_CHANNEL_COUNT; i++)
    {
        float v = data->voltages[i];

        if (i <= 3u)
        {
            if (!s_level_flags[i] && v >= THR)
            {
                s_level_flags[i] = 1u;
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
                            sizeof(loraPacket) - strlen(loraPacket) - 1u);
                    strncat(loraPacket, ";",
                            sizeof(loraPacket) - strlen(loraPacket) - 1u);
                }
            }
            else if (s_level_flags[i] && v < (THR - HYST_DELTA))
            {
                s_level_flags[i] = 0u;
            }
            continue;
        }

        if (i == 4u)   /* Ground water — always local ADC */
        {
            if (!s_level_flags[i] && v >= GROUND_THRESHOLD)
            {
                s_level_flags[i] = 1u;
                snprintf(dataPacketTx, sizeof(dataPacketTx), "@GW#");
                strncat(loraPacket, dataPacketTx,
                        sizeof(loraPacket) - strlen(loraPacket) - 1u);
                strncat(loraPacket, ";",
                        sizeof(loraPacket) - strlen(loraPacket) - 1u);
            }
            else if (s_level_flags[i] && v < (GROUND_THRESHOLD - HYST_DELTA))
            {
                s_level_flags[i] = 0u;
            }
            continue;
        }

        if (i == 5u)   /* Dry-run sensor */
        {
            if (!s_level_flags[i] && v >= DRY_VOLTAGE_THRESHOLD)
            {
                s_level_flags[i] = 1u;
                snprintf(dataPacketTx, sizeof(dataPacketTx), "@DRY#");
                strncat(loraPacket, dataPacketTx,
                        sizeof(loraPacket) - strlen(loraPacket) - 1u);
                strncat(loraPacket, ";",
                        sizeof(loraPacket) - strlen(loraPacket) - 1u);
            }
            else if (s_level_flags[i] && v < (DRY_VOLTAGE_THRESHOLD - HYST_DELTA))
            {
                s_level_flags[i] = 0u;
            }
            continue;
        }
    }
}

/* ── ADC_CheckMaxVoltage (unchanged) ────────────────────────────────── */
uint8_t ADC_CheckMaxVoltage(ADC_Data *data, float threshold)
{
    for (uint8_t i = 0u; i < ADC_CHANNEL_COUNT; i++)
        if (data->voltages[i] >= threshold) return 1u;
    return 0u;
}
