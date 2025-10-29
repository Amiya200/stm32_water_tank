// acs712.c  — noise-hardened current read

#include "acs712.h"
#include "math.h"

/* -----------------------------
 * Global (for display / debug)
 * ----------------------------- */
float g_currentA = 0.0f;   // Latest filtered current (A)
float g_voltageV = 0.0f;   // Latest filtered input voltage (V)

/* -----------------------------
 * Private
 * ----------------------------- */
static ADC_HandleTypeDef *hAdc;
static float zeroOffset = 0.0f;     // learned midpoint (V)

static float lastCurrent = 0.0f;
static float lastVoltage = 0.0f;

/* ===== Tuning knobs (adjust if needed) ===== */
#ifndef ACS712_NUM_SAMPLES
#define ACS712_NUM_SAMPLES       10     // per call to ReadAverageVoltage
#endif
#define SAMPLING_TIME_ACS        ADC_SAMPLETIME_239CYCLES_5

#define EMA_ALPHA_FINAL          0.12f  // final light smoothing
#define ZERO_TRACK_ALPHA         0.002f // VERY slow—only near zero
#define ZERO_TRACK_THRESH_A      0.20f  // track offset if |I| < 0.20 A
#define NOISE_DEADZONE_A         0.08f  // clamp tiny residuals to 0

/* RMS window: compute RMS over last M instantaneous samples */
#define RMS_BUF_LEN              64     // ~50–100 is fine
static float s_instBuf[RMS_BUF_LEN];
static uint16_t s_instHead = 0;
static uint16_t s_instCount = 0;

/* Median-of-5 helper */
static float median5(float a, float b, float c, float d, float e)
{
    float x[5] = {a,b,c,d,e};
    // simple insertion sort (tiny)
    for (int i=1;i<5;i++){
        float t = x[i]; int j = i-1;
        while (j>=0 && x[j] > t){ x[j+1]=x[j]; j--; }
        x[j+1]=t;
    }
    return x[2];
}

/* Read average ADC voltage for a given channel (with long sample time for SNR) */
static float ReadAverageVoltage(uint32_t channel, uint8_t samples)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = SAMPLING_TIME_ACS;  // longer sampling → better SNR
    HAL_ADC_ConfigChannel(hAdc, &sConfig);

    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
        HAL_ADC_Start(hAdc);
        HAL_ADC_PollForConversion(hAdc, HAL_MAX_DELAY);
        sum += HAL_ADC_GetValue(hAdc);
        HAL_ADC_Stop(hAdc);
    }

    float avg = (float)sum / samples;
    return (avg * ACS712_VREF_ADC) / ACS712_ADC_RESOLUTION;  // in Volts
}

/* Init + zero calibration */
void ACS712_Init(ADC_HandleTypeDef *hadc)
{
    hAdc = hadc;
    HAL_Delay(500);
    ACS712_CalibrateZero();
}

/* Calibrate sensor at 0 A (no load) */
void ACS712_CalibrateZero(void)
{
    const uint16_t samples = 500;
    float sum = 0;
    for (uint16_t i = 0; i < samples; i++) {
        sum += ReadAverageVoltage(ACS712_ADC_CHANNEL, 1);
    }
    zeroOffset    = sum / samples;
    s_instHead    = 0;
    s_instCount   = 0;
    g_currentA    = 0.0f;
    lastCurrent   = 0.0f;
}

/* --- One instantaneous current sample (Volts → Amps, no heavy filtering) --- */
static float sample_current_instant_A(void)
{
    /* 5 quick reads → median to kill spikes */
    float v0 = ReadAverageVoltage(ACS712_ADC_CHANNEL, 1);
    float v1 = ReadAverageVoltage(ACS712_ADC_CHANNEL, 1);
    float v2 = ReadAverageVoltage(ACS712_ADC_CHANNEL, 1);
    float v3 = ReadAverageVoltage(ACS712_ADC_CHANNEL, 1);
    float v4 = ReadAverageVoltage(ACS712_ADC_CHANNEL, 1);
    float v  = median5(v0, v1, v2, v3, v4);

    float dv = v - zeroOffset;
    float iA = dv / ACS712_SENSITIVITY_RAW; // Volts per Amp constant from your header

    /* Slow zero tracking ONLY when near zero current */
    if (fabsf(iA) < ZERO_TRACK_THRESH_A) {
        zeroOffset = (1.0f - ZERO_TRACK_ALPHA) * zeroOffset + ZERO_TRACK_ALPHA * v;
        iA = 0.0f;  // snub tiny wiggle during tracking phase
    }
    return iA;
}

/* --- Push into RMS buffer and return windowed RMS --- */
static float rms_window_update(float instA)
{
    s_instBuf[s_instHead] = instA;
    s_instHead = (uint16_t)((s_instHead + 1) % RMS_BUF_LEN);
    if (s_instCount < RMS_BUF_LEN) s_instCount++;

    /* Compute RMS over window (cheap loop; RMS_BUF_LEN is small) */
    float sumsq = 0.0f;
    for (uint16_t k=0; k<s_instCount; k++) sumsq += s_instBuf[k]*s_instBuf[k];
    float rms = sqrtf(sumsq / (float)s_instCount);

    /* Preserve sign like a rectified meter? For AC, RMS is positive.
       If you want signed DC too, mix: sign from latest sample. */
    (void)instA;
    return rms;
}

/* Read current in Amperes (robust, low-noise) */
float ACS712_ReadCurrent(void)
{
    /* Instant sample with spike suppression + offset care */
    float i_inst = sample_current_instant_A();

    /* Windowed RMS tames mains ripple & jitter */
    float i_rms = rms_window_update(i_inst);

    /* Final light EMA */
    float i_out = (1.0f - EMA_ALPHA_FINAL) * lastCurrent + EMA_ALPHA_FINAL * i_rms;

    /* Dead-zone clamp */
    if (fabsf(i_out) < NOISE_DEADZONE_A) i_out = 0.0f;

    lastCurrent = i_out;
    g_currentA  = i_out;
    return i_out;
}

/* Read input voltage (keep your existing divider math but add small EMA) */
float Voltage_ReadInput(void)
{
    float vAdc = ReadAverageVoltage(VOLTAGE_ADC_CHANNEL, 5);
    float vIn  = vAdc / VOLT_DIVIDER_RATIO;

    lastVoltage = (1.0f - EMA_ALPHA_FINAL) * lastVoltage + EMA_ALPHA_FINAL * vIn;
    g_voltageV  = lastVoltage;
    return lastVoltage;
}

/* Combined update */
void ACS712_Update(void)
{
    g_currentA = ACS712_ReadCurrent();
    g_voltageV = Voltage_ReadInput();
}



//// okk ////
