// acs712.c — fast + accurate ACS712 with one-shot gain calibration
#include "acs712.h"
#include "math.h"
#include "stm32f1xx_hal.h"

/* -----------------------------
 * Global (for display / debug)
 * ----------------------------- */
float g_currentA = 0.0f;   // Final filtered current (A)
float g_voltageV = 0.0f;   // Optional input voltage (V)

/* -----------------------------
 * Private state
 * ----------------------------- */
static ADC_HandleTypeDef *s_hadc = NULL;

/* Zero-current midpoint and gain scaling */
static float s_zeroOffset_V   = 0.0f;  // learned midpoint (V)
static float s_gain_A_per_A   = 1.0f;  // multiplicative scale to match meter

/* Filters/state */
static float s_lastCurrent_A  = 0.0f;

/* ===== Tunables (speed vs smoothness) =====
   Use FAST profile: quick, responsive display (~100–150 ms feel) */
#define MEDIAN_READS            3       // median-of-3 for spike kill
#define POLL_SAMPLES            1       // per ADC average when polling internally
#define ADC_POLL_SAMPLETIME     ADC_SAMPLETIME_41CYCLES_5

/* IIR RMS (on squared current) and final EMA */
#define IIR_RMS_BETA            0.25f   // higher = faster RMS (0.2–0.35 is good)
#define EMA_ALPHA_FINAL         0.28f   // higher = snappier (0.2–0.35)
#define ZERO_TRACK_ALPHA        0.0015f // very slow drift tracking
#define ZERO_TRACK_THRESH_A     0.15f   // track offset only near 0 A
#define NOISE_DEADZONE_A        0.03f   // clamp tiny residuals

/* IIR RMS accumulator */
static float s_rms2_iir = 0.0f;

/* External feed path (preferred if you already read the channel elsewhere) */
static uint8_t s_haveExtFeed   = 0;
static float   s_extVadcVolts  = 0.0f;

/* ============ Helpers ============ */
static float median3(float a, float b, float c)
{
    // branchless median-of-3
    float ab = (a + b - fabsf(a - b)) * 0.5f; // min(a,b)
    float AB = (a + b + fabsf(a - b)) * 0.5f; // max(a,b)
    float m  = (AB + c - fabsf(AB - c)) * 0.5f; // min(max(a,b), c)
    // max(min(a,b), min(max(a,b), c))
    return (ab + m + fabsf(ab - m)) * 0.5f;
}

static float ReadAverageVoltage(uint32_t channel, uint8_t samples)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_POLL_SAMPLETIME;
    HAL_ADC_ConfigChannel(s_hadc, &sConfig);

    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
        HAL_ADC_Start(s_hadc);
        HAL_ADC_PollForConversion(s_hadc, HAL_MAX_DELAY);
        sum += HAL_ADC_GetValue(s_hadc);
        HAL_ADC_Stop(s_hadc);
    }
    float avg = (float)sum / samples;
    return (avg * ACS712_VREF_ADC) / ACS712_ADC_RESOLUTION;  // Volts
}

static float instant_current_A_fromV(float v_adc)
{
    float dv = v_adc - s_zeroOffset_V;
    float iA = (dv / ACS712_SENSITIVITY_RAW) * s_gain_A_per_A;  // <-- your formula + gain

    // slow zero tracking only when near zero current
    if (fabsf(iA) < ZERO_TRACK_THRESH_A) {
        s_zeroOffset_V = (1.0f - ZERO_TRACK_ALPHA)*s_zeroOffset_V + ZERO_TRACK_ALPHA*v_adc;
        // don’t force to zero here; let the filters handle it
    }
    return iA;
}

/* ============ Public API ============ */
void ACS712_Init(ADC_HandleTypeDef *hadc)
{
    s_hadc = hadc;
    HAL_Delay(300);
    ACS712_CalibrateZero();
    s_gain_A_per_A = 1.0f;
    s_rms2_iir     = 0.0f;
    s_lastCurrent_A= 0.0f;
    g_currentA     = 0.0f;
}

void ACS712_CalibrateZero(void)
{
    // No-load calibration
    const uint16_t samples = 300;
    float sum = 0.0f;
    for (uint16_t i = 0; i < samples; i++) {
#ifdef ACS712_ADC_CHANNEL
        float v = s_haveExtFeed ? s_extVadcVolts : ReadAverageVoltage(ACS712_ADC_CHANNEL, POLL_SAMPLES);
#else
        float v = s_extVadcVolts;
#endif
        sum += v;
    }
    s_zeroOffset_V = sum / samples;

    // reset filters
    s_rms2_iir      = 0.0f;
    s_lastCurrent_A = 0.0f;
    g_currentA      = 0.0f;
}

/* One-shot gain calibration:
   Call while a known, steady current is flowing (e.g., 3.50 A).
   The driver measures raw current now and sets gain so it matches the known value. */
void ACS712_CalibrateGain(float known_current_A)
{
    if (known_current_A <= 0.0f) return;

    // Take a quick median-of-3 voltage
#ifdef ACS712_ADC_CHANNEL
    float v0 = s_haveExtFeed ? s_extVadcVolts : ReadAverageVoltage(ACS712_ADC_CHANNEL, POLL_SAMPLES);
    float v1 = s_haveExtFeed ? s_extVadcVolts : ReadAverageVoltage(ACS712_ADC_CHANNEL, POLL_SAMPLES);
    float v2 = s_haveExtFeed ? s_extVadcVolts : ReadAverageVoltage(ACS712_ADC_CHANNEL, POLL_SAMPLES);
#else
    float v0 = s_extVadcVolts, v1 = s_extVadcVolts, v2 = s_extVadcVolts;
#endif
    float v_med = median3(v0, v1, v2);

    // Compute instantaneous (pre-gain) using current gain=1.0 baseline
    float dv      = v_med - s_zeroOffset_V;
    float i_pre   = dv / ACS712_SENSITIVITY_RAW;
    float abs_pre = fabsf(i_pre);

    // Protect against division by near-zero
    if (abs_pre < 0.05f) return;

    s_gain_A_per_A = known_current_A / i_pre;

    // Nudge the filters to the calibrated value quickly
    s_rms2_iir      = known_current_A * known_current_A;
    s_lastCurrent_A = known_current_A;
    g_currentA      = known_current_A;
}

/* If you already read the ACS channel elsewhere (scan/DMA/ISR), feed its VOLTAGE here. */
void ACS712_FeedAdcVoltage(float v_adc_volts)
{
    s_extVadcVolts = v_adc_volts;
    s_haveExtFeed  = 1;
}

/* Fast, low-latency current read */
float ACS712_ReadCurrent(void)
{
#ifdef ACS712_ADC_CHANNEL
    float v0 = s_haveExtFeed ? s_extVadcVolts : ReadAverageVoltage(ACS712_ADC_CHANNEL, POLL_SAMPLES);
    float v1 = s_haveExtFeed ? s_extVadcVolts : ReadAverageVoltage(ACS712_ADC_CHANNEL, POLL_SAMPLES);
    float v2 = s_haveExtFeed ? s_extVadcVolts : ReadAverageVoltage(ACS712_ADC_CHANNEL, POLL_SAMPLES);
#else
    float v0 = s_extVadcVolts, v1 = s_extVadcVolts, v2 = s_extVadcVolts;
#endif
    float v_med = median3(v0, v1, v2);

    // Instantaneous A (formula + gain)
    float i_inst = instant_current_A_fromV(v_med);

    // IIR RMS on squared current (fast & stable), then light EMA
    s_rms2_iir = (1.0f - IIR_RMS_BETA) * s_rms2_iir + IIR_RMS_BETA * (i_inst * i_inst);
    float i_rms = sqrtf(s_rms2_iir);

    float i_out = (1.0f - EMA_ALPHA_FINAL) * s_lastCurrent_A + EMA_ALPHA_FINAL * i_rms;

    // Dead-zone clamp for tiny residuals
    if (fabsf(i_out) < NOISE_DEADZONE_A) i_out = 0.0f;

    s_lastCurrent_A = i_out;
    g_currentA      = i_out;
    return i_out;
}

/* Optional voltage divider read */
float Voltage_ReadInput(void)
{
#ifdef VOLTAGE_ADC_CHANNEL
    float vAdc = ReadAverageVoltage(VOLTAGE_ADC_CHANNEL, 3);
    float vIn  = vAdc / VOLT_DIVIDER_RATIO;
    // Light EMA reuse
    g_voltageV = (1.0f - EMA_ALPHA_FINAL) * g_voltageV + EMA_ALPHA_FINAL * vIn;
    return g_voltageV;
#else
    return 0.0f;
#endif
}

void ACS712_Update(void)
{
    g_currentA = ACS712_ReadCurrent();
#ifdef VOLTAGE_ADC_CHANNEL
    g_voltageV = Voltage_ReadInput();
#endif
}
