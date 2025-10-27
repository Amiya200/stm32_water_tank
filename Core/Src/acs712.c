#include "acs712.h"
#include "math.h"

/* -----------------------------
 * Global Variables (for display / debug)
 * ----------------------------- */
float g_currentA = 0.0f;   // Latest current in Amps
float g_voltageV = 0.0f;   // Latest voltage in Volts

/* -----------------------------
 * Private Variables
 * ----------------------------- */
static ADC_HandleTypeDef *hAdc;
static float zeroOffset = 0.0f;     // learned midpoint (V)
static float lastCurrent = 0.0f;
static float lastVoltage = 0.0f;

/* ---------------------------------------------------------------
 * Helper: read average ADC voltage for a given channel
 * --------------------------------------------------------------- */
static float ReadAverageVoltage(uint32_t channel, uint8_t samples)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
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

/* ---------------------------------------------------------------
 * Init + zero-offset calibration
 * --------------------------------------------------------------- */
void ACS712_Init(ADC_HandleTypeDef *hadc)
{
    hAdc = hadc;
    HAL_Delay(500);   // let voltage settle (~0.5 s)
    ACS712_CalibrateZero();
}

/* ---------------------------------------------------------------
 * Calibrate sensor at 0 A (no load)
 * --------------------------------------------------------------- */
void ACS712_CalibrateZero(void)
{
    const uint16_t samples = 500;
    float sum = 0;
    for (uint16_t i = 0; i < samples; i++) {
        sum += ReadAverageVoltage(ACS712_ADC_CHANNEL, 1);
    }
    zeroOffset = sum / samples;   // midpoint voltage (≈ 2.5 V typical)
}

/* ---------------------------------------------------------------
 * Read current in Amperes (smoothed)
 * --------------------------------------------------------------- */
//float ACS712_ReadCurrent(void)
//{
//    float v_meas = ReadAverageVoltage(ACS712_ADC_CHANNEL, ACS712_NUM_SAMPLES);
//    float dv = v_meas - zeroOffset;
//
//    // remove divider effect, then convert to Amps
//    float current = (dv / VOLT_DIVIDER_RATIO) / ACS712_SENSITIVITY_RAW;
//
//    if (fabsf(current) < ACS712_NOISE_DEADZONE)
//        current = 0.0f;
//
//    lastCurrent = (1.0f - ACS712_FILTER_ALPHA) * lastCurrent + ACS712_FILTER_ALPHA * current;
//    g_currentA = lastCurrent;
//    return lastCurrent;
//}

float ACS712_ReadCurrent(void)
{
    float voltage = ReadAverageVoltage(ACS712_ADC_CHANNEL, ACS712_NUM_SAMPLES);
    float current = (voltage - zeroOffset) / ACS712_SENSITIVITY_RAW;

    // Dead-zone filter
    if (fabsf(current) < ACS712_NOISE_DEADZONE)
        current = 0.0f;

    // Low-pass filter
    lastCurrent = (1.0f - ACS712_FILTER_ALPHA) * lastCurrent +
                  ACS712_FILTER_ALPHA * current;

    g_currentA = lastCurrent;    // ✅ store globally for external access
    return lastCurrent;
}

/* ---------------------------------------------------------------
 * Read input voltage (from divider) in Volts
 * --------------------------------------------------------------- */
float Voltage_ReadInput(void)
{
    float vAdc = ReadAverageVoltage(VOLTAGE_ADC_CHANNEL, 5);   // read scaled ADC voltage
    float vInput = vAdc / VOLT_DIVIDER_RATIO;                  // undo divider (R2/(R1+R2))

    // Optional low-pass filter for stability
    lastVoltage = (1.0f - ACS712_FILTER_ALPHA) * lastVoltage +
                  ACS712_FILTER_ALPHA * vInput;

    g_voltageV = lastVoltage;
    return lastVoltage;
}

/* ---------------------------------------------------------------
 * Combined update (for periodic tasks)
 * --------------------------------------------------------------- */
void ACS712_Update(void)
{
    g_currentA = ACS712_ReadCurrent();
    g_voltageV = Voltage_ReadInput();
}
