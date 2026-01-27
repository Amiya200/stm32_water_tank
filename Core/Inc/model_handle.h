#ifndef MODEL_HANDLE_H
#define MODEL_HANDLE_H

#include <stdint.h>
#include <stdbool.h>
#include "adc.h"
#include "eeprom_i2c.h" // Assuming this file is for EEPROM handling

/* ============================================================
   TIMER SLOT
   ============================================================ */
typedef struct __attribute__((packed)) {
    uint8_t enabled;
    uint8_t onHour;
    uint8_t onMinute;
    uint8_t offHour;
    uint8_t offMinute;
    uint8_t dayMask;
    uint16_t gapMinutes;
} TimerSlot;

/* ============================================================
   SYSTEM SETTINGS STRUCT
   ============================================================ */
typedef struct {
    uint16_t gap_time_s;
    uint8_t  retry_count;
    uint16_t uv_limit;
    uint16_t ov_limit;
    int16_t  overload;    // ✅ Amperes
    int16_t  underload;   // ✅ Amperes
    uint16_t maxrun_min;
} SystemSettings;

/* ❗ Only declare here — DO NOT DEFINE */
extern SystemSettings sys;

/* ============================================================
   EXTERNAL GLOBALS
   ============================================================ */
extern TimerSlot timerSlots[5];

typedef struct {
    uint16_t onDurationSeconds;
    uint16_t offDurationSeconds;
    uint8_t  onHour;
    uint8_t  onMinute;
    uint8_t  offHour;
    uint8_t  offMinute;
    bool     twistActive;
    bool     twistArmed;
} TwistSettings;

extern volatile uint8_t  motorStatus;
extern volatile bool manualActive;
extern volatile bool countdownActive;
extern volatile bool twistActive;
extern volatile bool timerActive;
extern volatile bool semiAutoActive;
extern volatile bool autoActive;
extern volatile uint16_t auto_retry_counter;
extern volatile bool countdownMode;
extern volatile uint32_t countdownDuration;
extern TwistSettings  twistSettings;

/* Protection flags */
extern volatile bool senseDryRun;
extern volatile bool senseOverLoad;
extern volatile bool senseOverUnderVolt;
extern volatile bool senseMaxRunReached;
extern volatile bool manualOverride;

/* ============================================================
   API FUNCTIONS
   ============================================================ */

/* Time Conversion Functions */
uint32_t ModelHandle_TimeToSeconds(uint8_t hh, uint8_t mm);
void     ModelHandle_SecondsToTime(uint32_t sec, uint8_t* hh, uint8_t* mm);

/* Timer Functions */
void ModelHandle_SaveTimerToEEPROM(void);             // Save timer slots to EEPROM
void ModelHandle_LoadTimerFromEEPROM(void);           // Load timer slots from EEPROM
void ModelHandle_UpdateTimerFromScreen(void);         // Update timer from screen UI
void ModelHandle_SetTimerSlot(uint8_t slot, uint8_t onH, uint8_t onM, uint8_t offH, uint8_t offM); // Set timer slot values
void ModelHandle_ProcessTimerSlots(void);             // Process timer slots (e.g., check if active)

/* Manual Mode Functions */
void ModelHandle_ToggleManual(void);
void ModelHandle_ManualLongPress(void);
void ModelHandle_SetMotor(bool on);
void ModelHandle_ClearManualOverride(void);
void Timer_EEPROM_EnsureValid(void);

/* Semi-auto Mode */
void ModelHandle_StartSemiAuto(void);

/* Auto Mode */
void ModelHandle_StartAuto(uint16_t gap_s, uint16_t maxrun_min, uint16_t retry);
void ModelHandle_StopAuto(void);
void ModelHandle_StopAllModesAndMotor(void);

/* Motor Control */
bool Motor_GetStatus(void);
bool ModelHandle_IsOverload(void);
bool ModelHandle_IsUnderload(void);
bool ModelHandle_IsDryRunActive(void);
bool ModelHandle_IsVoltageFault(void);
bool ModelHandle_IsMaxRunReached(void);

/* Protection Functions */
void ModelHandle_SetDryRun(bool on);
void ModelHandle_SetOverLoad(bool on);
void ModelHandle_SetOverUnderVolt(bool on);
void ModelHandle_ClearMaxRunFlag(void);

/* Settings Functions */
void ModelHandle_SetUserSettings(uint16_t gap_s,  uint8_t retry, uint16_t uv_limit, uint16_t ov_limit, int16_t overload, int16_t underload ,uint16_t maxrun_min);
void ModelHandle_SetAutoSettings(uint16_t gap_s, uint16_t maxrun_min, uint8_t retry);
void ModelHandle_OnPowerUp(void);

/* Load and Save Settings */
void ModelHandle_LoadSettingsFromEEPROM(void);       // Load system settings from EEPROM
void ModelHandle_SaveSettingsToEEPROM(void);         // Save system settings to EEPROM

/* Timer Functions */
void ModelHandle_LoadTimerState(void);               // Load the state of timers from EEPROM
void ModelHandle_SaveTimerState(void);               // Save timer state to EEPROM

#endif /* MODEL_HANDLE_H */
