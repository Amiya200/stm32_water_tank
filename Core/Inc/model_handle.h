#ifndef MODEL_HANDLE_H
#define MODEL_HANDLE_H

#include <stdint.h>
#include <stdbool.h>
#include "adc.h"

/* ============================================================
   TIMER SLOT
   ============================================================ */
typedef struct {
    bool enabled;
    uint8_t onHour;
    uint8_t onMinute;
    uint8_t offHour;
    uint8_t offMinute;
    uint8_t dayMask;
} TimerSlot;

/* ============================================================
   SYSTEM SETTINGS STRUCT
   ============================================================ */
typedef struct {
    uint16_t gap_time_s;
    uint8_t  retry_count;
    uint16_t uv_limit;
    uint16_t ov_limit;
    float    overload;
    float    underload;
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
uint32_t ModelHandle_TimeToSeconds(uint8_t hh, uint8_t mm);
void     ModelHandle_SecondsToTime(uint32_t sec, uint8_t* hh, uint8_t* mm);
void ModelHandle_Process(void);
void ModelHandle_ProcessUartCommand(const char* cmd);

/* Manual */
void ModelHandle_ToggleManual(void);
void ModelHandle_ManualLongPress(void);
void ModelHandle_SetMotor(bool on);
void ModelHandle_ClearManualOverride(void);

/* Semi-auto */
void ModelHandle_StartSemiAuto(void);

/* Auto Mode */
void ModelHandle_StartAuto(uint16_t gap_s, uint16_t maxrun_min, uint16_t retry);
void ModelHandle_StopAuto(void);
void ModelHandle_StopAllModesAndMotor(void);

/* Motor */
bool Motor_GetStatus(void);

/* Protections */
void ModelHandle_SetDryRun(bool on);
void ModelHandle_SetOverLoad(bool on);
void ModelHandle_SetOverUnderVolt(bool on);
void ModelHandle_ClearMaxRunFlag(void);

/* Timer */
void ModelHandle_SetTimerSlot(uint8_t slot, uint8_t onH, uint8_t onM,
                              uint8_t offH, uint8_t offM);
void ModelHandle_ProcessTimerSlots(void);

#endif /* MODEL_HANDLE_H */
