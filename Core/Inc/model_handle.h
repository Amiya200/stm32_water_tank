#ifndef MODEL_HANDLE_H
#define MODEL_HANDLE_H

#include <stdint.h>
#include <stdbool.h>
#include "adc.h"

/* ============================================================
   TIMER SLOT
   ============================================================ */
typedef struct {
    uint8_t enabled;
    uint8_t onHour, onMinute;
    uint8_t offHour, offMinute;
    uint8_t dayMask;     // bit0=Mon ... bit6=Sun
} TimerSlot;

/* ============================================================
   TWIST MODE SETTINGS
   ============================================================ */
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

/* ============================================================
   PUBLIC STATE
   ============================================================ */
extern volatile uint8_t  motorStatus;

/* Mode flags */
extern volatile bool manualActive;
extern volatile bool countdownActive;
extern volatile bool twistActive;
extern volatile bool timerActive;
extern volatile bool semiAutoActive;

/* ===== AUTO MODE FLAGS ===== */
extern volatile bool autoActive;           // NEW auto mode master flag
extern volatile uint16_t auto_retry_counter; // NEW retry counter for LCD

/* Countdown state */
extern volatile bool     countdownMode;
extern volatile uint32_t countdownDuration;

/* Global data */
extern TimerSlot      timerSlots[5];
extern TwistSettings  twistSettings;

/* Protection flags */
extern volatile bool senseDryRun;          // TRUE = water present
extern volatile bool senseOverLoad;
extern volatile bool senseOverUnderVolt;
extern volatile bool senseMaxRunReached;
extern volatile bool manualOverride;

/* ============================================================
   API FUNCTIONS
   ============================================================ */

/* Time helpers */
uint32_t ModelHandle_TimeToSeconds(uint8_t hh, uint8_t mm);
void     ModelHandle_SecondsToTime(uint32_t sec, uint8_t* hh, uint8_t* mm);

/* Main processing loop */
void ModelHandle_Process(void);
void ModelHandle_ProcessUartCommand(const char* cmd);

/* ===== MANUAL MODE ===== */
void ModelHandle_ToggleManual(void);
void ModelHandle_ManualLongPress(void);
void ModelHandle_SetMotor(bool on);
void ModelHandle_ClearManualOverride(void);

/* ===== SEMI AUTO ===== */
void ModelHandle_StartSemiAuto(void);

/* ===== AUTO MODE (NEW) ===== */
void ModelHandle_StartAuto(uint16_t gap_s, uint16_t maxrun_min, uint16_t retry);
void ModelHandle_StopAuto(void);
void ModelHandle_StopAllModesAndMotor(void);   // required by screen.c

/* Motor status */
bool Motor_GetStatus(void);

/* External protection setters */
void ModelHandle_SetDryRun(bool on);
void ModelHandle_SetOverLoad(bool on);
void ModelHandle_SetOverUnderVolt(bool on);
void ModelHandle_ClearMaxRunFlag(void);

/* Timer slot setter */
void ModelHandle_SetTimerSlot(uint8_t slot,
                              uint8_t onH, uint8_t onM,
                              uint8_t offH, uint8_t offM);

/* Internal timer processing */
void ModelHandle_ProcessTimerSlots(void);

#endif /* MODEL_HANDLE_H */
