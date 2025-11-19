#ifndef MODEL_HANDLE_H
#define MODEL_HANDLE_H

#include <stdint.h>
#include <stdbool.h>
#include "adc.h"

/* ===== Timer slot ===== */
typedef struct {
    uint8_t enabled;
    uint8_t onHour, onMinute;
    uint8_t offHour, offMinute;
    uint8_t dayMask;     // bit0=Mon ... bit6=Sun
} TimerSlot;


/* ===== Search mode settings ===== */
typedef struct {
    uint16_t gapSeconds;
    uint16_t probeSeconds;
    uint8_t onHour;
    uint8_t onMinute;
    uint8_t offHour;
    uint8_t offMinute;
    bool dayEnabled[7];
    bool searchActive;
} SearchSettings;

/* ===== Twist mode settings ===== */
typedef struct {
    uint16_t onDurationSeconds;
    uint16_t offDurationSeconds;

    uint8_t  onHour;
    uint8_t  onMinute;
    uint8_t  offHour;
    uint8_t  offMinute;

    bool     twistActive;
    bool     twistArmed;     // NEW → waiting for ON time
} TwistSettings;

typedef struct {
    uint16_t gap_seconds;     // dry gap between retries
    uint16_t maxrun_minutes;  // max continuous run
    uint16_t retry_limit;     // 0 = infinite
} AutoSettings;

static AutoSettings autoSettings = {
    .gap_seconds = 10,
    .maxrun_minutes = 120,
    .retry_limit = 0
};

/* ===== Public State ===== */
extern volatile uint8_t  motorStatus;

/* Mode activity flags */
extern volatile bool manualActive;
extern volatile bool countdownActive;
extern volatile bool twistActive;
extern volatile bool searchActive;
extern volatile bool timerActive;
extern volatile bool semiAutoActive;

/* Countdown state */
extern volatile bool     countdownMode;
extern volatile uint32_t countdownDuration;

/* Global settings */
extern TimerSlot         timerSlots[5];
extern SearchSettings    searchSettings;
extern TwistSettings     twistSettings;

/* Protections */
extern volatile bool senseDryRun;
extern volatile bool senseOverLoad;
extern volatile bool senseOverUnderVolt;
extern volatile bool senseMaxRunReached;
extern volatile bool manualOverride;

/* ===== API ===== */
/* Time helpers */
uint32_t ModelHandle_TimeToSeconds(uint8_t hh, uint8_t mm);
void     ModelHandle_SecondsToTime(uint32_t sec, uint8_t* hh, uint8_t* mm);

/* Core processing */
void ModelHandle_Process(void);
void ModelHandle_ProcessUartCommand(const char* cmd);

/* === Manual Mode === */
void ModelHandle_ToggleManual(void);       // short press toggle
void ModelHandle_ManualLongPress(void);    // long press restart
void ModelHandle_SetMotor(bool on);        // force ON/OFF
void ModelHandle_ClearManualOverride(void);


void ModelHandle_SetMotor(bool on);
void ModelHandle_ClearManualOverride(void);



// === Burst control for extra relays (R2, R3) ===
void ModelHandle_TriggerAuxBurst(uint16_t seconds);
bool ModelHandle_AuxBurstActive(void);


/* === Semi-Auto Mode === */
void ModelHandle_StartSemiAuto(void);

/* Motor status */
bool Motor_GetStatus(void);

/* External setters for protections */
void ModelHandle_SetDryRun(bool on);
void ModelHandle_SetOverLoad(bool on);
void ModelHandle_SetOverUnderVolt(bool on);
void ModelHandle_ClearMaxRunFlag(void);

void ModelHandle_SetTimerSlot(uint8_t slot,
                              uint8_t onH, uint8_t onM,
                              uint8_t offH, uint8_t offM);

void ModelHandle_ProcessTimerSlots(void);

#endif /* MODEL_HANDLE_H */
