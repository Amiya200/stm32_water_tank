#ifndef MODEL_HANDLE_H
#define MODEL_HANDLE_H

#include <stdint.h>
#include <stdbool.h>
#include "adc.h"

/* ===== Timer slot ===== */
typedef struct {
    bool     active;
    uint32_t onTimeSeconds;
    uint32_t offTimeSeconds;
} TimerSlot;

/* ===== Search mode settings ===== */
typedef struct {
    bool     searchActive;
    uint16_t testingGapSeconds;
    uint16_t dryRunTimeSeconds;
} SearchSettings;

/* ===== Twist mode settings ===== */
typedef struct {
    bool     twistActive;
    uint16_t onDurationSeconds;
    uint16_t offDurationSeconds;
} TwistSettings;

/* ===== Public State ===== */
extern volatile uint8_t  motorStatus;

extern volatile bool     countdownActive;
extern volatile bool     countdownMode;
extern volatile uint32_t countdownDuration;

extern TimerSlot         timerSlots[5];
extern SearchSettings    searchSettings;
extern TwistSettings     twistSettings;

extern volatile bool senseDryRun;
extern volatile bool senseOverLoad;
extern volatile bool senseOverUnderVolt;
extern volatile bool senseMaxRunReached;

/* ===== API ===== */
uint32_t ModelHandle_TimeToSeconds(uint8_t hh, uint8_t mm);
void     ModelHandle_SecondsToTime(uint32_t sec, uint8_t* hh, uint8_t* mm);

void ModelHandle_ProcessUartCommand(const char* cmd);
void ModelHandle_Process(void);

/* External setters */
void ModelHandle_SetDryRun(bool on);
void ModelHandle_SetOverLoad(bool on);
void ModelHandle_SetOverUnderVolt(bool on);
void ModelHandle_ClearMaxRunFlag(void);

/* Motor accessors */
bool Motor_GetStatus(void);
extern volatile bool manualOverride;

void ModelHandle_ClearManualOverride(void);
void ModelHandle_SetMotor(bool on);


#endif /* MODEL_HANDLE_H */
