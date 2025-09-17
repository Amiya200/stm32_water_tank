#ifndef MODEL_HANDLE_H
#define MODEL_HANDLE_H

#include <stdint.h>
#include <stdbool.h>

/* === Structures === */
typedef struct {
    bool     active;
    uint32_t onTimeSeconds;
    uint32_t offTimeSeconds;
    bool     executedToday;
} TimerSlot;

typedef struct {
    bool     searchActive;
    uint32_t testingGapSeconds;
    uint32_t dryRunTimeSeconds;
} SearchSettings;

typedef struct {
    bool     twistActive;
    uint32_t onDurationSeconds;
    uint32_t offDurationSeconds;
} TwistSettings;

/* === Globals === */
extern volatile uint8_t  motorStatus;
extern TimerSlot         timerSlots[5];
extern SearchSettings    searchSettings;
extern TwistSettings     twistSettings;

/* === API === */
void ModelHandle_Process(void);
void ModelHandle_ProcessUartCommand(const char* cmd);

uint32_t ModelHandle_TimeToSeconds(uint8_t hh, uint8_t mm);
void ModelHandle_SecondsToTime(uint32_t sec, uint8_t* hh, uint8_t* mm);

void ModelHandle_SetDryRun(bool on);
void ModelHandle_SetOverLoad(bool on);
void ModelHandle_SetOverUnderVolt(bool on);
void ModelHandle_ClearMaxRunFlag(void);

/* ✅ Added for screen.c */
bool Motor_GetStatus(void);

#endif
