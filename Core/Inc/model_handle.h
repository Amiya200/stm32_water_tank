#ifndef MODEL_HANDLE_H
#define MODEL_HANDLE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t onTimeSeconds;   // seconds since midnight
    uint32_t offTimeSeconds;  // seconds since midnight
    bool     active;
    bool     executedToday;
} TimerSlot;

typedef struct {
    bool     searchActive;
    uint32_t testingGapSeconds;  // time between tests
    uint32_t dryRunTimeSeconds;  // short test duration
} SearchSettings;

typedef struct {
    bool     twistActive;
    uint32_t onDurationSeconds;
    uint32_t offDurationSeconds;
} TwistSettings;

/* Public state used by UI or other modules */
extern volatile uint8_t  motorStatus;         // 0/1
extern volatile bool     countdownActive;
extern volatile bool     countdownMode;       // true => ON countdown
extern volatile uint32_t countdownDuration;   // remaining seconds

extern TimerSlot       timerSlots[5];
extern SearchSettings  searchSettings;
extern TwistSettings   twistSettings;

/* API */
uint32_t ModelHandle_TimeToSeconds(uint8_t hh, uint8_t mm);
void     ModelHandle_SecondsToTime(uint32_t sec, uint8_t* hh, uint8_t* mm);

void ModelHandle_ProcessUartCommand(const char* cmd);
void ModelHandle_Process(void);

/* status / flags */
bool Motor_GetStatus(void);

/* protection flag setters (call from ADC/LoRa layer) */
void ModelHandle_SetDryRun(bool on);
void ModelHandle_SetOverLoad(bool on);
void ModelHandle_SetOverUnderVolt(bool on);
void ModelHandle_ClearMaxRunFlag(void);
/* Alias for legacy main.c call site */
void ModelHandle_ProcessReceivedPacket(const char* pkt);

/* optional debug */
void ModelHandle_DebugPrint(const char* msg);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_HANDLE_H */
