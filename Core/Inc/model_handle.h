#ifndef MODEL_HANDLE_H
#define MODEL_HANDLE_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// External declaration for UART handle (should be defined in main.c)
extern UART_HandleTypeDef huart1;

// Motor control functions
void Motor_On(void);
void Motor_Off(void);
bool Motor_GetStatus(void);

// UART command processing
void ModelHandle_ProcessReceivedPacket(const char* packet);
void ModelHandle_ProcessUartCommand(const char* command);

// Mode specific handlers
void ModelHandle_ManualMode(const char* command);
void ModelHandle_CountdownMode(const char* command);
void ModelHandle_TimerMode(const char* command);
void ModelHandle_SearchMode(const char* command);
void ModelHandle_TwistMode(const char* command);
void ModelHandle_SemiAutoMode(const char* command);
void ModelHandle_ErrorMode(const char* command);

// Timer and countdown management
void ModelHandle_RunTimers(void);
void ModelHandle_RunCountdown(void);
void ModelHandle_RunSearchLogic(void);
void ModelHandle_RunTwistLogic(void);
void ModelHandle_CheckAllModes(void);

// Utility functions
bool ModelHandle_ParseTime(const char* timeStr, uint8_t* hours, uint8_t* minutes);
bool ModelHandle_ParseDuration(const char* durationStr, uint8_t* minutes, uint8_t* seconds);
uint32_t ModelHandle_TimeToSeconds(uint8_t hours, uint8_t minutes);
uint32_t ModelHandle_DurationToSeconds(uint8_t minutes, uint8_t seconds);
void ModelHandle_SecondsToTime(uint32_t seconds, uint8_t* hours, uint8_t* minutes);
uint8_t ModelHandle_ParseDayBits(const char* daysStr);
bool ModelHandle_IsDayActive(uint8_t dayBitmask, uint8_t currentDay);
uint8_t ModelHandle_GetCurrentDayOfWeek(void);

// Debug output
void ModelHandle_DebugPrint(const char* message);

// Mode state variables (extern for access from other modules)
extern volatile uint8_t motorStatus;
extern volatile bool countdownActive;
extern volatile uint32_t countdownEndTime;
extern volatile uint32_t countdownDuration;
extern volatile bool countdownMode; // true = ON countdown, false = OFF countdown

// Day of week constants
#define SUNDAY    0
#define MONDAY    1
#define TUESDAY   2
#define WEDNESDAY 3
#define THURSDAY  4
#define FRIDAY    5
#define SATURDAY  6

// Timer structure
typedef struct {
    uint8_t slot;
    uint32_t onTimeSeconds;  // Time of day in seconds when to turn ON
    uint32_t offTimeSeconds; // Time of day in seconds when to turn OFF
    bool active;
    bool executedToday;
} TimerSlot;

// Search mode structure
typedef struct {
    uint32_t testingGapSeconds;
    uint32_t dryRunTimeSeconds;
    uint8_t activeDays; // Bitmask: 0b00000001 = Sunday, 0b00000010 = Monday, etc.
    bool searchActive;
    uint32_t lastTestTime;
} SearchSettings;

// Twist mode structure
typedef struct {
    uint32_t onDurationSeconds;
    uint32_t offDurationSeconds;
    uint32_t startTimeSeconds;
    uint32_t endTimeSeconds;
    uint8_t activeDays; // Bitmask
    bool twistActive;
    bool currentState; // true = ON, false = OFF
    uint32_t stateChangeTime;
} TwistSettings;

extern TimerSlot timerSlots[5];
extern SearchSettings searchSettings;
extern TwistSettings twistSettings;

#endif // MODEL_HANDLE_H
