#include "model_handle.h"
#include "global.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// External UART handle (defined in main.c)
extern UART_HandleTypeDef huart1;

// Global variables

volatile uint8_t motorStatus = 0; // 0: Off, 1: On

volatile bool countdownActive = false;
volatile uint32_t countdownEndTime = 0;
volatile uint32_t countdownDuration = 0;
volatile bool countdownMode = false; // false = OFF countdown, true = ON countdown

// Timer slots
TimerSlot timerSlots[5] = {
    {1, 0, 0, false, false},
    {2, 0, 0, false, false},
    {3, 0, 0, false, false},
    {4, 0, 0, false, false},
    {5, 0, 0, false, false}
};

// Search mode settings
SearchSettings searchSettings = {
    .testingGapSeconds = 0,
    .dryRunTimeSeconds = 0,
    .activeDays = 0,
    .searchActive = false,
    .lastTestTime = 0
};

// Twist mode settings
TwistSettings twistSettings = {
    .onDurationSeconds = 0,
    .offDurationSeconds = 0,
    .startTimeSeconds = 0,
    .endTimeSeconds = 0,
    .activeDays = 0,
    .twistActive = false,
    .currentState = false,
    .stateChangeTime = 0
};

// Motor control functions
void Motor_On(void) {
    motorStatus = 1;
    // Add hardware-specific code to turn motor ON
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // Example GPIO
    ModelHandle_DebugPrint("Motor turned ON");
}

void Motor_Off(void) {
    motorStatus = 0;
    // Add hardware-specific code to turn motor OFF
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // Example GPIO
    ModelHandle_DebugPrint("Motor turned OFF");
}

bool Motor_GetStatus(void) {
    return motorStatus;
}

// Command processing
void ModelHandle_ProcessReceivedPacket(const char* packet) {
    if (packet == NULL || strlen(packet) == 0) {
        return;
    }

    ModelHandle_DebugPrint("Received packet:");
    ModelHandle_DebugPrint(packet);

    ModelHandle_ProcessUartCommand(packet);
}

void ModelHandle_ProcessUartCommand(const char* command) {
    if (command == NULL) return;

    // Manual mode commands
    if (strcmp(command, "MOTOR_ON") == 0) {
        ModelHandle_ManualMode(command);
    }
    else if (strcmp(command, "MOTOR_OFF") == 0) {
        ModelHandle_ManualMode(command);
    }
    // Countdown mode
    else if (strncmp(command, "COUNTDOWN_ON:", 13) == 0) {
        ModelHandle_CountdownMode(command);
    }
    else if (strncmp(command, "COUNTDOWN_OFF:", 14) == 0) {
        ModelHandle_CountdownMode(command);
    }
    else if (strcmp(command, "COUNTDOWN_END_ON") == 0 ||
             strcmp(command, "COUNTDOWN_END_OFF") == 0) {
        ModelHandle_CountdownMode(command);
    }
    // Timer mode
    else if (strcmp(command, "TIMER_CLEAR") == 0) {
        ModelHandle_TimerMode(command);
    }
    else if (strncmp(command, "TIMER_SET:", 10) == 0) {
        ModelHandle_TimerMode(command);
    }
    // Search mode
    else if (strncmp(command, "SEARCH_GAP:", 11) == 0) {
        ModelHandle_SearchMode(command);
    }
    else if (strncmp(command, "SEARCH_DRYRUN:", 14) == 0) {
        ModelHandle_SearchMode(command);
    }
    else if (strncmp(command, "SEARCH_DAYS:", 12) == 0) {
        ModelHandle_SearchMode(command);
    }
    // Twist mode
    else if (strncmp(command, "TWIST_ONDUR:", 12) == 0) {
        ModelHandle_TwistMode(command);
    }
    else if (strncmp(command, "TWIST_OFFDUR:", 13) == 0) {
        ModelHandle_TwistMode(command);
    }
    else if (strncmp(command, "TWIST_ONTIME:", 13) == 0) {
        ModelHandle_TwistMode(command);
    }
    else if (strncmp(command, "TWIST_OFFTIME:", 14) == 0) {
        ModelHandle_TwistMode(command);
    }
    else if (strncmp(command, "TWIST_DAYS:", 11) == 0) {
        ModelHandle_TwistMode(command);
    }
    // Semi-auto mode
    else if (strcmp(command, "SEMI_ON") == 0) {
        ModelHandle_SemiAutoMode(command);
    }
    else if (strcmp(command, "SEMI_OFF") == 0) {
        ModelHandle_SemiAutoMode(command);
    }
    // Error mode
    else if (strncmp(command, "ERROR_MSG:", 10) == 0) {
        ModelHandle_ErrorMode(command);
    }
    else {
        char debugMsg[50];
        snprintf(debugMsg, sizeof(debugMsg), "Unknown command: %s", command);
        ModelHandle_DebugPrint(debugMsg);
    }
}

// Mode specific handlers
void ModelHandle_ManualMode(const char* command) {
    if (strcmp(command, "MOTOR_ON") == 0) {
        Motor_On();
    }
    else if (strcmp(command, "MOTOR_OFF") == 0) {
        Motor_Off();
    }
}

void ModelHandle_CountdownMode(const char* command) {
    if (strncmp(command, "COUNTDOWN_ON:", 13) == 0) {
        int duration;
        if (sscanf(command + 13, "%d", &duration) == 1) {
            countdownDuration = duration * 60; // Convert minutes to seconds
            countdownEndTime = HAL_GetTick() / 1000 + countdownDuration;
            countdownActive = true;
            countdownMode = true; // ON countdown
            Motor_On();

            char debugMsg[50];
            snprintf(debugMsg, sizeof(debugMsg), "Countdown ON started: %d min", duration);
            ModelHandle_DebugPrint(debugMsg);
        }
    }
    else if (strncmp(command, "COUNTDOWN_OFF:", 14) == 0) {
        int duration;
        if (sscanf(command + 14, "%d", &duration) == 1) {
            countdownDuration = duration * 60; // Convert minutes to seconds
            countdownEndTime = HAL_GetTick() / 1000 + countdownDuration;
            countdownActive = true;
            countdownMode = false; // OFF countdown
            Motor_Off();

            char debugMsg[50];
            snprintf(debugMsg, sizeof(debugMsg), "Countdown OFF started: %d min", duration);
            ModelHandle_DebugPrint(debugMsg);
        }
    }
    else if (strcmp(command, "COUNTDOWN_END_ON") == 0) {
        countdownActive = false;
        Motor_On();
        ModelHandle_DebugPrint("Countdown ended - Motor ON");
    }
    else if (strcmp(command, "COUNTDOWN_END_OFF") == 0) {
        countdownActive = false;
        Motor_Off();
        ModelHandle_DebugPrint("Countdown ended - Motor OFF");
    }
}

void ModelHandle_TimerMode(const char* command) {
    if (strcmp(command, "TIMER_CLEAR") == 0) {
        // Clear all timer slots
        for (int i = 0; i < 5; i++) {
            timerSlots[i].active = false;
            timerSlots[i].executedToday = false;
        }
        ModelHandle_DebugPrint("All timers cleared");
    }
    else if (strncmp(command, "TIMER_SET:", 10) == 0) {
        int slot;
        char onTime[6], offTime[6];

        if (sscanf(command + 10, "%d:%5[^:]:%5s", &slot, onTime, offTime) == 3) {
            if (slot >= 1 && slot <= 5) {
                uint8_t onHours, onMinutes, offHours, offMinutes;

                if (ModelHandle_ParseTime(onTime, &onHours, &onMinutes) &&
                    ModelHandle_ParseTime(offTime, &offHours, &offMinutes)) {

                    timerSlots[slot-1].onTimeSeconds = ModelHandle_TimeToSeconds(onHours, onMinutes);
                    timerSlots[slot-1].offTimeSeconds = ModelHandle_TimeToSeconds(offHours, offMinutes);
                    timerSlots[slot-1].active = true;
                    timerSlots[slot-1].executedToday = false;

                    char debugMsg[100];
                    snprintf(debugMsg, sizeof(debugMsg), "Timer slot %d: ON@%s, OFF@%s",
                            slot, onTime, offTime);
                    ModelHandle_DebugPrint(debugMsg);
                }
            }
        }
    }
}

void ModelHandle_SearchMode(const char* command) {
    if (strncmp(command, "SEARCH_GAP:", 11) == 0) {
        uint8_t gapMin, gapSec;
        if (ModelHandle_ParseDuration(command + 11, &gapMin, &gapSec)) {
            searchSettings.testingGapSeconds = ModelHandle_DurationToSeconds(gapMin, gapSec);
            char debugMsg[50];
            snprintf(debugMsg, sizeof(debugMsg), "Search gap: %dm %ds", gapMin, gapSec);
            ModelHandle_DebugPrint(debugMsg);
        }
    }
    else if (strncmp(command, "SEARCH_DRYRUN:", 14) == 0) {
        uint8_t dryMin, drySec;
        if (ModelHandle_ParseDuration(command + 14, &dryMin, &drySec)) {
            searchSettings.dryRunTimeSeconds = ModelHandle_DurationToSeconds(dryMin, drySec);
            char debugMsg[50];
            snprintf(debugMsg, sizeof(debugMsg), "Dry run time: %dm %ds", dryMin, drySec);
            ModelHandle_DebugPrint(debugMsg);
        }
    }
    else if (strncmp(command, "SEARCH_DAYS:", 12) == 0) {
        searchSettings.activeDays = ModelHandle_ParseDayBits(command + 12);
        searchSettings.searchActive = (searchSettings.activeDays != 0);
        ModelHandle_DebugPrint("Search days configured");
    }
}

void ModelHandle_TwistMode(const char* command) {
    if (strncmp(command, "TWIST_ONDUR:", 12) == 0) {
        uint8_t onMin, onSec;
        if (ModelHandle_ParseDuration(command + 12, &onMin, &onSec)) {
            twistSettings.onDurationSeconds = ModelHandle_DurationToSeconds(onMin, onSec);
            char debugMsg[50];
            snprintf(debugMsg, sizeof(debugMsg), "Twist ON duration: %dm %ds", onMin, onSec);
            ModelHandle_DebugPrint(debugMsg);
        }
    }
    else if (strncmp(command, "TWIST_OFFDUR:", 13) == 0) {
        uint8_t offMin, offSec;
        if (ModelHandle_ParseDuration(command + 13, &offMin, &offSec)) {
            twistSettings.offDurationSeconds = ModelHandle_DurationToSeconds(offMin, offSec);
            char debugMsg[50];
            snprintf(debugMsg, sizeof(debugMsg), "Twist OFF duration: %dm %ds", offMin, offSec);
            ModelHandle_DebugPrint(debugMsg);
        }
    }
    else if (strncmp(command, "TWIST_ONTIME:", 13) == 0) {
        uint8_t onHours, onMinutes;
        if (ModelHandle_ParseTime(command + 13, &onHours, &onMinutes)) {
            twistSettings.startTimeSeconds = ModelHandle_TimeToSeconds(onHours, onMinutes);
            char debugMsg[50];
            snprintf(debugMsg, sizeof(debugMsg), "Twist start time: %02d:%02d", onHours, onMinutes);
            ModelHandle_DebugPrint(debugMsg);
        }
    }
    else if (strncmp(command, "TWIST_OFFTIME:", 14) == 0) {
        uint8_t offHours, offMinutes;
        if (ModelHandle_ParseTime(command + 14, &offHours, &offMinutes)) {
            twistSettings.endTimeSeconds = ModelHandle_TimeToSeconds(offHours, offMinutes);
            char debugMsg[50];
            snprintf(debugMsg, sizeof(debugMsg), "Twist end time: %02d:%02d", offHours, offMinutes);
            ModelHandle_DebugPrint(debugMsg);
        }
    }
    else if (strncmp(command, "TWIST_DAYS:", 11) == 0) {
        twistSettings.activeDays = ModelHandle_ParseDayBits(command + 11);
        twistSettings.twistActive = (twistSettings.activeDays != 0);
        ModelHandle_DebugPrint("Twist days configured");
    }
}

void ModelHandle_SemiAutoMode(const char* command) {
    if (strcmp(command, "SEMI_ON") == 0) {
        Motor_On();
        ModelHandle_DebugPrint("Semi-auto mode: Motor ON");
    }
    else if (strcmp(command, "SEMI_OFF") == 0) {
        Motor_Off();
        ModelHandle_DebugPrint("Semi-auto mode: Motor OFF");
    }
}

void ModelHandle_ErrorMode(const char* command) {
    if (strncmp(command, "ERROR_MSG:", 10) == 0) {
        char errorMsg[100];
        strncpy(errorMsg, command + 10, sizeof(errorMsg) - 1);
        errorMsg[sizeof(errorMsg) - 1] = '\0';

        char debugMsg[150];
        snprintf(debugMsg, sizeof(debugMsg), "Error message received: %s", errorMsg);
        ModelHandle_DebugPrint(debugMsg);

        // Here you could add code to display the error on an LCD or trigger an alarm
    }
}

// Timer and countdown management
// ... [Previous content remains the same until the incomplete function] ...

// Timer and countdown management
void ModelHandle_RunTimers(void) {
    uint32_t currentTime = HAL_GetTick() / 1000; // Current time in seconds

    for (int i = 0; i < 5; i++) {
        if (timerSlots[i].active) {
            if (currentTime >= timerSlots[i].onTimeSeconds &&
                !timerSlots[i].executedToday) {
                Motor_On();
                timerSlots[i].executedToday = true;

                char debugMsg[50];
                snprintf(debugMsg, sizeof(debugMsg), "Timer slot %d: Motor ON", i+1);
                ModelHandle_DebugPrint(debugMsg);
            }
            else if (currentTime >= timerSlots[i].offTimeSeconds &&
                    timerSlots[i].executedToday) {
                Motor_Off();
                timerSlots[i].executedToday = false;

                char debugMsg[50];
                snprintf(debugMsg, sizeof(debugMsg), "Timer slot %d: Motor OFF", i+1);
                ModelHandle_DebugPrint(debugMsg);
            }

            // Reset executedToday flag at midnight (new day)
            if (currentTime % 86400 == 0) { // 86400 seconds = 24 hours
                timerSlots[i].executedToday = false;
            }
        }
    }
}

void ModelHandle_RunCountdown(void) {
    if (countdownActive) {
        uint32_t currentTime = HAL_GetTick() / 1000;

        if (currentTime >= countdownEndTime) {
            countdownActive = false;

            if (countdownMode) {
                // Countdown was for ON, so turn OFF when finished
                Motor_Off();
                ModelHandle_DebugPrint("Countdown finished: Motor OFF");
            } else {
                // Countdown was for OFF, so turn ON when finished
                Motor_On();
                ModelHandle_DebugPrint("Countdown finished: Motor ON");
            }
        }
    }
}

void ModelHandle_RunSearchLogic(void) {
    if (searchSettings.searchActive && searchSettings.testingGapSeconds > 0) {
        uint32_t currentTime = HAL_GetTick() / 1000;
        uint8_t currentDay = ModelHandle_GetCurrentDayOfWeek();

        // Check if today is an active day
        if (ModelHandle_IsDayActive(searchSettings.activeDays, currentDay)) {
            if (currentTime - searchSettings.lastTestTime >= searchSettings.testingGapSeconds) {
                // Time for a test run
                Motor_On();
                searchSettings.lastTestTime = currentTime;

                // Schedule motor off after dry run time
                // Note: This is simplified - in a real implementation, you'd need
                // to track this as another timer/countdown
                if (searchSettings.dryRunTimeSeconds > 0) {
                    // You'd implement this with a separate timer
                    ModelHandle_DebugPrint("Search mode: Dry run started");
                }
            }
        }
    }
}

void ModelHandle_RunTwistLogic(void) {
    if (twistSettings.twistActive) {
        uint32_t currentTime = HAL_GetTick() / 1000;
        uint8_t currentDay = ModelHandle_GetCurrentDayOfWeek();

        // Check if we're within the operational time window and active day
        if (ModelHandle_IsDayActive(twistSettings.activeDays, currentDay) &&
            currentTime >= twistSettings.startTimeSeconds &&
            currentTime < twistSettings.endTimeSeconds) {

            if (twistSettings.currentState) {
                // Currently ON, check if time to turn OFF
                if (currentTime - twistSettings.stateChangeTime >= twistSettings.onDurationSeconds) {
                    Motor_Off();
                    twistSettings.currentState = false;
                    twistSettings.stateChangeTime = currentTime;
                    ModelHandle_DebugPrint("Twist mode: Motor OFF");
                }
            } else {
                // Currently OFF, check if time to turn ON
                if (currentTime - twistSettings.stateChangeTime >= twistSettings.offDurationSeconds) {
                    Motor_On();
                    twistSettings.currentState = true;
                    twistSettings.stateChangeTime = currentTime;
                    ModelHandle_DebugPrint("Twist mode: Motor ON");
                }
            }
        } else {
            // Outside operational window or inactive day, ensure motor is OFF
            if (twistSettings.currentState) {
                Motor_Off();
                twistSettings.currentState = false;
                ModelHandle_DebugPrint("Twist mode: Outside schedule, Motor OFF");
            }
        }
    }
}

void ModelHandle_CheckAllModes(void) {
    ModelHandle_RunTimers();
    ModelHandle_RunCountdown();
    ModelHandle_RunSearchLogic();
    ModelHandle_RunTwistLogic();
}

// Utility functions
bool ModelHandle_ParseTime(const char* timeStr, uint8_t* hours, uint8_t* minutes) {
    if (timeStr == NULL || hours == NULL || minutes == NULL) return false;

    int h, m;
    if (sscanf(timeStr, "%d:%d", &h, &m) == 2) {
        if (h >= 0 && h <= 23 && m >= 0 && m <= 59) {
            *hours = (uint8_t)h;
            *minutes = (uint8_t)m;
            return true;
        }
    }
    return false;
}

bool ModelHandle_ParseDuration(const char* durationStr, uint8_t* minutes, uint8_t* seconds) {
    if (durationStr == NULL || minutes == NULL || seconds == NULL) return false;

    int m, s;
    if (sscanf(durationStr, "%d:%d", &m, &s) == 2) {
        if (m >= 0 && m <= 59 && s >= 0 && s <= 59) {
            *minutes = (uint8_t)m;
            *seconds = (uint8_t)s;
            return true;
        }
    }
    return false;
}

uint32_t ModelHandle_TimeToSeconds(uint8_t hours, uint8_t minutes) {
    return hours * 3600 + minutes * 60;
}

uint32_t ModelHandle_DurationToSeconds(uint8_t minutes, uint8_t seconds) {
    return minutes * 60 + seconds;
}

void ModelHandle_SecondsToTime(uint32_t seconds, uint8_t* hours, uint8_t* minutes) {
    if (hours && minutes) {
        *hours = (seconds / 3600) % 24;
        *minutes = (seconds % 3600) / 60;
    }
}

uint8_t ModelHandle_ParseDayBits(const char* daysStr) {
    uint8_t dayBits = 0;
    if (daysStr == NULL) return dayBits;

    // Simple implementation - assumes days are sent as space-separated names
    // You might need to adjust this based on how ESP32 sends the data
    if (strstr(daysStr, "sun") != NULL) dayBits |= (1 << SUNDAY);
    if (strstr(daysStr, "mon") != NULL) dayBits |= (1 << MONDAY);
    if (strstr(daysStr, "tue") != NULL) dayBits |= (1 << TUESDAY);
    if (strstr(daysStr, "wed") != NULL) dayBits |= (1 << WEDNESDAY);
    if (strstr(daysStr, "thu") != NULL) dayBits |= (1 << THURSDAY);
    if (strstr(daysStr, "fri") != NULL) dayBits |= (1 << FRIDAY);
    if (strstr(daysStr, "sat") != NULL) dayBits |= (1 << SATURDAY);

    return dayBits;
}

bool ModelHandle_IsDayActive(uint8_t dayBitmask, uint8_t currentDay) {
    return (dayBitmask & (1 << currentDay)) != 0;
}

uint8_t ModelHandle_GetCurrentDayOfWeek(void) {
    // This is a placeholder - you'd implement actual RTC or timekeeping logic
    // For now, return a fixed day (Monday) as an example
    return MONDAY;
}

void ModelHandle_DebugPrint(const char* message) {
    // Send debug message over UART
    char debugBuffer[128];
    snprintf(debugBuffer, sizeof(debugBuffer), "[DEBUG] %s\r\n", message);
    HAL_UART_Transmit(&huart1, (uint8_t*)debugBuffer, strlen(debugBuffer), HAL_MAX_DELAY);
}

// Main processing function to be called from main loop
void ModelHandle_Process(void) {
    // This function should be called regularly from main loop
    ModelHandle_CheckAllModes();
}
