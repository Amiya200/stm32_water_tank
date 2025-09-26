#include "model_handle.h"
#include "relay.h"
#include "led.h"
#include "global.h"
#include "adc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart1;
extern ADC_Data adcData;

/* ===== Public model state ===== */
volatile uint8_t  motorStatus = 0;

/* Mode trackers */
volatile bool manualActive    = false;
volatile bool countdownActive = false;
volatile bool twistActive     = false;
volatile bool searchActive    = false;
volatile bool timerActive     = false;
volatile bool semiAutoActive  = false;

/* Countdown */
volatile bool     countdownMode     = true;
volatile uint32_t countdownDuration = 0;
static   uint32_t countdownDeadline = 0;

/* Timer */
TimerSlot timerSlots[5] = {0};

/* Search */
SearchSettings searchSettings = {
    .searchActive        = false,
    .testingGapSeconds   = 60,
    .dryRunTimeSeconds   = 20,
};

/* Twist */
TwistSettings twistSettings = {
    .twistActive         = false,
    .onDurationSeconds   = 20,
    .offDurationSeconds  = 40,
};

/* Protections */
volatile bool senseDryRun         = false;
volatile bool senseOverLoad       = false;
volatile bool senseOverUnderVolt  = false;
volatile bool senseMaxRunReached  = false;
volatile bool manualOverride      = false;

/* Max runtime protection (2h default) */
static const uint32_t MAX_CONT_RUN_MS = 2UL * 60UL * 60UL * 1000UL;
static bool           maxRunTimerArmed = false;
static uint32_t       maxRunStartTick  = 0;

/* ===== Utilities ===== */
static inline uint32_t now_ms(void) { return HAL_GetTick(); }

uint32_t ModelHandle_TimeToSeconds(uint8_t hh, uint8_t mm) {
    return ((uint32_t)hh * 3600UL) + ((uint32_t)mm * 60UL);
}

void ModelHandle_SecondsToTime(uint32_t sec, uint8_t* hh, uint8_t* mm) {
    sec %= 24UL * 3600UL;
    if (hh) *hh = (uint8_t)(sec / 3600UL);
    if (mm) *mm = (uint8_t)((sec % 3600UL) / 60UL);
}

/* ===== Motor Control ===== */
static inline void motor_apply(bool on)
{
    Relay_Set(1, on);
    motorStatus = on ? 1U : 0U;

    if (on) {
        if (!maxRunTimerArmed) {
            maxRunTimerArmed = true;
            maxRunStartTick  = now_ms();
        }
    } else {
        maxRunTimerArmed = false;
    }
}

static inline void start_motor(void) { motor_apply(true); }
static inline void stop_motor(void)  { motor_apply(false); }

/* ===== Manual Mode ===== */
/* Toggle manual ON/OFF (button short press) */
void ModelHandle_ToggleManual(void)
{
    manualOverride = true;   // manual bypass enabled
    manualActive   = !manualActive;

    if (manualActive) {
        start_motor();
        printf("Manual ON\r\n");
    } else {
        stop_motor();
        printf("Manual OFF\r\n");
    }
}

/* Manual long press = restart device */
void ModelHandle_ManualLongPress(void)
{
    manualOverride = false;
    manualActive   = false;

    printf("Manual Long Press → Restarting into last mode...\r\n");
    HAL_Delay(100);
    NVIC_SystemReset();   // hardware reset, system boots into last saved mode
}

/* ===== Countdown ===== */
static void countdown_start(uint32_t seconds)
{
    if (seconds == 0) { countdownActive = false; return; }

    countdownMode     = true;
    countdownDuration = seconds;
    countdownActive   = true;

    manualActive   = false;
    twistActive    = false;
    searchActive   = false;
    timerActive    = false;
    semiAutoActive = false;

    countdownDeadline = now_ms() + (seconds * 1000UL);
    start_motor();
}

static void countdown_tick(void)
{
    if (!countdownActive) return;

    uint32_t tnow = now_ms();
    if ((int32_t)(countdownDeadline - tnow) <= 0) {
        stop_motor();
        countdownActive   = false;
        countdownDuration = 0;
        return;
    }

    uint32_t remaining_ms = countdownDeadline - tnow;
    countdownDuration = (remaining_ms + 999) / 1000;

    if (adcData.voltages[4] < 0.1f) {
        stop_motor();
        countdownActive = false;
    }
}

/* ===== Twist ===== */
static bool     twist_on_phase = false;
static uint32_t twist_phase_deadline = 0;

static void twist_tick(void)
{
    if (!twistSettings.twistActive) { twistActive = false; return; }
    twistActive = true;

    uint32_t tnow = now_ms();
    if ((int32_t)(twist_phase_deadline - tnow) > 0) return;

    twist_on_phase = !twist_on_phase;
    if (twist_on_phase) {
        start_motor();
        twist_phase_deadline = tnow + (twistSettings.onDurationSeconds * 1000UL);
    } else {
        stop_motor();
        twist_phase_deadline = tnow + (twistSettings.offDurationSeconds * 1000UL);
    }

    if (adcData.voltages[4] < 0.1f) {
        stop_motor();
        twistSettings.twistActive = false;
        twistActive = false;
    }
}

/* ===== Search ===== */
static bool     search_in_test = false;
static uint32_t search_phase_deadline = 0;

static void search_tick(void)
{
    if (!searchSettings.searchActive) { searchActive = false; return; }
    searchActive = true;

    uint32_t tnow = now_ms();
    if ((int32_t)(search_phase_deadline - tnow) > 0) return;

    if (!search_in_test) {
        search_in_test = true;
        start_motor();
        search_phase_deadline = tnow + (searchSettings.dryRunTimeSeconds * 1000UL);
    } else {
        if (adcData.voltages[0] < 0.1f) {
            stop_motor();
            search_in_test = false;
            search_phase_deadline = tnow + (searchSettings.testingGapSeconds * 1000UL);
        } else {
            if (adcData.voltages[4] < 0.1f) {
                stop_motor();
                searchSettings.searchActive = false;
                searchActive = false;
            } else {
                start_motor();
                search_phase_deadline = tnow + 5000;
            }
        }
    }
}

/* ===== Timer ===== */
static uint32_t seconds_since_midnight(void)
{
    uint32_t ms = now_ms() % (24UL*3600UL*1000UL);
    return ms / 1000UL;
}

static void timer_tick(void)
{
    timerActive = false;
    uint32_t nowS = seconds_since_midnight();

    for (int i = 0; i < 5; i++) {
        TimerSlot* s = &timerSlots[i];
        if (!s->active) continue;

        bool inWindow;
        if (s->onTimeSeconds <= s->offTimeSeconds) {
            inWindow = (nowS >= s->onTimeSeconds) && (nowS < s->offTimeSeconds);
        } else {
            inWindow = (nowS >= s->onTimeSeconds) || (nowS < s->offTimeSeconds);
        }

        if (i == 0) {
            if (inWindow) {
                timerActive = true;
                if (adcData.voltages[0] < 0.1f) {
                    stop_motor();
                } else {
                    start_motor();
                }
            } else {
                stop_motor();
            }
        }
    }
}

/* ===== Protections ===== */
static void protections_tick(void)
{
    if (manualOverride) return; // skip for manual

    if (motorStatus == 1U && adcData.voltages[0] < 0.1f) {
        senseDryRun = true;
        stop_motor();
    } else {
        senseDryRun = false;
    }

    if (senseOverLoad && motorStatus == 1U) {
        stop_motor();
    }

    if (senseOverUnderVolt) {
        stop_motor();
    }

    if (maxRunTimerArmed && (now_ms() - maxRunStartTick) >= MAX_CONT_RUN_MS) {
        stop_motor();
        senseMaxRunReached = true;
        maxRunTimerArmed = false;
    }
}

/* ===== LED synthesis ===== */
static void leds_from_model(void)
{
    LED_ClearAllIntents();

    if (motorStatus == 1U) {
        LED_SetIntent(LED_COLOR_GREEN, LED_MODE_STEADY, 0);
    }
    if (countdownActive) {
        LED_SetIntent(LED_COLOR_GREEN, LED_MODE_BLINK, 500);
    }
    if (senseDryRun) {
        LED_SetIntent(LED_COLOR_RED, LED_MODE_STEADY, 0);
    }
    if (senseMaxRunReached) {
        LED_SetIntent(LED_COLOR_RED, LED_MODE_BLINK, 400);
    }
    if (senseOverLoad) {
        LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 350);
    }
    if (senseOverUnderVolt) {
        LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_BLINK, 350);
    }

    LED_ApplyIntents();
}

/* ===== Command parser ===== */
void ModelHandle_ProcessUartCommand(const char* cmd)
{
    if (!cmd || !*cmd) return;

    if (strcmp(cmd, "MOTOR_ON") == 0) {
        manualOverride = true;
        manualActive   = true;
        start_motor();
        printf("Manual ON (UART)\r\n");
    }
    else if (strcmp(cmd, "MOTOR_OFF") == 0) {
        manualOverride = true;
        manualActive   = false;
        stop_motor();
        printf("Manual OFF (UART)\r\n");
    }
    else if (strcmp(cmd, "SEMI_AUTO_START") == 0) {
        ModelHandle_StartSemiAuto();
    }
}


/* Public API for motor control */
void ModelHandle_SetMotor(bool on)
{
    manualOverride = true;   // force manual override
    Relay_Set(1, on);
    motorStatus = on ? 1U : 0U;
}

void ModelHandle_ClearManualOverride(void)
{
    manualOverride = false;
}

/* === Semi-Auto === */

static bool isTankFull(void)
{
    int submergedCount = 0;
    for (int i = 0; i < 5; i++) {
        if (adcData.voltages[i] < 0.1f) submergedCount++;
    }
    return (submergedCount >= 4); // consider FULL when 4 or more sensors submerged
}

void ModelHandle_StartSemiAuto(void)
{
    semiAutoActive = true;
    manualOverride = false;
    manualActive   = false;

    // If not full, start motor
    if (!isTankFull()) {
        start_motor();
        printf("Semi-Auto Started\r\n");
    } else {
        stop_motor();
        semiAutoActive = false;
        printf("Semi-Auto Not Started: Already Full\r\n");
    }
}


static void semi_auto_tick(void)
{
    if (!semiAutoActive) return;

    if (isTankFull()) {
        stop_motor();
        semiAutoActive   = false;
        maxRunTimerArmed = false;
        printf("Semi-Auto Complete: Tank Full\r\n");
    } else {
        // keep motor running until full
        if (!Motor_GetStatus()) {
            start_motor();
        }
    }
}

/* ===== Main tick ===== */
void ModelHandle_Process(void)
{
    countdown_tick();
    twist_tick();
    search_tick();
    timer_tick();
    semi_auto_tick();   // <-- now fixed
    protections_tick();
    leds_from_model();
}


/* ===== Public getter ===== */
bool Motor_GetStatus(void)
{
    return (motorStatus == 1U);
}
