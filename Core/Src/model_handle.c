#include "model_handle.h"
#include "relay.h"
#include "led.h"
#include "global.h"
#include "adc.h"
#include "rtc_i2c.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart1;
extern ADC_Data adcData;
extern RTC_Time_t time;

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

/* ===== Tank Check ===== */
static bool isTankFull(void)
{
    int submergedCount = 0;
    for (int i = 0; i < 5; i++) {
        if (adcData.voltages[i] < 0.1f) submergedCount++;
    }
    return (submergedCount >= 4);
}

/* ===== Manual Mode ===== */
void ModelHandle_ToggleManual(void)
{
    // Disable other modes
    semiAutoActive  = false;
    timerActive     = false;
    searchActive    = false;
    countdownActive = false;
    twistActive     = false;

    manualOverride = true;
    manualActive   = !manualActive;

    if (manualActive) {
        start_motor();
        printf("Manual ON\r\n");
    } else {
        stop_motor();
        printf("Manual OFF\r\n");
    }
}

void ModelHandle_ManualLongPress(void)
{
    manualOverride = false;
    manualActive   = false;

    printf("Manual Long Press → Restarting...\r\n");
    HAL_Delay(100);
    NVIC_SystemReset();
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

    if (isTankFull()) {
        stop_motor();
        countdownActive   = false;
        countdownDuration = 0;
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
            if (isTankFull()) {
                stop_motor();
                searchSettings.searchActive = false;
                searchActive = false;
            } else {
                start_motor();
                search_phase_deadline = tnow + (searchSettings.dryRunTimeSeconds * 1000UL);
            }
        }
    }
}

/* ===== Timer (RTC based) ===== */
static uint32_t seconds_since_midnight(void)
{
    RTC_GetTimeDate();
    return ((uint32_t)time.hour * 3600UL) +
           ((uint32_t)time.minutes * 60UL) +
           (uint32_t)time.seconds;
}

static void timer_tick(void)
{
    timerActive = false;
    uint32_t nowS = seconds_since_midnight();

    static uint32_t timerRetryDeadline = 0;

    for (int i = 0; i < 3; i++) {
        TimerSlot* s = &timerSlots[i];
        if (!s->active) continue;

        bool inWindow;
        if (s->onTimeSeconds <= s->offTimeSeconds) {
            inWindow = (nowS >= s->onTimeSeconds) && (nowS < s->offTimeSeconds);
        } else {
            inWindow = (nowS >= s->onTimeSeconds) || (nowS < s->offTimeSeconds);
        }

        if (inWindow) {
            timerActive = true;

            if (now_ms() < timerRetryDeadline) {
                return;
            }

            if (adcData.voltages[0] < 0.1f) {
                stop_motor();
                timerRetryDeadline = now_ms() + (searchSettings.testingGapSeconds * 1000UL);
                return;
            }

            if (isTankFull()) {
                stop_motor();
                return;
            }

            start_motor();
            return;
        }
    }

    stop_motor();
    timerRetryDeadline = 0;
}

/* ===== Semi-Auto ===== */
void ModelHandle_StartSemiAuto(void)
{
    manualOverride = false;
    manualActive   = false;
    timerActive    = false;
    searchActive   = false;
    countdownActive= false;
    twistActive    = false;

    semiAutoActive = true;

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
        if (!Motor_GetStatus()) {
            start_motor();
        }
    }
}

/* ===== Protections ===== */
static void protections_tick(void)
{
    if (manualOverride && manualActive) {
        // In manual → enforce only hard protections
        if (senseOverLoad && motorStatus == 1U) stop_motor();
        if (senseOverUnderVolt) stop_motor();
        if (maxRunTimerArmed && (now_ms() - maxRunStartTick) >= MAX_CONT_RUN_MS) {
            stop_motor();
            senseMaxRunReached = true;
            maxRunTimerArmed = false;
        }
        return;
    }

    if (motorStatus == 1U && adcData.voltages[0] < 0.1f) {
        senseDryRun = true;
        stop_motor();
    } else {
        senseDryRun = false;
    }

    if (senseOverLoad && motorStatus == 1U) stop_motor();
    if (senseOverUnderVolt) stop_motor();
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

/* ===== Public API ===== */
void ModelHandle_SetMotor(bool on)
{
    manualOverride = true;
    Relay_Set(1, on);
    motorStatus = on ? 1U : 0U;
}

void ModelHandle_ClearManualOverride(void)
{
    manualOverride = false;
}

bool Motor_GetStatus(void)
{
    return (motorStatus == 1U);
}

/* ===== Main tick ===== */
void ModelHandle_Process(void)
{
    if (manualOverride && manualActive) {
        protections_tick();
        leds_from_model();
        return;
    }

    countdown_tick();
    twist_tick();
    search_tick();
    timer_tick();
    semi_auto_tick();
    protections_tick();
    leds_from_model();
}

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
