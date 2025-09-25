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

volatile bool     countdownActive   = false;
volatile bool     countdownMode     = true;
volatile uint32_t countdownDuration = 0;
static   uint32_t countdownDeadline = 0;

TimerSlot timerSlots[5] = {0};

SearchSettings searchSettings = {
    .searchActive        = false,
    .testingGapSeconds   = 30,
    .dryRunTimeSeconds   = 10,
};

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

static const uint32_t MAX_CONT_RUN_MS = 2UL * 60UL * 60UL * 1000UL; // 2h
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

/* ===== Motor ===== */
static void motor_apply(bool on)
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

/* Public API for motor control */
void ModelHandle_SetMotor(bool on)
{
    motor_apply(on);
}

/* ===== Countdown ===== */
static void countdown_start(bool onMode, uint32_t seconds)
{
    if (seconds == 0) { countdownActive = false; return; }
    countdownMode     = onMode;
    countdownDuration = seconds;
    countdownActive   = true;
    countdownDeadline = now_ms() + (seconds * 1000UL);

    if (onMode) motor_apply(true);
    else        motor_apply(false);
}

static void countdown_tick(void)
{
    if (!countdownActive) return;

    uint32_t tnow = now_ms();
    if ((int32_t)(countdownDeadline - tnow) <= 0) {
        if (countdownMode) motor_apply(false);
        else               motor_apply(true);
        countdownActive   = false;
        countdownDuration = 0;
        return;
    }

    uint32_t remaining_ms = countdownDeadline - tnow;
    countdownDuration = (remaining_ms + 999) / 1000;
}

/* ===== Twist ===== */
static bool     twist_on_phase = false;
static uint32_t twist_phase_deadline = 0;

static void twist_tick(void)
{
    if (!twistSettings.twistActive) return;

    uint32_t tnow = now_ms();
    if ((int32_t)(twist_phase_deadline - tnow) > 0) return;

    twist_on_phase = !twist_on_phase;
    if (twist_on_phase) {
        motor_apply(true);
        twist_phase_deadline = tnow + (twistSettings.onDurationSeconds * 1000UL);
    } else {
        motor_apply(false);
        twist_phase_deadline = tnow + (twistSettings.offDurationSeconds * 1000UL);
    }
}

/* ===== Search ===== */
static bool     search_in_test = false;
static uint32_t search_phase_deadline = 0;

static void search_tick(void)
{
    if (!searchSettings.searchActive) return;

    uint32_t tnow = now_ms();
    if ((int32_t)(search_phase_deadline - tnow) > 0) return;

    if (!search_in_test) {
        search_in_test = true;
        motor_apply(true);
        search_phase_deadline = tnow + (searchSettings.dryRunTimeSeconds * 1000UL);
    } else {
        motor_apply(false);
        search_in_test = false;
        search_phase_deadline = tnow + (searchSettings.testingGapSeconds * 1000UL);
    }
}

/* ===== Timer (daily window) ===== */
static uint32_t seconds_since_midnight(void)
{
    uint32_t ms = now_ms() % (24UL*3600UL*1000UL);
    return ms / 1000UL;
}

static void timer_tick(void)
{
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
            motor_apply(inWindow);
        }
    }
}

/* ===== Protections ===== */
static void protections_tick(void)
{
    /* ✅ Dry run: motor ON but ADC channel 0 shows water missing */
    if (motorStatus == 1U && adcData.voltages[0] < 0.1f) {
        senseDryRun = true;
        motor_apply(false);
    } else {
        senseDryRun = false;
    }

    if (senseOverLoad && motorStatus == 1U) {
        motor_apply(false);
    }

    if (senseOverUnderVolt) {
        motor_apply(false);
    }

    if (maxRunTimerArmed && (now_ms() - maxRunStartTick) >= MAX_CONT_RUN_MS) {
        motor_apply(false);
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
    if (countdownActive && countdownMode) {
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
        motor_apply(true);
    }
    else if (strcmp(cmd, "MOTOR_OFF") == 0) {
        motor_apply(false);
    }
    /* ... add other commands as needed ... */
}

/* ===== External setters ===== */
void ModelHandle_SetDryRun(bool on)        { senseDryRun = on; }
void ModelHandle_SetOverLoad(bool on)      { senseOverLoad = on; }
void ModelHandle_SetOverUnderVolt(bool on) { senseOverUnderVolt = on; }
void ModelHandle_ClearMaxRunFlag(void)     { senseMaxRunReached = false; }

/* ===== Main tick ===== */
void ModelHandle_Process(void)
{
    countdown_tick();
    twist_tick();
    search_tick();
    timer_tick();
    protections_tick();
    leds_from_model();
}

/* ===== Public getter ===== */
bool Motor_GetStatus(void)
{
    return (motorStatus == 1U);
}
