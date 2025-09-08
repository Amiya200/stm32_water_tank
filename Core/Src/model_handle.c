#include "model_handle.h"
#include "relay.h"
#include "led.h"
#include "global.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart1;

/* ===== Public model state ===== */
volatile uint8_t  motorStatus = 0;

volatile bool     countdownActive   = false;
volatile bool     countdownMode     = true;   // true => ON countdown; false => OFF
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
    /* TODO: replace with RTC if available */
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
    if (senseDryRun && motorStatus == 1U) {
        motor_apply(false);
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

/* ===== Helpers ===== */
static int parse_mm_ss(const char* p, uint8_t* mm, uint8_t* ss)
{
    int m, s;
    if (sscanf(p, "%d:%d", &m, &s) == 2) {
        if (m < 0) { m = 0; } if (m > 59) { m = 59; }
        if (s < 0) { s = 0; } if (s > 59) { s = 59; }
        *mm = (uint8_t)m; *ss = (uint8_t)s;
        return 1;
    }
    return 0;
}

/* ===== Command parser (from LCD/UI or serial) ===== */
void ModelHandle_ProcessUartCommand(const char* cmd)
{
    if (!cmd || !*cmd) return;

    if (strcmp(cmd, "MOTOR_ON") == 0) {
        motor_apply(true);
    }
    else if (strcmp(cmd, "MOTOR_OFF") == 0) {
        motor_apply(false);
    }
    else if (strncmp(cmd, "COUNTDOWN_ON:", 13) == 0) {
        int minutes = atoi(cmd + 13);
        if (minutes < 0) minutes = 0;
        if (minutes > 600) minutes = 600;
        countdown_start(true, (uint32_t)minutes * 60UL);
    }
    else if (strncmp(cmd, "COUNTDOWN_OFF:", 14) == 0) {
        int minutes = atoi(cmd + 14);
        if (minutes < 0) minutes = 0;
        if (minutes > 600) minutes = 600;
        countdown_start(false, (uint32_t)minutes * 60UL);
    }
    else if (strncmp(cmd, "TIMER_SET:1:", 12) == 0) {
        uint8_t onH=0,onM=0,offH=0,offM=0;
        const char* p = cmd + 12;
        if (sscanf(p, "%hhu:%hhu:%hhu:%hhu", &onH,&onM,&offH,&offM) == 4) {
            timerSlots[0].onTimeSeconds  = ModelHandle_TimeToSeconds(onH,onM);
            timerSlots[0].offTimeSeconds = ModelHandle_TimeToSeconds(offH,offM);
            timerSlots[0].active = true;
            timerSlots[0].executedToday = false;
        }
    }
    else if (strncmp(cmd, "SEARCH_GAP:", 11) == 0) {
        uint8_t mm, ss;
        if (parse_mm_ss(cmd + 11, &mm, &ss)) {
            searchSettings.testingGapSeconds = (uint32_t)mm*60UL + (uint32_t)ss;
            searchSettings.searchActive = true;
        }
    }
    else if (strncmp(cmd, "SEARCH_DRYRUN:", 14) == 0) {
        uint8_t mm, ss;
        if (parse_mm_ss(cmd + 14, &mm, &ss)) {
            searchSettings.dryRunTimeSeconds = (uint32_t)mm*60UL + (uint32_t)ss;
            searchSettings.searchActive = true;
        }
    }
    else if (strncmp(cmd, "TWIST_ONDUR:", 12) == 0) {
        uint8_t mm, ss;
        if (parse_mm_ss(cmd + 12, &mm, &ss)) {
            twistSettings.onDurationSeconds = (uint32_t)mm*60UL + (uint32_t)ss;
            twistSettings.twistActive = true;
            twist_on_phase = false;
            twist_phase_deadline = now_ms();
        }
    }
    else if (strncmp(cmd, "TWIST_OFFDUR:", 13) == 0) {
        uint8_t mm, ss;
        if (parse_mm_ss(cmd + 13, &mm, &ss)) {
            twistSettings.offDurationSeconds = (uint32_t)mm*60UL + (uint32_t)ss;
            twistSettings.twistActive = true;
            twist_on_phase = false;
            twist_phase_deadline = now_ms();
        }
    }
    else if (strcmp(cmd, "TWIST_OFF") == 0) {
        twistSettings.twistActive = false;
        motor_apply(false);
    }
    else if (strcmp(cmd, "SEARCH_OFF") == 0) {
        searchSettings.searchActive = false;
        search_in_test = false;
        motor_apply(false);
    }
}
/* Backward-compat wrapper: some code calls this name */
void ModelHandle_ProcessReceivedPacket(const char* pkt)
{
    ModelHandle_ProcessUartCommand(pkt);
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

/* ===== UI helper ===== */
bool Motor_GetStatus(void) { return (motorStatus == 1U); }

/* ===== Debug ===== */
void ModelHandle_DebugPrint(const char* msg)
{
    if (!msg) return;
    char line[140];
    int n = snprintf(line, sizeof(line), "[MDL] %s\r\n", msg);
    if (n > 0) HAL_UART_Transmit(&huart1, (uint8_t*)line, (uint16_t)n, 50);
}
