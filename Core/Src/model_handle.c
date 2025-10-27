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

/* =========================
   CONFIG
   ========================= */
/* If your probe reads LOW when DRY, keep 1; if HIGH when DRY, set 0 */
#ifndef DRY_ACTIVE_LOW
#define DRY_ACTIVE_LOW 1
#endif

#ifndef DRY_THRESHOLD_V
#define DRY_THRESHOLD_V 0.10f
#endif

/* Twist: allow initial ON even if sensor shows dry (build pressure) */
#ifndef TWIST_PRIME_SECONDS
#define TWIST_PRIME_SECONDS 5
#endif

/* =========================
   Externals
   ========================= */
extern UART_HandleTypeDef huart1;
extern ADC_Data adcData;
extern RTC_Time_t time;

/* =========================
   Public model state
   ========================= */
volatile uint8_t  motorStatus = 0;

/* Mode trackers */
volatile bool manualActive    = false;
volatile bool countdownActive = false;
volatile bool twistActive     = false;
volatile bool searchActive    = false;
volatile bool timerActive     = false;
volatile bool semiAutoActive  = false;

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

/* =========================
   Utilities
   ========================= */
static inline uint32_t now_ms(void) { return HAL_GetTick(); }

uint32_t ModelHandle_TimeToSeconds(uint8_t hh, uint8_t mm) {
    return ((uint32_t)hh * 3600UL) + ((uint32_t)mm * 60UL);
}
void ModelHandle_SecondsToTime(uint32_t sec, uint8_t* hh, uint8_t* mm) {
    sec %= 24UL * 3600UL;
    if (hh) *hh = (uint8_t)(sec / 3600UL);
    if (mm) *mm = (uint8_t)((sec % 3600UL) / 60UL);
}

/* =========================
   Countdown (basic)
   ========================= */
volatile bool     countdownMode     = true;
volatile uint32_t countdownDuration = 0;

/* Timer slots (RTC based) */
TimerSlot timerSlots[5] = {0};

/* Search settings */
SearchSettings searchSettings = {
    .searchActive        = false,
    .testingGapSeconds   = 60,   // gap between probes
    .dryRunTimeSeconds   = 20,   // probe window (ON time for test)
};

/* Twist settings */
TwistSettings twistSettings = {
    .twistActive         = false,
    .onDurationSeconds   = 20,
    .offDurationSeconds  = 40,
};

/* =========================
   Motor Control
   ========================= */
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

/* =========================
   Tank Check (4/5 submerged)
   ========================= */
static bool isTankFull(void)
{
    int submergedCount = 0;
    for (int i = 0; i < 5; i++) {
        if (adcData.voltages[i] < 0.1f) submergedCount++;
    }
    return (submergedCount >= 4);
}

/* =========================
   DRY helpers (shared)
   ========================= */
static inline bool dry_raw_is_dry(void)
{
    float v = adcData.voltages[0];
    if (DRY_ACTIVE_LOW) return (v < DRY_THRESHOLD_V);
    else                return (v > DRY_THRESHOLD_V);
}

/* =========================
   Manual Mode
   ========================= */
void ModelHandle_ToggleManual(void)
{
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

/* ============================================================
   COUNTDOWN MODE (repeats, live MM:SS, tank auto-off)
   ============================================================ */
volatile uint16_t countdownRemainingRuns = 0;

static uint32_t cd_deadline_ms      = 0;
static uint32_t cd_rest_deadline_ms = 0;
static uint32_t cd_run_seconds      = 0;
static bool     cd_in_rest          = false;
static const uint32_t CD_REST_MS    = 3000;

void ModelHandle_StopCountdown(void)
{
    stop_motor();
    countdownActive        = false;
    countdownMode          = false;
    countdownRemainingRuns = 0;
    cd_run_seconds         = 0;
    cd_in_rest             = false;
    countdownDuration      = 0;
}
static void countdown_start_one_run(void)
{
    cd_deadline_ms    = now_ms() + (cd_run_seconds * 1000UL);
    countdownDuration = cd_run_seconds;
    cd_in_rest        = false;
    start_motor();
}
void ModelHandle_StartCountdown(uint32_t seconds_per_run, uint16_t repeats)
{
    if (seconds_per_run == 0 || repeats == 0) {
        ModelHandle_StopCountdown();
        return;
    }

    manualActive   = false;
    semiAutoActive = false;
    timerActive    = false;
    searchActive   = false;
    twistActive    = false;

    countdownMode          = true;
    countdownActive        = true;
    cd_run_seconds         = seconds_per_run;
    countdownRemainingRuns = repeats;
    cd_in_rest             = false;

    countdown_start_one_run();
}
static void countdown_tick(void)
{
    if (!countdownActive) return;
    uint32_t now = now_ms();

    if (cd_in_rest) {
        if ((int32_t)(cd_rest_deadline_ms - now) <= 0) {
            if (countdownRemainingRuns > 0) {
                countdown_start_one_run();
            } else {
                ModelHandle_StopCountdown();
            }
        }
        return;
    }

    if ((int32_t)(cd_deadline_ms - now) > 0) {
        uint32_t rem_ms = cd_deadline_ms - now;
        countdownDuration = (rem_ms + 999U) / 1000U;

        if (isTankFull()) {
            stop_motor();
            ModelHandle_StopCountdown();
        }
        return;
    }

    stop_motor();
    if (countdownRemainingRuns > 0) countdownRemainingRuns--;
    if (countdownRemainingRuns == 0) {
        ModelHandle_StopCountdown();
        return;
    }

    cd_in_rest          = true;
    cd_rest_deadline_ms = now + CD_REST_MS;
    countdownDuration   = 0;
}

/* =======================
   TWIST MODE (enhanced)
   ======================= */
static bool     twist_on_phase       = false;
static uint32_t twist_phase_deadline = 0;
static uint8_t  twist_dry_cnt        = 0;    // debounce
static bool     twist_priming        = false;
static uint32_t twist_prime_deadline = 0;

static inline bool isDryLowSupply_debounced(void)
{
    bool raw = dry_raw_is_dry();
    if (raw) {
        if (twist_dry_cnt < 255) twist_dry_cnt++;
    } else {
        twist_dry_cnt = 0;
    }
    return (twist_dry_cnt >= 3);
}
static inline void twist_arm_priming(void)
{
    if (TWIST_PRIME_SECONDS > 0) {
        twist_priming        = true;
        twist_prime_deadline = now_ms() + (uint32_t)TWIST_PRIME_SECONDS * 1000UL;
    } else {
        twist_priming        = false;
        twist_prime_deadline = 0;
    }
}

void ModelHandle_StartTwist(uint16_t on_s, uint16_t off_s)
{
    manualOverride   = false;
    manualActive     = false;
    semiAutoActive   = false;
    timerActive      = false;
    searchActive     = false;
    countdownActive  = false;

    if (on_s == 0)  on_s  = 1;
    if (off_s == 0) off_s = 1;

    twistSettings.onDurationSeconds  = on_s;
    twistSettings.offDurationSeconds = off_s;

    twistSettings.twistActive  = true;
    twistActive                = true;
    twist_on_phase             = false;
    twist_phase_deadline       = now_ms();
    twist_dry_cnt              = 0;
    twist_arm_priming();
}
void ModelHandle_StopTwist(void)
{
    twistSettings.twistActive = false;
    twistActive = false;
    twist_priming  = false;
    twist_dry_cnt  = 0;
    stop_motor();
}
static void twist_tick(void)
{
    if (!twistSettings.twistActive) { twistActive = false; return; }
    twistActive = true;

    if (isTankFull()) {
        ModelHandle_StopTwist();
        return;
    }
    if (maxRunTimerArmed && (now_ms() - maxRunStartTick) >= MAX_CONT_RUN_MS) {
        stop_motor();
        senseMaxRunReached = true;
        ModelHandle_StopTwist();
        return;
    }

    uint32_t tnow = now_ms();

    if (twist_priming) {
        if (tnow < twist_prime_deadline) {
            if (!Motor_GetStatus()) start_motor();
            twist_on_phase       = true;
            twist_phase_deadline = tnow + (uint32_t)twistSettings.onDurationSeconds * 1000UL;
            return;
        } else {
            twist_priming = false;
        }
    }

    if ((int32_t)(twist_phase_deadline - tnow) > 0) return;

    if (isDryLowSupply_debounced()) {
        stop_motor();
        twist_on_phase       = false;
        twist_phase_deadline = tnow + (uint32_t)twistSettings.offDurationSeconds * 1000UL;
        return;
    }

    twist_on_phase = !twist_on_phase;
    if (twist_on_phase) {
        start_motor();
        twist_phase_deadline = tnow + (uint32_t)twistSettings.onDurationSeconds * 1000UL;
    } else {
        stop_motor();
        twist_phase_deadline = tnow + (uint32_t)twistSettings.offDurationSeconds * 1000UL;
    }
}

/* =======================
   SEARCH MODE (periodic supply test)
   ======================= */
typedef enum {
    SEARCH_GAP_WAIT = 0,   // OFF; wait gap before probe
    SEARCH_PROBE,          // ON briefly to test supply
    SEARCH_RUN             // ON continuously while supply ok
} SearchState;

static SearchState search_state = SEARCH_GAP_WAIT;
static uint32_t    search_deadline_ms = 0;
static uint8_t     search_dry_cnt = 0;    // debounce while RUN

void ModelHandle_StartSearch(uint16_t gap_s, uint16_t probe_s)
{
    manualOverride   = false;
    manualActive     = false;
    semiAutoActive   = false;
    timerActive      = false;
    countdownActive  = false;
    twistActive      = false;

    if (gap_s   == 0) gap_s   = 5;
    if (probe_s == 0) probe_s = 3;

    searchSettings.testingGapSeconds = gap_s;     // gap
    searchSettings.dryRunTimeSeconds = probe_s;   // probe

    searchSettings.searchActive = true;
    searchActive = true;

    stop_motor();
    search_state       = SEARCH_GAP_WAIT;
    search_deadline_ms = now_ms() + (uint32_t)searchSettings.testingGapSeconds * 1000UL;
    search_dry_cnt     = 0;
}
void ModelHandle_StopSearch(void)
{
    searchSettings.searchActive = false;
    searchActive = false;
    search_state = SEARCH_GAP_WAIT;
    stop_motor();
}
static inline bool isDryDebounced_RUN(void)
{
    if (dry_raw_is_dry()) {
        if (search_dry_cnt < 255) search_dry_cnt++;
    } else {
        search_dry_cnt = 0;
    }
    return (search_dry_cnt >= 3);
}
static void search_tick(void)
{
    if (!searchSettings.searchActive) { searchActive = false; return; }
    searchActive = true;

    uint32_t now = now_ms();

    if (isTankFull()) {
        stop_motor();
        searchSettings.searchActive = false;
        searchActive = false;
        return;
    }

    switch (search_state)
    {
        case SEARCH_GAP_WAIT:
            if (Motor_GetStatus()) stop_motor();
            if ((int32_t)(search_deadline_ms - now) <= 0) {
                start_motor(); // probe ON
                search_state       = SEARCH_PROBE;
                search_deadline_ms = now + (uint32_t)searchSettings.dryRunTimeSeconds * 1000UL;
                search_dry_cnt     = 0;
            }
            break;

        case SEARCH_PROBE:
            if (!dry_raw_is_dry()) {
                // Water detected → RUN
                search_state       = SEARCH_RUN;
                search_deadline_ms = 0;
                search_dry_cnt     = 0;
                break;
            }
            if ((int32_t)(search_deadline_ms - now) <= 0) {
                // Still dry after probe
                stop_motor();
                search_state       = SEARCH_GAP_WAIT;
                search_deadline_ms = now + (uint32_t)searchSettings.testingGapSeconds * 1000UL;
            }
            break;

        case SEARCH_RUN:
            if (isDryDebounced_RUN()) {
                stop_motor();
                search_state       = SEARCH_GAP_WAIT;
                search_deadline_ms = now + (uint32_t)searchSettings.testingGapSeconds * 1000UL;
                search_dry_cnt     = 0;
                break;
            }
            if (!Motor_GetStatus()) start_motor(); // keep asserted
            break;
    }
}

/* =======================
   TIMER (RTC based)
   ======================= */
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

            if (now_ms() < timerRetryDeadline) return;

            if (dry_raw_is_dry()) {
                stop_motor();
                timerRetryDeadline = now_ms() + (uint32_t)searchSettings.testingGapSeconds * 1000UL;
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

/* =======================
   SEMI-AUTO
   ======================= */
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
        if (!Motor_GetStatus()) start_motor();
    }
}

/* =======================
   PROTECTIONS
   ======================= */
static void protections_tick(void)
{
    /* Manual override → only hard protections */
    if (manualOverride && manualActive) {
        if (senseOverLoad && motorStatus == 1U) stop_motor();
        if (senseOverUnderVolt) stop_motor();
        if (maxRunTimerArmed && (now_ms() - maxRunStartTick) >= MAX_CONT_RUN_MS) {
            stop_motor();
            senseMaxRunReached = true;
            maxRunTimerArmed = false;
        }
        return;
    }

    /* DRY:
       - Always set the flag for UI.
       - Hard-stop on dry ONLY if Twist is NOT active (Twist soft-handles dry). */
    if (motorStatus == 1U && dry_raw_is_dry()) {
        senseDryRun = true;
        if (!twistSettings.twistActive) {
            stop_motor();
        }
    } else {
        senseDryRun = false;
    }

    /* Other protections remain hard stops */
    if (senseOverLoad && motorStatus == 1U) stop_motor();
    if (senseOverUnderVolt) stop_motor();

    if (maxRunTimerArmed && (now_ms() - maxRunStartTick) >= MAX_CONT_RUN_MS) {
        stop_motor();
        senseMaxRunReached = true;
        maxRunTimerArmed = false;
    }
}

/* =======================
   LEDs
   ======================= */
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

/* =======================
   Public Motor APIs
   ======================= */
void ModelHandle_SetMotor(bool on)
{
    manualOverride = true;
    Relay_Set(1, on);
    motorStatus = on ? 1U : 0U;
}
void ModelHandle_ClearManualOverride(void) { manualOverride = false; }
bool Motor_GetStatus(void) { return (motorStatus == 1U); }

/* =======================
   Main pump
   ======================= */
void ModelHandle_Process(void)
{
    countdown_tick();
    twist_tick();
    search_tick();
    timer_tick();
    semi_auto_tick();

    protections_tick();
    leds_from_model();
}

/* =======================
   UART commands
   ======================= */
void ModelHandle_ProcessUartCommand(const char* cmd)
{
    if (!cmd || !*cmd) return;

    if      (strcmp(cmd, "MOTOR_ON") == 0)  { manualOverride = true; manualActive = true;  start_motor(); printf("Manual ON (UART)\r\n"); }
    else if (strcmp(cmd, "MOTOR_OFF")== 0)  { manualOverride = true; manualActive = false; stop_motor();  printf("Manual OFF (UART)\r\n"); }
    else if (strcmp(cmd, "SEMI_AUTO_START")==0) { ModelHandle_StartSemiAuto(); }
}
