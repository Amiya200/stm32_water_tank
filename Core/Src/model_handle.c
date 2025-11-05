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
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "uart_commands.h"
/* =========================
   CONFIG
   ========================= */
#ifndef DRY_ACTIVE_LOW
#define DRY_ACTIVE_LOW 1
#endif

#ifndef DRY_THRESHOLD_V
#define DRY_THRESHOLD_V 0.10f
#endif

/* Global dry-run stop delay (per spec: 30–60s). Adjust as needed. */
#ifndef DRY_STOP_DELAY_SECONDS
#define DRY_STOP_DELAY_SECONDS 30
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

/* DRY protection timer (delayed stop) */
static bool     dryTimerArmed    = false;
static uint32_t dryStopDeadline  = 0;

/* =========================
   Utilities
   ========================= */

// =====================================================
// RATE-LIMITED STATUS UPDATER to avoid UART spam
// =====================================================
static void Safe_SendStatusPacket(void)
{
    static uint32_t lastSentTick = 0;
    uint32_t now = HAL_GetTick();

    // only allow one send every 3000 ms
    if (now - lastSentTick < 3000) return;

    lastSentTick = now;
    UART_SendStatusPacket();
}


static inline uint32_t now_ms(void) { return HAL_GetTick(); }

static inline void clear_all_modes(void)
{
    manualActive    = false;
    countdownActive = false;
    twistActive     = false;
    searchActive    = false;
    timerActive     = false;
    semiAutoActive  = false;
    manualOverride  = false;
}

void ModelHandle_ResetAll(void)
{
    clear_all_modes();

    senseDryRun = senseOverLoad = senseOverUnderVolt = senseMaxRunReached = false;
    maxRunTimerArmed = false;
    maxRunStartTick  = 0;
    dryTimerArmed    = false;
    dryStopDeadline  = 0;

    Relay_Set(1, false);
    Relay_Set(2, false);
    Relay_Set(3, false);
    motorStatus = 0;

    LED_ClearAllIntents();
    LED_ApplyIntents();

    // ✅ Initialize defaults moved from compile-time
    memset(timerSlots, 0, sizeof(timerSlots));
    searchSettings.searchActive = false;
    searchSettings.testingGapSeconds = 60;
    searchSettings.dryRunTimeSeconds = 20;
    twistSettings.twistActive = false;
    twistSettings.onDurationSeconds = 20;
    twistSettings.offDurationSeconds = 40;
}

//TimerSlot timerSlots[5];  // define global array
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

/* ===== Aux burst (Relays 2 & 3 for a fixed time) ===== */
static bool     auxBurstActive     = false;
static uint32_t auxBurstDeadlineMs = 0;

/* Timer slots (RTC based) */
TimerSlot timerSlots[5];
SearchSettings searchSettings;
TwistSettings twistSettings;


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
            Safe_SendStatusPacket();
        }
    } else {
        maxRunTimerArmed = false;
        // When motor turns OFF, cancel pending dry timer
        dryTimerArmed = false;
        dryStopDeadline = 0;
        Safe_SendStatusPacket();
    }
}

static inline void start_motor(void)
{
    motor_apply(true);
    //printf("Relay1 -> %s\r\n", Relay_Get(1) ? "ON" : "OFF");
}
static inline void stop_motor_keep_modes(void)  { motor_apply(false); }

/* Hard OFF: stop and clear every mode flag (used for terminal OFF or external OFF) */
void ModelHandle_StopAllModesAndMotor(void)
{
    clear_all_modes();
    stop_motor_keep_modes();
    ////printf("ALL MODES OFF + MOTOR OFF\r\n");
    ModelHandle_SaveCurrentStateToEEPROM();
}

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

/* ===== Aux burst public API ===== */
void ModelHandle_TriggerAuxBurst(uint16_t seconds)
{
    if (seconds == 0) seconds = 1;
    Relay_Set(2, true);
    Relay_Set(3, true);
    auxBurstActive     = true;
    auxBurstDeadlineMs = HAL_GetTick() + ((uint32_t)seconds * 1000UL);
}
bool ModelHandle_AuxBurstActive(void) { return auxBurstActive; }

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
    // toggling manual should dominate and clear other modes
    bool will_on = !manualActive;

    if (will_on) {
        clear_all_modes();
        manualOverride = true;
        manualActive   = true;
        start_motor();
        //printf("Manual ON\r\n");
    } else {
        ModelHandle_StopAllModesAndMotor(); // OFF should clear everything
        //printf("Manual OFF\r\n");
    }
    ModelHandle_SaveCurrentStateToEEPROM();
}

void ModelHandle_ManualLongPress(void)
{
    //printf("Manual Long Press → System Reset\r\n");
    HAL_Delay(100);
    NVIC_SystemReset();
}

/* ============================================================
   COUNTDOWN MODE
   ============================================================ */
volatile uint16_t countdownRemainingRuns = 0;

static uint32_t cd_deadline_ms      = 0;
static uint32_t cd_rest_deadline_ms = 0;
static uint32_t cd_run_seconds      = 0;
static bool     cd_in_rest          = false;
static const uint32_t CD_REST_MS    = 3000;

void ModelHandle_StopCountdown(void)
{
    stop_motor_keep_modes();
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

    clear_all_modes();
    countdownMode          = true;
    countdownActive        = true;
    cd_run_seconds         = seconds_per_run;
    countdownRemainingRuns = repeats;
    cd_in_rest             = false;

    countdown_start_one_run();
    ModelHandle_SaveCurrentStateToEEPROM();
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
            ModelHandle_StopAllModesAndMotor(); // terminal condition -> clear all modes
        }
        return;
    }

    stop_motor_keep_modes();
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
   TWIST MODE
   ======================= */
static bool     twist_on_phase       = false;
static uint32_t twist_phase_deadline = 0;
/* DRY counter must not accumulate while motor is OFF */
static uint8_t  twist_dry_cnt        = 0;
static bool     twist_priming        = false;
static uint32_t twist_prime_deadline = 0;

/* Count DRY only while motor is ON to avoid latch-off after first OFF */
static inline bool isDryLowSupply_debounced(void)
{
    if (!Motor_GetStatus()) {
        // If motor is OFF, do not count; keep counter at 0
        twist_dry_cnt = 0;
        return false;
    }

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
    clear_all_modes();
    if (on_s == 0)  on_s  = 1;
    if (off_s == 0) off_s = 1;

    twistSettings.onDurationSeconds  = on_s;
    twistSettings.offDurationSeconds = off_s;
    twistSettings.twistActive  = true;
    twistActive                = true;
    twist_on_phase       = false;
    twist_phase_deadline = now_ms() + (uint32_t)twistSettings.offDurationSeconds * 1000UL;
    twist_dry_cnt        = 0;
    // Do NOT arm priming here; it's armed when ON actually starts.

    twist_arm_priming();
    ModelHandle_SaveCurrentStateToEEPROM();
}

void ModelHandle_StopTwist(void)
{
    twistSettings.twistActive = false;
    twistActive = false;
    twist_priming  = false;
    twist_dry_cnt  = 0;
    stop_motor_keep_modes();
}
// Call this from your main scheduler at ~10–100ms rate
// Call ~every 10–100 ms from your main loop/scheduler
void twist_tick(void)
{
    // 0) Inert unless explicitly enabled by the user
    if (!twistSettings.twistActive) {
        // Defensive: never force motor here—other modes own it.
        return;
    }

    uint32_t tnow = now_ms();

    // 1) Global safety gates (if your project uses these flags)
    //    If any protection is latched, keep Twist "enabled" but stop driving the motor.
    if (senseOverLoad || senseOverUnderVolt || senseMaxRunReached) {
        stop_motor_keep_modes();   // do not clear modes; just stop motor output
        return;
    }

    // 2) Priming window logic:
    //    When we first enter an ON phase we allow a short "prime" even if the sensor reads dry,
    //    to build initial pressure. This DOES NOT move the phase deadline.
    if (twist_priming) {
        if ((int32_t)(twist_prime_deadline - tnow) > 0) {
            if (!Motor_GetStatus()) start_motor();   // stay ON during priming
            return;                                  // do not touch phase deadline
        } else {
            twist_priming = false;                   // priming finished
        }
    }

    // 3) While in ON phase, you may optionally gate on "dry" (if your design wants this).
    //    We DON’T toggle the motor from ADC elsewhere; we only *observe* a dry flag here.
    //    If you don’t want dry handling in Twist, you can remove this block.
#if defined(DRY_STOP_DELAY_SECONDS) && (DRY_STOP_DELAY_SECONDS > 0)
    if (twist_on_phase) {
        // If a dry condition is being signaled by the sensing layer, count it.
        if (senseDryRun) {
            if (twist_dry_cnt < 0xFF) twist_dry_cnt++;
        } else {
            twist_dry_cnt = 0;
        }

        // If dry persisted long enough, end the ON phase early and jump to OFF.
        if (twist_dry_cnt >= (uint8_t)DRY_STOP_DELAY_SECONDS) {
            stop_motor_keep_modes();
            twist_on_phase       = false;
            twist_dry_cnt        = 0;
            twist_phase_deadline = tnow + (uint32_t)twistSettings.offDurationSeconds * 1000UL;
            return;
        }
    }
#endif

    // 4) Phase timing: flip when deadline elapses (signed compare prevents wrap issues).
    if ((int32_t)(twist_phase_deadline - tnow) <= 0) {
        twist_on_phase = !twist_on_phase;

        if (twist_on_phase) {
            // Entering ON phase
            twist_dry_cnt        = 0;
            start_motor();
            twist_phase_deadline = tnow + (uint32_t)twistSettings.onDurationSeconds * 1000UL;

            // Arm priming exactly when ON begins (once per ON phase).
#if defined(TWIST_PRIME_SECONDS) && (TWIST_PRIME_SECONDS > 0)
            twist_priming        = true;
            twist_prime_deadline = tnow + (uint32_t)TWIST_PRIME_SECONDS * 1000UL;
#endif
        } else {
            // Entering OFF phase
            stop_motor_keep_modes();
            twist_dry_cnt        = 0;
            twist_phase_deadline = tnow + (uint32_t)twistSettings.offDurationSeconds * 1000UL;
        }
    } else {
        // 5) Maintain current phase outputs until the deadline arrives
        if (twist_on_phase) {
            if (!Motor_GetStatus()) start_motor();
        } else {
            if (Motor_GetStatus())  stop_motor_keep_modes();
        }
    }
}

/* =======================
   SEARCH MODE
   ======================= */
typedef enum {
    SEARCH_GAP_WAIT = 0,
    SEARCH_PROBE,
    SEARCH_RUN
} SearchState;

static SearchState search_state = SEARCH_GAP_WAIT;
static uint32_t    search_deadline_ms = 0;
static uint8_t     search_dry_cnt = 0;

void ModelHandle_StartSearch(uint16_t gap_s, uint16_t probe_s)
{
    clear_all_modes();

    if (gap_s   == 0) gap_s   = 5;
    if (probe_s == 0) probe_s = 3;

    searchSettings.testingGapSeconds = gap_s;
    searchSettings.dryRunTimeSeconds = probe_s;

    searchSettings.searchActive = true;
    searchActive = true;

    stop_motor_keep_modes();
    search_state       = SEARCH_GAP_WAIT;
    search_deadline_ms = now_ms() + (uint32_t)searchSettings.testingGapSeconds * 1000UL;
    search_dry_cnt     = 0;
    ModelHandle_SaveCurrentStateToEEPROM();
}
void ModelHandle_StopSearch(void)
{
    searchSettings.searchActive = false;
    searchActive = false;
    search_state = SEARCH_GAP_WAIT;
    stop_motor_keep_modes();
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

    uint32_t nowt = now_ms();

    if (isTankFull()) {
        ModelHandle_StopAllModesAndMotor();
        return;
    }

    switch (search_state)
    {
        case SEARCH_GAP_WAIT:
            if (Motor_GetStatus()) stop_motor_keep_modes();
            if ((int32_t)(search_deadline_ms - nowt) <= 0) {
                start_motor(); // probe ON
                search_state       = SEARCH_PROBE;
                search_deadline_ms = nowt + (uint32_t)searchSettings.dryRunTimeSeconds * 1000UL;
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
            if ((int32_t)(search_deadline_ms - nowt) <= 0) {
                // Still dry after probe
                stop_motor_keep_modes();
                search_state       = SEARCH_GAP_WAIT;
                search_deadline_ms = nowt + (uint32_t)searchSettings.testingGapSeconds * 1000UL;
            }
            break;

        case SEARCH_RUN:
            if (isDryDebounced_RUN()) {
                ModelHandle_StopAllModesAndMotor(); // terminal dry -> clear modes
                break;
            }
            if (!Motor_GetStatus()) start_motor(); // keep asserted
            break;
    }
}

/* =======================
   TIMER (RTC based)
   ======================= */


void ModelHandle_SetTimerSlot(uint8_t i,
                              uint8_t onH, uint8_t onM,
                              uint8_t offH, uint8_t offM,
                              bool en)
{
    if (i < 5)
    {
        timerSlots[i].enabled = en;
        timerSlots[i].onHour = onH;
        timerSlots[i].onMinute = onM;
        timerSlots[i].offHour = offH;
        timerSlots[i].offMinute = offM;
    }
}

void ModelHandle_ProcessTimerSlots(void)
{
    RTC_GetTimeDate();  // update 'time' from RTC

    uint16_t nowM = time.hour * 60 + time.minutes;
    bool motorShouldRun = false;

    for (uint8_t i = 0; i < 5; i++)
    {
        if (!timerSlots[i].enabled)
            continue;

        uint16_t onM  = timerSlots[i].onHour * 60 + timerSlots[i].onMinute;
        uint16_t offM = timerSlots[i].offHour * 60 + timerSlots[i].offMinute;

        if (onM < offM)
        {
            if (nowM >= onM && nowM < offM)
                motorShouldRun = true;
        }
        else  // handle midnight wrap-around (e.g., 23:00 → 02:00)
        {
            if (nowM >= onM || nowM < offM)
                motorShouldRun = true;
        }
    }

    ModelHandle_SetMotor(motorShouldRun);
}

static uint32_t seconds_since_midnight(void)
{
    RTC_GetTimeDate();
    return ((uint32_t)time.hour * 3600UL) +
           ((uint32_t)time.minutes * 60UL) +
           (uint32_t)time.seconds;
}
/* =========================
   TIMER MODE
   ========================= */
void timer_tick(void)
{
    if (!timerActive) return;

    RTC_GetTimeDate();
    uint32_t nowSec = ModelHandle_TimeToSeconds(time.hour, time.minutes);

    bool shouldRun = false;
    for (int i = 0; i < 5; i++) {
        if (!timerSlots[i].enabled) continue;

        uint32_t onSec  = ModelHandle_TimeToSeconds(timerSlots[i].onHour,  timerSlots[i].onMinute);
        uint32_t offSec = ModelHandle_TimeToSeconds(timerSlots[i].offHour, timerSlots[i].offMinute);

        if (onSec < offSec) {
            if (nowSec >= onSec && nowSec < offSec) shouldRun = true;
        } else { // overnight slot (e.g. 23:00–05:00)
            if (nowSec >= onSec || nowSec < offSec) shouldRun = true;
        }
    }

    if (shouldRun && !Motor_GetStatus()) {
        start_motor();
    } else if (!shouldRun && Motor_GetStatus()) {
        stop_motor_keep_modes();
    }
}
void ModelHandle_StartTimer(void) {
    clear_all_modes();
    timerActive = true;
    ModelHandle_SaveCurrentStateToEEPROM();
}

void ModelHandle_StopTimer(void) {
    timerActive = false;
    stop_motor_keep_modes();
}


/* =======================
   SEMI-AUTO
   ======================= */
void ModelHandle_StartSemiAuto(void)
{
    clear_all_modes();
    semiAutoActive = true;

    if (!isTankFull()) {
        start_motor();
        //printf("Semi-Auto Started\r\n");
    } else {
        ModelHandle_StopAllModesAndMotor();
        //printf("Semi-Auto Not Started: Already Full\r\n");
    }
    ModelHandle_SaveCurrentStateToEEPROM();
}
void ModelHandle_StopSemiAuto(void)
{
    semiAutoActive   = false;
    ModelHandle_StopAllModesAndMotor();
    //printf("Semi-Auto Stopped\r\n");
}
static void semi_auto_tick(void)
{
    if (!semiAutoActive) return;

    // Only complete on full; keep asserting motor otherwise
    if (isTankFull()) {
        semiAutoActive = false;
        ModelHandle_StopAllModesAndMotor();
        //printf("Semi-Auto Complete: Tank Full\r\n");
        return;
    }

    // Keep motor ON during semi-auto (protections may still cut it)
    if (!Motor_GetStatus()) start_motor();
}

/* =======================
   PROTECTIONS
   ======================= */
static void protections_tick(void)
{
    /* Manual override → only hard protections */
    if (manualOverride && manualActive) {
        if (senseOverLoad && motorStatus == 1U) ModelHandle_StopAllModesAndMotor();
        if (senseOverUnderVolt) ModelHandle_StopAllModesAndMotor();
        if (maxRunTimerArmed && (now_ms() - maxRunStartTick) >= MAX_CONT_RUN_MS) {
            ModelHandle_StopAllModesAndMotor();
            senseMaxRunReached = true;
        }
        return;
    }

    /* DRY-RUN: delayed stop */
    if (motorStatus == 1U) {
        if (dry_raw_is_dry()) {
            senseDryRun = true; // show on UI while counting
            if (!dryTimerArmed) {
                dryTimerArmed   = true;
                dryStopDeadline = now_ms() + (uint32_t)DRY_STOP_DELAY_SECONDS * 1000UL;
            } else {
                if ((int32_t)(dryStopDeadline - now_ms()) <= 0) {
                    ModelHandle_StopAllModesAndMotor();
                    // Keep flag true until user action clears (or water returns and motor restarts)
                }
            }
        } else {
            // water found => cancel dry timer & flag
            dryTimerArmed   = false;
            dryStopDeadline = 0;
            senseDryRun     = false;
        }
    } else {
        // motor is off; reset dry timing
        dryTimerArmed   = false;
        dryStopDeadline = 0;
        // keep senseDryRun as-is for UI; it will clear on next water detection
    }

    /* Other protections -> hard clear */
    if (senseOverLoad && motorStatus == 1U) ModelHandle_StopAllModesAndMotor();
    if (senseOverUnderVolt) ModelHandle_StopAllModesAndMotor();

    if (maxRunTimerArmed && (now_ms() - maxRunStartTick) >= MAX_CONT_RUN_MS) {
        ModelHandle_StopAllModesAndMotor();
        senseMaxRunReached = true;
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
    if (on) {
        clear_all_modes();         // turning ON directly should not have stray modes
        manualOverride = true;
        start_motor();
    } else {
        ModelHandle_StopAllModesAndMotor();
    }
}
void ModelHandle_ClearManualOverride(void) { manualOverride = false; }
bool Motor_GetStatus(void) { return (motorStatus == 1U); }

/* =======================
   Main pump
   ======================= */
void ModelHandle_Process(void)
{
    /* keep your original order of tickers */
	if (twistActive)    twist_tick();
	if (countdownActive) countdown_tick();
	if (searchActive)   search_tick();
	if (timerActive)    timer_tick();   // ✅ add this
    semi_auto_tick();

    /* --- Aux Burst auto-OFF (Relays 2 & 3) --- */
    if (auxBurstActive) {
        uint32_t nowt = HAL_GetTick();
        if ((int32_t)(auxBurstDeadlineMs - nowt) <= 0) {
            Relay_Set(2, false);
            Relay_Set(3, false);
            auxBurstActive = false;
        }
    }

    protections_tick();
    leds_from_model();
}

/* =======================
   UART commands
   ======================= */
void ModelHandle_ProcessUartCommand(const char* cmd)
{
    if (!cmd || !*cmd) return;

    if      (strcmp(cmd, "MOTOR_ON") == 0)  { ModelHandle_SetMotor(true); } //printf("Manual ON (UART)\r\n"); }
    else if (strcmp(cmd, "MOTOR_OFF")== 0)  { ModelHandle_SetMotor(false);} //printf("Manual OFF (UART)\r\n"); }
    else if (strcmp(cmd, "SEMI_AUTO_START")==0) { ModelHandle_StartSemiAuto(); }
    else if (strcmp(cmd, "SEMI_AUTO_STOP")==0)  { ModelHandle_StopSemiAuto(); }
    else if (strcmp(cmd, "TWIST_START")==0)     { ModelHandle_StartTwist(twistSettings.onDurationSeconds, twistSettings.offDurationSeconds); }
    else if (strcmp(cmd, "TWIST_STOP")==0)      { ModelHandle_StopTwist(); }
    else if (strcmp(cmd, "SEARCH_START")==0)    { ModelHandle_StartSearch(searchSettings.testingGapSeconds, searchSettings.dryRunTimeSeconds); }
    else if (strcmp(cmd, "SEARCH_STOP")==0)     { ModelHandle_StopSearch(); }
    else if (strcmp(cmd, "RESET_ALL")==0)       { ModelHandle_ResetAll(); }
}
// --- Lightweight persistent save ---
// ⚙️ Requires: RTC_SavePersistentState() from rtc_i2c.c
// ⚙️ Safe for 24C32 / 24C02 EEPROM (4-Kbit+)

void ModelHandle_SaveCurrentStateToEEPROM(void)
{
    static uint32_t lastWriteTick = 0;
    static uint8_t  lastCRC = 0;        // 1-byte change fingerprint
    uint32_t now = HAL_GetTick();

    // 🧭 Limit writes: once every 15 s
    if (now - lastWriteTick < 15000U)
        return;

    // 🧱 Allocate statically once (no stack each call)
    static RTC_PersistState s;

    // Manual zeroing (tiny, no memset())
    for (uint16_t i = 0; i < sizeof(s); i++)
        ((uint8_t*)&s)[i] = 0;

    // === Minimal state capture ===
    if (manualActive)        s.mode = 1;
    else if (semiAutoActive) s.mode = 2;
    else if (timerActive)    s.mode = 3;
    else if (searchActive)   s.mode = 4;
    else if (countdownActive)s.mode = 5;
    else if (twistActive)    s.mode = 6;
    else                     s.mode = 0;

    s.motor        = Motor_GetStatus();
    s.searchGap    = searchSettings.testingGapSeconds;
    s.searchProbe  = searchSettings.dryRunTimeSeconds;
    s.twistOn      = twistSettings.onDurationSeconds;
    s.twistOff     = twistSettings.offDurationSeconds;
    s.countdownMin = (uint16_t)(countdownDuration / 60U);
    s.countdownRep = 1U;

    // ⚙️ Copy only first timer slot (8–10 B)
    extern TimerSlot timerSlots[5];
    for (uint8_t i = 0; i < sizeof(TimerSlot); i++)
        ((uint8_t*)&s.timerSlots[0])[i] = ((uint8_t*)&timerSlots[0])[i];

    // === Compact 8-bit CRC-like fingerprint for quick diff ===
    uint8_t crc = 0;
    const uint8_t *p = (const uint8_t*)&s;
    for (uint16_t i = 0; i < sizeof(s) - 2; i++)
        crc ^= p[i];

    // Write only if changed since last save
    if (crc != lastCRC)
    {
        RTC_SavePersistentState(&s);  // actual I2C write
        lastCRC = crc;
        lastWriteTick = now;
    }
}
