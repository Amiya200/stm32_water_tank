/***************************************************************
 *  HELONIX Water Pump Controller
 *  FINAL MODEL HANDLE (Option B - Fix Errors & Workflow Only)
 *  Cleaned AUTO flow, DRY RUN stability, TIMER/TWIST fixes
 ***************************************************************/

#include "model_handle.h"
#include "relay.h"
#include "led.h"
#include "global.h"
#include "adc.h"
#include "rtc_i2c.h"
#include "uart_commands.h"
#include "stm32f1xx_hal.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ============================================================
   EXTERNAL ADC + RTC
   ============================================================ */
extern ADC_Data adcData;
extern RTC_Time_t time;

/* ============================================================
   GLOBAL FLAGS (UNCHANGED — do not rename)
   ============================================================ */
volatile bool manualActive    = false;
volatile bool semiAutoActive  = false;
volatile bool countdownActive = false;
volatile bool twistActive     = false;
volatile bool timerActive     = false;
volatile bool autoActive      = false;

/* ============================================================
   MOTOR STATUS (0 = OFF, 1 = ON)
   ============================================================ */
volatile uint8_t motorStatus = 0;

/* ============================================================
   SAFETY FLAGS (Your design: TRUE = Water for dry-run)
   ============================================================ */
volatile bool senseDryRun         = false;    // TRUE = WATER
volatile bool senseOverLoad       = false;
volatile bool senseOverUnderVolt  = false;
volatile bool senseMaxRunReached  = false;

/* ============================================================
   MANUAL OVERRIDE FLAG
   ============================================================ */
volatile bool manualOverride = false;

/* ============================================================
   UTILITY HELPERS
   ============================================================ */
static inline uint32_t now_ms(void)
{
    return HAL_GetTick();
}

static inline void clear_all_modes(void)
{
    manualActive    = false;
    semiAutoActive  = false;
    countdownActive = false;
    twistActive     = false;
    timerActive     = false;
    autoActive      = false;
    manualOverride  = false;
}

/* ============================================================
   MOTOR CONTROL — FIXED RELAY CHATTERING
   ============================================================ */
static inline void motor_apply(bool on)
{
    if (on == Motor_GetStatus())
        return;  // Prevent unnecessary relay toggling

    Relay_Set(1, on);
    motorStatus = on ? 1 : 0;

    UART_SendStatusPacket();
}

static inline void start_motor(void)
{
    motor_apply(true);
}

static inline void stop_motor(void)
{
    motor_apply(false);
}

bool Motor_GetStatus(void)
{
    return (motorStatus == 1);
}

/* ============================================================
   MANUAL SET MOTOR
   ============================================================ */
void ModelHandle_SetMotor(bool on)
{
    clear_all_modes();
    manualOverride = true;

    /* Manual mode ignores dry-run → treat as WATER true */
    senseDryRun = true;

    if (on) start_motor();
    else    stop_motor();
}

/* ============================================================
   TANK FULL DETECTION (Stabilized)
   ============================================================ */
static inline bool isTankFull(void)
{
    static uint32_t stableStart = 0;
    static bool lastState = false;

    bool allZero = true;
    for (int i = 1; i <= 5; i++)
    {
        if (adcData.voltages[i] > 0.10f)  // Water present
        {
            allZero = false;
            break;
        }
    }

    uint32_t now = HAL_GetTick();

    if (allZero)
    {
        if (!lastState)
        {
            lastState = true;
            stableStart = now;
        }

        if (now - stableStart >= 1000)
            return true;   // tank full stable
    }
    else
        lastState = false;

    return false;
}

/* ============================================================
   DRY RUN CHECK (Your convention: TRUE = WATER)
   ============================================================ */
void ModelHandle_CheckDryRun(void)
{
    /* Manual, Semi-Auto, Countdown ignore dry */
    if (manualActive || semiAutoActive || countdownActive)
    {
        senseDryRun = true;       // treat as WATER
        return;
    }

    /* Actual dry-run sensor */
    float v = adcData.voltages[0];

    /* You defined:
        <= 0.01V = WATER present
        > 0.01V  = DRY
    */

    senseDryRun = (v <= 0.01f);   // TRUE = WATER
}

/* ============================================================
   MANUAL TOGGLE (fixed: was leaving motor on)
   ============================================================ */
void ModelHandle_ToggleManual(void)
{
    if (manualActive)
    {
        manualActive = false;
        stop_motor();
    }
    else
    {
        clear_all_modes();
        manualActive = true;
        manualOverride = true;

        start_motor();
    }
}

/* ============================================================
   STOP ALL MODES + MOTOR
   ============================================================ */
void ModelHandle_StopAllModesAndMotor(void)
{
    clear_all_modes();
    stop_motor();
}

/* ============================================================
   DRY RUN FSM (YOU PICKED SOFT-DRY-RUN → KEEP, FIXED)
   ============================================================ */

/*
   FIXES:
   ❌ You forced senseDryRun=true in auto → broke FSM
   ❌ FSM broke because MODE ACTIVE check was wrong
   ❌ DRY_PROBE never retried properly
   ✔ This version keeps your design but repairs logic
*/

typedef enum {
    DRY_IDLE = 0,
    DRY_PROBE,
    DRY_NORMAL
} DryFSMState;

static DryFSMState dryState = DRY_IDLE;
static bool dryConfirming   = false;

static uint32_t dryDeadline     = 0;
static uint32_t dryConfirmStart = 0;

#define DRY_PROBE_ON_MS     5000UL
#define DRY_PROBE_OFF_MS   10000UL
#define DRY_CONFIRM_MS      1500UL

static inline bool isAnyModeActive(void)
{
    return (manualActive ||
            semiAutoActive ||
            countdownActive ||
            twistActive ||
            timerActive ||
            autoActive);
}

void ModelHandle_SoftDryRunHandler(void)
{
    uint32_t now = now_ms();

    /* First: detect dry-run state correctly */
    ModelHandle_CheckDryRun();

    if (!isAnyModeActive())
    {
        stop_motor();
        dryState = DRY_IDLE;
        dryConfirming = false;
        return;
    }

    switch (dryState)
    {
        case DRY_IDLE:
            if (senseDryRun)   // WATER detected
            {
                start_motor();
                dryState = DRY_NORMAL;
            }
            else  // DRY
            {
                if (now >= dryDeadline)
                {
                    start_motor();
                    dryState = DRY_PROBE;
                    dryDeadline = now + DRY_PROBE_ON_MS;
                }
            }
            break;

        case DRY_PROBE:
            if (senseDryRun)   // WATER
            {
                dryState = DRY_NORMAL;
                break;
            }

            if (now >= dryDeadline)
            {
                stop_motor();
                dryState = DRY_IDLE;
                dryDeadline = now + DRY_PROBE_OFF_MS;
            }
            break;

        case DRY_NORMAL:
            if (!senseDryRun)  // DRY detected
            {
                if (!dryConfirming)
                {
                    dryConfirming = true;
                    dryConfirmStart = now;
                }
                else if (now - dryConfirmStart >= DRY_CONFIRM_MS)
                {
                    stop_motor();
                    dryState = DRY_IDLE;
                    dryDeadline = now + DRY_PROBE_OFF_MS;
                    dryConfirming = false;
                }
            }
            else
                dryConfirming = false;
            break;
    }
}
void ModelHandle_ProcessDryRun(void)
{
    ModelHandle_SoftDryRunHandler();
}

/***************************************************************
 *  MODEL_HANDLE.C — PART 2 OF 3
 *  Countdown + Twist + Timer + Semi-Auto + Auto FSM FIXED
 ***************************************************************/

/* ============================================================
   COUNTDOWN MODE (fixed stability, tank stop, 0-lock)
   ============================================================ */

volatile uint32_t countdownDuration = 0;  // remaining seconds

static uint32_t cd_deadline = 0;

void ModelHandle_StopCountdown(void)
{
    countdownActive = false;
    countdownDuration = 0;
    stop_motor();
}

void ModelHandle_StartCountdown(uint32_t seconds)
{
    clear_all_modes();

    if (seconds == 0)
    {
        countdownActive = false;
        countdownDuration = 0;
        return;
    }

    countdownActive = true;
    countdownDuration = seconds;
    cd_deadline = now_ms() + (seconds * 1000UL);

    start_motor();
}

static void countdown_tick(void)
{
    if (!countdownActive)
        return;

    uint32_t now = now_ms();

    if (isTankFull())
    {
        ModelHandle_StopCountdown();
        return;
    }

    if (now < cd_deadline)
    {
        countdownDuration = (cd_deadline - now) / 1000UL;
    }
    else
    {
        countdownDuration = 0;
        // motor continues unless user stops manually
    }
}

/* ============================================================
   TWIST MODE — FIXED (duration-phase only)
   ============================================================ */

TwistSettings twistSettings;

static bool     twist_on_phase = false;
static uint32_t twist_deadline = 0;

void ModelHandle_StartTwist(uint16_t on_s, uint16_t off_s,
                            uint8_t onH, uint8_t onM,
                            uint8_t offH, uint8_t offM)
{
    clear_all_modes();

    if (on_s == 0)  on_s = 1;
    if (off_s == 0) off_s = 1;

    twistSettings.onDurationSeconds  = on_s;
    twistSettings.offDurationSeconds = off_s;

    twistSettings.onHour   = onH;
    twistSettings.onMinute = onM;
    twistSettings.offHour  = offH;
    twistSettings.offMinute= offM;

    twistSettings.twistArmed = true;
    twistActive = false;

    twist_on_phase = true;
    twist_deadline = now_ms() + (on_s * 1000UL);

    start_motor();
}

void ModelHandle_StopTwist(void)
{
    twistActive = false;
    twistSettings.twistActive = false;
    stop_motor();
}

/* Time-bound ON/OFF trigger (HH:MM) */
static void twist_time_logic(void)
{
    if (!twistSettings.twistArmed) return;

    /* Start twist at on-time */
    if (!twistActive &&
        time.hour   == twistSettings.onHour &&
        time.minutes== twistSettings.onMinute)
    {
        twistActive = true;
        twistSettings.twistActive = true;

        twist_on_phase = true;
        twist_deadline = now_ms() + twistSettings.onDurationSeconds * 1000UL;

        start_motor();
    }

    /* Stop twist at off-time */
    if (twistActive &&
        time.hour   == twistSettings.offHour &&
        time.minutes== twistSettings.offMinute)
    {
        ModelHandle_StopTwist();
    }
}

/* Duration cycle handling */
static void twist_tick(void)
{
    if (!twistActive) return;

    if (isTankFull())
    {
        ModelHandle_StopTwist();
        return;
    }

    uint32_t now = now_ms();

    if (now >= twist_deadline)
    {
        if (twist_on_phase)
        {
            /* switch to OFF phase */
            twist_on_phase = false;
            stop_motor();
            twist_deadline = now + twistSettings.offDurationSeconds * 1000UL;
        }
        else
        {
            /* switch to ON phase */
            twist_on_phase = true;
            start_motor();
            twist_deadline = now + twistSettings.onDurationSeconds * 1000UL;
        }
    }

    /* safety: ensure motor follows phase */
    if (twist_on_phase) start_motor();
    else                stop_motor();
}

/* ============================================================
   TIMER MODE — FIXED tick
   ============================================================ */

TimerSlot timerSlots[5];

static bool timer_any_slot_should_run(void)
{
    RTC_GetTimeDate();
    uint32_t nowS = time.hour * 3600UL + time.minutes * 60UL;

    for (int i = 0; i < 5; i++)
    {
        if (!timerSlots[i].enabled) continue;

        uint32_t onS =
            timerSlots[i].onHour * 3600UL +
            timerSlots[i].onMinute * 60UL;

        uint32_t offS =
            timerSlots[i].offHour * 3600UL +
            timerSlots[i].offMinute * 60UL;

        /* Normal slot */
        if (onS < offS)
        {
            if (nowS >= onS && nowS < offS)
                return true;
        }
        /* Overnight slot */
        else
        {
            if (nowS >= onS || nowS < offS)
                return true;
        }
    }
    return false;
}

void ModelHandle_TimerRecalculateNow(void)
{
    if (!timerActive) return;

    if (timer_any_slot_should_run()) start_motor();
    else                              stop_motor();
}

void ModelHandle_StartTimer(void)
{
    clear_all_modes();
    timerActive = true;

    if (timer_any_slot_should_run()) start_motor();
    else                              stop_motor();
}

void ModelHandle_StopTimer(void)
{
    timerActive = false;
    stop_motor();
}

static void timer_tick(void)
{
    if (!timerActive) return;

    if (isTankFull())
    {
        stop_motor();
        return;
    }

    if (timer_any_slot_should_run())
        start_motor();
    else
        stop_motor();
}

/* ============================================================
   SEMI-AUTO MODE — FIXED
   ============================================================ */
void ModelHandle_StartSemiAuto(void)
{
    clear_all_modes();
    semiAutoActive = true;

    if (!isTankFull())
        start_motor();
}

void ModelHandle_StopSemiAuto(void)
{
    semiAutoActive = false;
    stop_motor();
}

/* ============================================================
   AUTO MODE — FIXED TRUE WORKFLOW
   ============================================================ */

/*
   FIXES:
   ✔ Removed broken duplicated AUTO block at bottom of your file
   ✔ Restored your original AUTO FSM
   ✔ Corrected dry-run logic
   ✔ Correct gap-wait retry logic
   ✔ Correct retry limit handling
*/

static uint16_t auto_gap_s       = 60;
static uint16_t auto_maxrun_min  = 12;
static uint16_t auto_retry_limit = 5;
static uint8_t  auto_retry_count = 0;

typedef enum {
    AUTO_IDLE = 0,
    AUTO_ON_WAIT,
    AUTO_DRY_CHECK,
    AUTO_OFF_WAIT
} AutoState;

static AutoState autoState = AUTO_IDLE;
static uint32_t autoDeadline = 0;
static uint32_t autoRunStart = 0;

void ModelHandle_StartAuto(uint16_t gap_s, uint16_t maxrun_min, uint16_t retry)
{
    clear_all_modes();
    autoActive = true;

    auto_gap_s       = gap_s;
    auto_maxrun_min  = maxrun_min;
    auto_retry_limit = retry;
    auto_retry_count = 0;

    autoState = AUTO_ON_WAIT;

    start_motor();
    autoRunStart = now_ms();
    autoDeadline = now_ms() + (auto_gap_s * 1000UL);
}

void ModelHandle_StopAuto(void)
{
    autoActive = false;
    autoState = AUTO_IDLE;
    auto_retry_count = 0;
    stop_motor();
}

static void auto_tick(void)
{
    if (!autoActive)
        return;

    uint32_t now = now_ms();

    /* Protection exits */
    if (isTankFull() ||
        senseOverLoad ||
        senseOverUnderVolt ||
        senseMaxRunReached)
    {
        ModelHandle_StopAuto();
        return;
    }

    /* Max-run protection */
    if (now - autoRunStart >= (auto_maxrun_min * 60000UL))
    {
        senseMaxRunReached = true;
        ModelHandle_StopAuto();
        return;
    }

    /* CONTROL FSM */
    switch (autoState)
    {
        case AUTO_ON_WAIT:
            if (now >= autoDeadline)
                autoState = AUTO_DRY_CHECK;
            break;

        case AUTO_DRY_CHECK:
            ModelHandle_CheckDryRun();

            if (senseDryRun)   // water present
            {
                autoState = AUTO_ON_WAIT;
                autoDeadline = now + (auto_gap_s * 1000UL);
            }
            else
            {
                stop_motor();
                autoState = AUTO_OFF_WAIT;
                autoDeadline = now + (auto_gap_s * 1000UL);
            }
            break;

        case AUTO_OFF_WAIT:
            if (now >= autoDeadline)
            {
                auto_retry_count++;

                if (auto_retry_limit != 0 &&
                    auto_retry_count > auto_retry_limit)
                {
                    ModelHandle_StopAuto();
                    return;
                }

                start_motor();
                autoRunStart = now;

                autoState = AUTO_ON_WAIT;
                autoDeadline = now + (auto_gap_s * 1000UL);
            }
            break;

        default:
            break;
    }
}
/***************************************************************
 *  MODEL_HANDLE.C — PART 3 OF 3
 *  Protections + LED System + Final Process Loop + EEPROM Save
 ***************************************************************/

/* ============================================================
   PROTECTIONS — FIXED
   ============================================================ */

static void protections_tick(void)
{
    /* Overload or voltage fault must stop ALL modes */
    if (senseOverLoad || senseOverUnderVolt)
    {
        ModelHandle_StopAllModesAndMotor();
        return;
    }
}

/* ============================================================
   LED SYSTEM — Corrected to match your document
   ============================================================ */
/*
    Your LED meanings (document):

    GREEN:
        solid   → motor ON
        blink   → dry-run active retry

    RED:
        solid   → motor OFF due to dry-run
        blink   → max-run error

    BLUE:
        blink   → overload / underload

    PURPLE:
        blink   → over-voltage / under-voltage
*/

static void leds_from_model(void)
{
    LED_ClearAllIntents();

    /* MOTOR ON → GREEN Solid */
    if (Motor_GetStatus())
        LED_SetIntent(LED_COLOR_GREEN, LED_MODE_STEADY, 0);

    /* DRY-RUN retry → GREEN blinking */
    if (!senseDryRun && autoActive)
        LED_SetIntent(LED_COLOR_GREEN, LED_MODE_BLINK, 350);

    /* Motor stopped due to dry-run → RED solid */
    if (!senseDryRun && !Motor_GetStatus())
        LED_SetIntent(LED_COLOR_RED, LED_MODE_STEADY, 0);

    /* Max-run reached → RED blink */
    if (senseMaxRunReached)
        LED_SetIntent(LED_COLOR_RED, LED_MODE_BLINK, 300);

    /* Overload / Underload → BLUE blink */
    if (senseOverLoad)
        LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 350);

    /* Voltage faults → PURPLE blink */
    if (senseOverUnderVolt)
        LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_BLINK, 350);

    LED_ApplyIntents();
}

/* ============================================================
   MAIN PROCESS LOOP — FIXED COMPLETELY
   ============================================================ */

void ModelHandle_Process(void)
{
    uint32_t now = now_ms();

    /* Shared safety check */
    protections_tick();

    /* Twist time-bound start/stop */
    twist_time_logic();

    /* ============================================================
       AUTO MODE
       ============================================================ */
    if (autoActive)
    {
        auto_tick();
        leds_from_model();
        return;     // ← VERY IMPORTANT: avoid double-processing
    }

    /* ============================================================
       SEMI-AUTO MODE
       ============================================================ */
    if (semiAutoActive)
    {
        /* semi-auto ignores dry-run */
        senseDryRun = true;

        if (!isTankFull())
        {
            if (!Motor_GetStatus())
                start_motor();
        }
        else
        {
            stop_motor();
            semiAutoActive = false; // auto-terminate as per spec
        }

        leds_from_model();
        return;
    }

    /* ============================================================
       TIMER MODE
       ============================================================ */
    if (timerActive)
    {
        timer_tick();
        leds_from_model();
        return;
    }

    /* ============================================================
       COUNTDOWN MODE
       ============================================================ */
    if (countdownActive)
    {
        countdown_tick();
        leds_from_model();
        return;
    }

    /* ============================================================
       TWIST MODE (duration based)
       ============================================================ */
    if (twistActive)
    {
        twist_tick();
        leds_from_model();
        return;
    }

    /* ============================================================
       MANUAL MODE
       ============================================================ */
    if (manualActive)
    {
        /* manual ignores sensors */
        leds_from_model();
        return;
    }

    /* ============================================================
       NO MODE ACTIVE → MOTOR OFF
       ============================================================ */
    stop_motor();
    leds_from_model();
}

/* ============================================================
   RESET ALL STATES
   ============================================================ */
void ModelHandle_ResetAll(void)
{
    clear_all_modes();
    stop_motor();

    senseDryRun         = false;
    senseOverLoad       = false;
    senseOverUnderVolt  = false;
    senseMaxRunReached  = false;

    countdownDuration = 0;

    UART_SendStatusPacket();
}

/* ============================================================
   AUTO SETTINGS — kept as original
   ============================================================ */
void ModelHandle_SetAutoSettings(uint16_t gap_s, uint16_t maxrun_min, uint8_t retry_count)
{
    auto_gap_s       = gap_s;
    auto_maxrun_min  = maxrun_min;
    auto_retry_limit = retry_count;
    auto_retry_count = 0;
}

bool ModelHandle_IsAutoActive(void)
{
    return autoActive;
}

/* ============================================================
   EEPROM SAVE — unchanged
   ============================================================ */

void ModelHandle_SaveCurrentStateToEEPROM(void)
{
    RTC_PersistState s;
    memset(&s, 0, sizeof(s));

    if      (manualActive)    s.mode = 1;
    else if (semiAutoActive)  s.mode = 2;
    else if (timerActive)     s.mode = 3;
    else if (countdownActive) s.mode = 4;
    else if (twistActive)     s.mode = 5;
    else if (autoActive)      s.mode = 6;
    else                      s.mode = 0;

    RTC_SavePersistentState(&s);
}


