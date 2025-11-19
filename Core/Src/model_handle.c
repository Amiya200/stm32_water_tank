#include "model_handle.h"
#include "relay.h"
#include "led.h"
#include "global.h"
#include "adc.h"
#include "rtc_i2c.h"
#include "uart_commands.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

/* ============================================================
   DRY-RUN POLARITY (YOU CONFIRMED)
   TRUE  → WATER PRESENT
   FALSE → DRY
   ============================================================ */

#ifndef DRY_THRESHOLD_V
#define DRY_THRESHOLD_V 0.10f
#endif

/* DRY soft-FSM timings */
#define DRY_PROBE_ON_MS   5000UL    // motor on for test (5s)
#define DRY_PROBE_OFF_MS  10000UL   // motor off between tests (10s)
#define DRY_CONFIRM_MS    1500UL    // confirm water loss before shutting down

/* General max-run protection */
static uint32_t MAX_CONT_RUN_MS = 2UL * 60UL * 60UL * 1000UL;


/* ============================================================
   GLOBAL FLAGS
   ============================================================ */

extern ADC_Data adcData;
extern RTC_Time_t time;

volatile uint8_t motorStatus = 0;

volatile bool manualActive    = false;
volatile bool semiAutoActive  = false;
volatile bool countdownActive = false;
volatile bool twistActive     = false;
volatile bool timerActive     = false;

volatile bool senseDryRun         = false;  // TRUE = water present
volatile bool senseOverLoad       = false;
volatile bool senseOverUnderVolt  = false;
volatile bool senseMaxRunReached  = false;
volatile bool autoActive = false;

static uint32_t auto_gap_ms = 10000;         // 10 sec default
static uint32_t auto_next_retry_time = 0;
static uint32_t auto_retry_counter = 0;
static uint32_t auto_retry_limit = 0;        // 0 = infinite retries
typedef struct {
    uint16_t dryRunThreshold;
    uint16_t overloadCurrent;
    uint16_t underloadCurrent;
    uint16_t overVoltage;
    uint16_t underVoltage;
} SafetySettings;

extern SafetySettings safety;

/* manual override (MOTOR_ON/MOTOR_OFF manual push) */
volatile bool manualOverride = false;
/* ============================================================
   UTILS
   ============================================================ */
static inline uint32_t now_ms(void)
{
    return HAL_GetTick();
}

static inline void clear_all_modes(void)
{
    manualActive = false;
    semiAutoActive = false;
    countdownActive = false;
    twistActive = false;
    timerActive = false;
    manualOverride = false;
}
static void Safe_SendStatusPacket(void)
{
    static uint32_t last = 0;
    uint32_t now = now_ms();

    if (now - last >= 3000)
    {
        UART_SendStatusPacket();
        last = now;
    }
}
/* ============================================================
   MOTOR CONTROL CORE
   ============================================================ */

static inline void motor_apply(bool on)
{
    Relay_Set(1, on);
    motorStatus = on ? 1 : 0;

    static uint32_t maxRunStartTick = 0;
    static bool maxRunArmed = false;

    if (on)
    {
        if (!maxRunArmed)
        {
            maxRunArmed = true;
            maxRunStartTick = now_ms();
        }
    }
    else
    {
        maxRunArmed = false;
    }

    Safe_SendStatusPacket();
}
void Motor_ImmediateOff(void)
{
	motor_apply(false);
}

void Motor_ImmediateOn(void)
{
	motor_apply(true);
}

static inline void start_motor(void)
{
    motor_apply(true);
}

static inline void stop_motor_keep_modes(void)
{
    motor_apply(false);
}

bool Motor_GetStatus(void)
{
    return (motorStatus == 1U);
}

void ModelHandle_SetMotor(bool on)
{
    clear_all_modes();
    manualOverride = true;
    senseDryRun = true;     // ignore dry-run always

    if (on) start_motor();
    else    stop_motor_keep_modes();
}
/* ============================================================
   DRY SENSOR CHECK (YOU CONFIRMED LOGIC)
   TRUE  => WATER
   FALSE => DRY
   ============================================================ */
void ModelHandle_CheckDryRun(void)
{
    /* Ignore dry-run completely during countdown */
    if (manualActive || semiAutoActive || countdownActive) {
        senseDryRun = true;   // force WATER PRESENT
        return;
    }

    float v = adcData.voltages[0];

    if (v <= 0.01f)
        senseDryRun = true;
    else
        senseDryRun = false;
}




/* Pure helper */
static inline bool isTankFull(void)
{
    int submerged = 0;
    for (int i = 0; i < 5; i++)
    {
        if (adcData.voltages[i] < 0.1f)
            submerged++;
    }
    return (submerged >= 4);
}

/* ============================================================
   A small safe UART status sender
   ============================================================ */

/* ============================================================
   SOFT DRY-RUN STATE MACHINE (FULLY UPDATED)
   ------------------------------------------------------------
   ============================================================ */

typedef enum {
    DRY_IDLE = 0,    // motor OFF waiting for next probe
    DRY_PROBE,       // motor ON short probe
    DRY_NORMAL       // normal running (water continuously present)
} DryFSMState;
static bool auxBurstActive = false;
static uint32_t auxBurstDeadline = 0;

static DryFSMState dryState = DRY_IDLE;
static uint32_t dryDeadline = 0;
static uint32_t dryConfirmStart = 0;
static bool dryConfirming = false;

void ModelHandle_ToggleManual(void)
{
    /* Turning ON manual mode */
    if (!manualActive)
    {
        clear_all_modes();          // stop all other modes
        manualActive = true;

        manualOverride = true;      // manual control is the top override
        senseDryRun = true;         // ignore dry-run sensor completely

        start_motor();              // PURE direct ON
    }
    else
    {
        /* Turning OFF manual mode */
        manualActive = false;
        manualOverride = false;
        senseDryRun = true;         // stay safe (ignore dry-run)

        stop_motor_keep_modes();    // PURE direct OFF
    }
}



void ModelHandle_StopAllModesAndMotor(void)
{
    clear_all_modes();
    manualOverride = false;
    senseDryRun = true;     // safe
    stop_motor_keep_modes();
}
void ModelHandle_ClearManualOverride(void)
{
    manualOverride = false;
}
void ModelHandle_TriggerAuxBurst(uint16_t seconds)
{
    if (seconds == 0) seconds = 1;

    Relay_Set(2, true);
    Relay_Set(3, true);

    auxBurstActive = true;
    auxBurstDeadline = now_ms() + seconds * 1000UL;
}
void ModelHandle_ManualLongPress(void)
{
    HAL_Delay(200);
    NVIC_SystemReset();
}

/* Return TRUE if ANY mode is active */
static inline bool isAnyModeActive(void)
{
    return (manualActive ||
            semiAutoActive ||
            countdownActive ||
            twistActive ||
            timerActive);
}
void ModelHandle_SoftDryRunHandler(void)
{
    uint32_t now = now_ms();

    /* Always refresh dry-run sensor */
    ModelHandle_CheckDryRun();  // TRUE = WATER, FALSE = DRY


    /* Skip global dry-run if TIMER mode active */
    if (timerActive)
        return;

    /* If no mode active → motor must be OFF */
    if (!isAnyModeActive())
    {
        stop_motor_keep_modes();
        dryState = DRY_IDLE;
        dryConfirming = false;
        return;
    }


    switch (dryState)
    {
        /* ========================================================
           DRY_IDLE → motor OFF, waiting to check water
           ======================================================== */
        case DRY_IDLE:

            /* Keep motor OFF ONLY IF no mode wants it ON */
            if (!Motor_GetStatus())
                stop_motor_keep_modes();

            /* If water is present immediately → GO NORMAL */
            if (senseDryRun == true)  // water present
            {
                /* If any mode requested ON, keep ON */
                if (!Motor_GetStatus())
                    start_motor();

                dryState = DRY_NORMAL;
                dryConfirming = false;
                break;
            }

            /* No water → perform probe cycle */
            if ((int32_t)(dryDeadline - now) <= 0)
            {
                /* Only probe if NO mode already started motor */
                if (!Motor_GetStatus())
                    start_motor();

                dryState = DRY_PROBE;
                dryDeadline = now + DRY_PROBE_ON_MS;
                dryConfirming = false;
            }
            break;

        /* ========================================================
           DRY_PROBE → motor ON for short test interval
           ======================================================== */
        case DRY_PROBE:

            /* If water found anytime → NORMAL RUN */
            if (senseDryRun == true)
            {
                dryState = DRY_NORMAL;
                dryConfirming = false;

                /* ensure motor ON */
                if (!Motor_GetStatus())
                    start_motor();

                break;
            }

            /* Timeout & still dry → return to IDLE */
            if ((int32_t)(dryDeadline - now) <= 0)
            {
                /* Only turn OFF if mode did not force ON */
                if (!manualActive && !semiAutoActive)
                    stop_motor_keep_modes();

                dryState = DRY_IDLE;
                dryDeadline = now + DRY_PROBE_OFF_MS;
                dryConfirming = false;
            }
            break;

        /* ========================================================
           DRY_NORMAL → continuous run as long as water present
           ======================================================== */
        case DRY_NORMAL:

            /* Ensure motor ON (mode may force it ON) */
            if (!Motor_GetStatus())
                start_motor();

            /* Detect DRY condition */
            if (senseDryRun == false)   // DRY
            {
                if (!dryConfirming)
                {
                    dryConfirming = true;
                    dryConfirmStart = now;
                }

                /* Confirm DRY for safety duration */
                if ((int32_t)(now - dryConfirmStart) >= (int32_t)DRY_CONFIRM_MS)
                {
                    /* Stop ONLY if mode didn’t ask for ON */
                    if (!manualActive && !semiAutoActive)
                        stop_motor_keep_modes();

                    dryState = DRY_IDLE;
                    dryDeadline = now + DRY_PROBE_OFF_MS;
                    dryConfirming = false;
                }
            }
            else
            {
                /* Water restored → continue normally */
                dryConfirming = false;
            }
            break;
    }
}


void ModelHandle_ProcessDryRun(void)
{
    if (timerActive || countdownActive)
        return;

    ModelHandle_SoftDryRunHandler();
}

/* ============================================================
   COUNTDOWN MODE
   ------------------------------------------------------------
   Packet expected:
       @COUNTDOWN:ON:seconds#

   Fixes:
   ✔ Screen shows ON correctly
   ✔ Single run (no repeats unless you add)
   ✔ Fixed ON → REST cycle
   ✔ Stops if tank full
   ============================================================ */

volatile uint32_t countdownDuration = 0;
volatile bool countdownMode = false;
volatile uint16_t countdownRemainingRuns = 0;

static uint32_t cd_deadline_ms      = 0;
static uint32_t cd_rest_deadline_ms = 0;
static uint32_t cd_run_seconds      = 0;
static bool     cd_in_rest          = false;
static const uint32_t CD_REST_MS = 3000;   // 3 second rest

void ModelHandle_StopCountdown(void)
{
    countdownActive = false;
    countdownMode   = false;
    cd_in_rest      = false;
    countdownRemainingRuns = 0;
    countdownDuration = 0;

    stop_motor_keep_modes();
}

static void countdown_start_one_run(void)
{
    cd_deadline_ms = now_ms() + cd_run_seconds * 1000UL;
    cd_in_rest = false;
    countdownDuration = cd_run_seconds;

    start_motor();           // FIX — ensure ON status shows correctly
}

void ModelHandle_StartCountdown(uint32_t seconds)
{
    clear_all_modes();

    if (seconds == 0) {
        ModelHandle_StopCountdown();
        return;
    }

    cd_run_seconds = seconds;
    countdownRemainingRuns = 1;     // FORCE 1 RUN ONLY

    countdownActive = true;
    countdownMode   = true;

    cd_in_rest = false;
    cd_deadline_ms = now_ms() + (cd_run_seconds * 1000UL);

    countdownDuration = cd_run_seconds;
    start_motor();
}


/* tick */
static void countdown_tick(void)
{
    if (!countdownActive) return;

    uint32_t now = now_ms();

    /* Tank full safety */
    if (isTankFull()) {
        ModelHandle_StopCountdown();
        return;
    }

    /* Still running */
    if ((int32_t)(cd_deadline_ms - now) > 0) {
        uint32_t rem = cd_deadline_ms - now;
        countdownDuration = (rem + 999U) / 1000U;
        return;
    }

    /* Time over → stop everything */
    ModelHandle_StopCountdown();
}
/* ============================================================
   TWIST MODE
   ------------------------------------------------------------
   Packet:
       @TWIST:SET:on:off#
   ============================================================ */

TwistSettings twistSettings;

static bool     twist_on_phase = false;
static uint32_t twist_deadline = 0;

void ModelHandle_StartTwist(uint16_t on_s, uint16_t off_s,
                            uint8_t onH, uint8_t onM,
                            uint8_t offH, uint8_t offM)
{
    clear_all_modes();

    if (on_s == 0)  on_s  = 1;
    if (off_s == 0) off_s = 1;

    twistSettings.onDurationSeconds  = on_s;
    twistSettings.offDurationSeconds = off_s;

    twistSettings.onHour   = onH;
    twistSettings.onMinute = onM;
    twistSettings.offHour  = offH;
    twistSettings.offMinute= offM;


    twistSettings.twistArmed  = true;
    twistSettings.twistActive = false;
    twistActive = false;  // global flag

    /* Start ON-phase immediately */
    twist_on_phase = true;
    twist_deadline = now_ms() + (twistSettings.onDurationSeconds * 1000UL);

    start_motor();
}


void ModelHandle_StopTwist(void)
{
    twistSettings.twistActive = false;
    twistActive = false;

    stop_motor_keep_modes();
}

static void twist_tick(void)
{
    if (!twistActive) return;

    uint32_t now = now_ms();

    /* ===== Phase timeout reached ===== */
    if ((int32_t)(twist_deadline - now) <= 0)
    {
        if (twist_on_phase)
        {
            /* -------------------------
               ON → PHASE TRANSITION
               ------------------------- */

            if (senseDryRun == true)
            {
                /* WATER PRESENT:
                   Continue ON phase (stay ON)
                */
                twist_on_phase = true;
                start_motor();
                twist_deadline = now + twistSettings.onDurationSeconds * 1000UL;
            }
            else
            {
                /* DRY CONDITION:
                   Move to OFF phase
                */
                twist_on_phase = false;
                stop_motor_keep_modes();
                twist_deadline = now + twistSettings.offDurationSeconds * 1000UL;
            }
        }
        else
        {
            /* -------------------------
               OFF → ON TRANSITION
               ------------------------- */

            twist_on_phase = true;

            /* Motor must start ON BEFORE checking dry-run */
            start_motor();

            twist_deadline = now + twistSettings.onDurationSeconds * 1000UL;
        }
    }

    /* ===== ACTIVE PHASE CONTROL ===== */

    if (twist_on_phase)
    {
        /* ✔ ON duration:
           Motor must remain ON always
        */
        if (!Motor_GetStatus())
            start_motor();
    }
    else
    {
        /* ✔ OFF duration:
           Motor must remain OFF always
        */
        stop_motor_keep_modes();
    }
}



/* ============================================================
   TIMER MODE — FINAL, CLEAN, FULLY FIXED VERSION
   ============================================================ */

TimerSlot timerSlots[5];
void ModelHandle_TimerRecalculateNow(void)
{
    timerActive = false;    // default: timer OFF
    bool turnMotorOn = false;

    // Get current time
    uint8_t H = time.hour;
    uint8_t M = time.minutes;

    for (int i = 0; i < 5; i++)
    {
        if (!timerSlots[i].enabled)
            continue;

        uint8_t onH  = timerSlots[i].onHour;
        uint8_t onM  = timerSlots[i].onMinute;
        uint8_t offH = timerSlots[i].offHour;
        uint8_t offM = timerSlots[i].offMinute;

        // Convert time to minutes for easier comparison
        int nowMin  = H * 60 + M;
        int onMin   = onH * 60 + onM;
        int offMin  = offH * 60 + offM;

        // Normal slot
        if (onMin < offMin)
        {
            if (nowMin >= onMin && nowMin < offMin)
            {
                timerActive = true;
                turnMotorOn = true;
            }
        }
        else  // Overnight slot (e.g., 22:00 → 02:00)
        {
            if (nowMin >= onMin || nowMin < offMin)
            {
                timerActive = true;
                turnMotorOn = true;
            }
        }
    }

    // Apply motor decision
    if (turnMotorOn)
    {
        manualOverride = false;
        start_motor();
    }
    else
    {
        stop_motor_keep_modes();
    }
}

/* convert H:M → seconds since midnight */
static uint32_t toSec(uint8_t h, uint8_t m)
{
    return (uint32_t)h * 3600UL + (uint32_t)m * 60UL;
}

/* Set ONE slot manually (from UART or UI) */
void ModelHandle_SetTimerSlot(uint8_t slot,
                              uint8_t onH, uint8_t onM,
                              uint8_t offH, uint8_t offM)
{
    if (slot >= 5) return;

    TimerSlot* ts = &timerSlots[slot];

    ts->onHour    = onH;
    ts->onMinute  = onM;
    ts->offHour   = offH;
    ts->offMinute = offM;

    /* Auto-enable only if ON!=OFF */
    ts->enabled = !((onH == offH) && (onM == offM));
}

/* ============================================================
   Returns TRUE if ANY slot should turn the motor ON
   ============================================================ */
static bool timer_any_slot_should_run(void)
{
    RTC_GetTimeDate();  // update time struct
    uint32_t nowS = toSec(time.hour, time.minutes);

    for (int i = 0; i < 5; i++)
    {
        if (!timerSlots[i].enabled) continue;

        uint32_t on  = toSec(timerSlots[i].onHour,  timerSlots[i].onMinute);
        uint32_t off = toSec(timerSlots[i].offHour, timerSlots[i].offMinute);

        /* normal window: 10:00–14:00 */
        if (on < off)
        {
            if (nowS >= on && nowS < off)
                return true;
        }
        else
        {
            /* cross-midnight: 22:00–06:00 */
            if (nowS >= on || nowS < off)
                return true;
        }
    }

    return false;
}
void ModelHandle_StartAuto(void)
{
    clear_all_modes();

    autoActive = true;
    auto_retry_counter = 0;
    auto_next_retry_time = now_ms();

    // start motor only if tank is not full
    if (!isTankFull() && senseDryRun == true)
        start_motor();
}

void ModelHandle_StopAuto(void)
{
    autoActive = false;
    stop_motor_keep_modes();
}


/* ============================================================
   Recalculate schedule IMMEDIATELY after editing a slot
   ============================================================ */


/* ============================================================
   STOP TIMER MODE
   ============================================================ */
void ModelHandle_StopTimer(void)
{
    timerActive = false;
    stop_motor_keep_modes();
}

/* ============================================================
   START TIMER MODE
   ============================================================ */
void ModelHandle_StartTimer(void)
{
    clear_all_modes();
    timerActive = true;

    bool shouldRun = timer_any_slot_should_run();

    if (shouldRun)
        start_motor();
    else
        stop_motor_keep_modes();
}

/* ============================================================
   TIMER TICK — called in ModelHandle_Process()
   ============================================================ */
static void timer_tick(void)
{
	if (semiAutoActive)
	{
	    /* Always ON unless tank is full */
	    if (isTankFull())
	    {
	        stop_motor_keep_modes();
	        semiAutoActive = false;   // auto cut
	    }
	    else
	    {
	        start_motor();  // Always ON
	    }

	    /* Semi-auto must ignore dry sensor */
	    senseDryRun = true;
	}

    if (!timerActive)
        return;

    bool shouldRun = timer_any_slot_should_run();

    if (shouldRun)
    {
        if (!Motor_GetStatus())
            start_motor();
    }
    else
    {
        if (Motor_GetStatus())
            stop_motor_keep_modes();
    }
}

/* ============================================================
   SEMI AUTO
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
    stop_motor_keep_modes();
}

static void auto_tick(void)
{
    if (!autoActive) return;

    uint32_t now = now_ms();

    /* 1) Tank full → stop AUTO */
    if (isTankFull())
    {
        autoActive = false;
        stop_motor_keep_modes();
        return;
    }

    /* 2) Protection fault → stop AUTO */
    if (senseOverLoad || senseOverUnderVolt || senseMaxRunReached)
    {
        autoActive = false;
        stop_motor_keep_modes();
        return;
    }

    /* 3) If WATER PRESENT → keep ON */
    if (senseDryRun == true)
    {
        auto_retry_counter = 0;

        if (!Motor_GetStatus())
            start_motor();

        return;
    }

    /* 4) WATER NOT PRESENT → stop and retry later */
    stop_motor_keep_modes();

    /* Wait until retry time */
    if (now < auto_next_retry_time)
        return;

    /* Check retry limit */
    if (auto_retry_limit > 0 && auto_retry_counter >= auto_retry_limit)
    {
        autoActive = false;
        stop_motor_keep_modes();
        return;
    }

    /* Retry */
    auto_retry_counter++;
    auto_next_retry_time = now + (autoSettings.gap_seconds * 1000UL);

    start_motor();
}

void ModelHandle_SetAutoSettings(uint16_t gap_s, uint16_t maxrun_min, uint16_t retry)
{
    if (gap_s == 0) gap_s = 1;

    autoSettings.gap_seconds     = gap_s;
    autoSettings.maxrun_minutes  = maxrun_min;
    autoSettings.retry_limit     = retry;

    auto_gap_ms      = gap_s * 1000UL;
    auto_retry_limit = retry;
    MAX_CONT_RUN_MS  = (uint32_t)maxrun_min * 60UL * 1000UL;
}

void ModelHandle_ToggleAuto(void)
{
    if (autoActive)
        ModelHandle_StopAuto();
    else
        ModelHandle_StartAuto();
}

/* ============================================================
   CENTRAL PROTECTION SYSTEM
   ============================================================ */

static void protections_tick(void)
{
    static uint32_t maxRunStart = 0;
    static bool maxRunArmed = false;

    /* Overload / UnderVoltage — immediate hard stop */
    if (senseOverLoad || senseOverUnderVolt)
    {
        ModelHandle_StopAllModesAndMotor();
        return;
    }

    /* Max continuous run time */
    if (Motor_GetStatus() && !maxRunArmed)
    {
        maxRunStart = now_ms();
        maxRunArmed = true;
    }
    else if (!Motor_GetStatus())
    {
        maxRunArmed = false;
    }

    if (maxRunArmed &&
        (now_ms() - maxRunStart) >= MAX_CONT_RUN_MS)
    {
        senseMaxRunReached = true;
        ModelHandle_StopAllModesAndMotor();
    }
}

/* ============================================================
   LED SYSTEM (updated to match your logic)
   ============================================================ */

static void leds_from_model(void)
{
    LED_ClearAllIntents();

    /* GREEN = motor ON */
    if (Motor_GetStatus())
        LED_SetIntent(LED_COLOR_GREEN, LED_MODE_STEADY, 0);

    /* GREEN BLINK = countdown active */
    if (countdownActive)
        LED_SetIntent(LED_COLOR_GREEN, LED_MODE_BLINK, 400);

    /* RED = dry-run problem (motor wants ON but no water) */
    if (!senseDryRun && Motor_GetStatus())
        LED_SetIntent(LED_COLOR_RED, LED_MODE_STEADY, 0);

    /* BLUE = overload */
    if (senseOverLoad)
        LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 350);

    /* PURPLE = under/over voltage */
    if (senseOverUnderVolt)
        LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_BLINK, 350);

    /* RED BLINK = max runtime hit */
    if (senseMaxRunReached)
        LED_SetIntent(LED_COLOR_RED, LED_MODE_BLINK, 300);

    LED_ApplyIntents();
}

/* ============================================================
   MAIN PROCESS LOOP (THE HEART OF THE SYSTEM)
   ============================================================ */
static void twist_time_logic(void)
{
    if (!twistSettings.twistArmed)
        return;

    /* Activate at ON time */
    if (!twistActive &&
        time.hour == twistSettings.onHour &&
        time.minutes == twistSettings.onMinute)
    {
        twistActive = true;
        twistSettings.twistActive = true;

        twist_on_phase = true;
        twist_deadline = now_ms() + twistSettings.onDurationSeconds * 1000UL;

        start_motor();
    }

    /* Deactivate at OFF time */
    if (twistActive &&
        time.hour == twistSettings.offHour &&
        time.minutes == twistSettings.offMinute)
    {
        twistActive = false;
        twistSettings.twistActive = false;

        stop_motor_keep_modes();
    }
}


void ModelHandle_StartNearestTimer(void)
{
    /* If no slots enabled → do nothing */
    int bestSlot = -1;
    int bestDelta = 24 * 60 + 1; // > max minutes in day

    uint16_t nowMin = time.hour * 60 + time.minutes;

    for (int i = 0; i < 5; i++)
    {
        if (!timerSlots[i].enabled)
            continue;

        uint16_t slotMin = timerSlots[i].onHour * 60 + timerSlots[i].onMinute;

        int delta = slotMin - nowMin;
        if (delta < 0) delta += 24 * 60;  // wrap next day

        if (delta < bestDelta)
        {
            bestDelta = delta;
            bestSlot = i;
        }
    }

    if (bestSlot < 0)
    {
        /* No timer slots active */
        return;
    }

    /* Turn on TIMER MODE */
    clear_all_modes();
    timerActive = true;

    /* If nearest slot should already be ON, start motor */
    if (timer_any_slot_should_run())
        start_motor();
    else
        stop_motor_keep_modes();
}


void ModelHandle_Process(void)
{
    /* 1) Dry run logic */
    ModelHandle_SoftDryRunHandler();

    /* ⭐ NEW: Twist time-based ON/OFF control */
    twist_time_logic();   // <<<< NEW
    if (twistActive)
        twist_tick();
    if (autoActive)      auto_tick();

    if (countdownActive)     countdown_tick();
    if (timerActive)         timer_tick();
    if (autoActive) auto_tick();


    /* 3) Protections */
    protections_tick();

    /* 4) Led */
    leds_from_model();

    /* 5) Status */
    Safe_SendStatusPacket();
}



/* ============================================================
   UART COMMAND BRIDGE  (lightweight)
   ============================================================ */
void ModelHandle_ProcessUartCommand(const char* cmd)
{
    if (!cmd || !*cmd) return;

    /* ---------- MOTOR ---------- */
    if (strcmp(cmd, "MOTOR_ON") == 0)
    {
        ModelHandle_SetMotor(true);
    }
    else if (strcmp(cmd, "MOTOR_OFF") == 0)
    {
        ModelHandle_SetMotor(false);
    }

    /* ---------- SEMI AUTO ---------- */
    else if (strcmp(cmd, "SEMI_AUTO_START") == 0)
    {
        ModelHandle_StartSemiAuto();
    }
    else if (strcmp(cmd, "SEMI_AUTO_STOP") == 0)
    {
        ModelHandle_StopSemiAuto();
    }

    /* ---------- TWIST ---------- */
    else if (strcmp(cmd, "TWIST_START") == 0)
    {
    	ModelHandle_StartTwist(
    	    twistSettings.onDurationSeconds,
    	    twistSettings.offDurationSeconds,
    	    twistSettings.onHour,
    	    twistSettings.onMinute,
    	    twistSettings.offHour,
    	    twistSettings.offMinute
    	);



    }
    else if (strcmp(cmd, "TWIST_STOP") == 0)
    {
        ModelHandle_StopTwist();
    }

    /* ---------- TIMER ---------- */
    else if (strcmp(cmd, "TIMER_START") == 0)
    {
        ModelHandle_StartTimer();
    }
    else if (strcmp(cmd, "TIMER_STOP") == 0)
    {
        ModelHandle_StopTimer();
    }

    /* ---------- RESET ALL ---------- */
    else if (strcmp(cmd, "RESET_ALL") == 0)
    {
        ModelHandle_ResetAll();
    }
}
void ModelHandle_ResetAll(void)
{
    clear_all_modes();
    stop_motor_keep_modes();

    /* Reset flags */
    senseDryRun        = false;
    senseOverLoad      = false;
    senseOverUnderVolt = false;
    senseMaxRunReached = false;

    /* Reset all modes */
    countdownActive = false;
    twistActive     = false;
    timerActive     = false;
    semiAutoActive  = false;
    manualActive    = false;

    /* Reset mode settings */
    twistSettings.onDurationSeconds  = 5;
    twistSettings.offDurationSeconds = 5;



    countdownDuration = 0;

    for (int i = 0; i < 5; i++)
    {
        timerSlots[i].enabled = false;
        timerSlots[i].onHour = 0;
        timerSlots[i].onMinute = 0;
        timerSlots[i].offHour = 0;
        timerSlots[i].offMinute = 0;
    }

    Safe_SendStatusPacket();
}
void ModelHandle_SaveCurrentStateToEEPROM(void)
{
    static uint32_t lastWriteTick = 0;
    uint32_t now = now_ms();

    if (now - lastWriteTick < 15000U)
        return;

    static RTC_PersistState s;
    memset(&s, 0, sizeof(s));

    /* mode */
    if      (manualActive)    s.mode = 1;
    else if (semiAutoActive)  s.mode = 2;
    else if (timerActive)     s.mode = 3;
    else if (countdownActive) s.mode = 4;
    else if (twistActive)     s.mode = 5;
    else                      s.mode = 0;

    s.motor = Motor_GetStatus();

    /* twist */
    s.twistOn  = twistSettings.onDurationSeconds;
    s.twistOff = twistSettings.offDurationSeconds;


    /* countdown */
    s.countdownMin = countdownDuration;
    s.countdownRep = 1;

    /* Save only Timer Slot 0 */
    memcpy(&s.timerSlots[0], &timerSlots[0], sizeof(TimerSlot));

    RTC_SavePersistentState(&s);

    lastWriteTick = now;
}
