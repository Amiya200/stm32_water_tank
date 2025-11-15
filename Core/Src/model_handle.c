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
static const uint32_t MAX_CONT_RUN_MS = 2UL * 60UL * 60UL * 1000UL; // 2 hours

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
volatile bool searchActive    = false;
volatile bool timerActive     = false;

volatile bool senseDryRun         = false;  // TRUE = water present
volatile bool senseOverLoad       = false;
volatile bool senseOverUnderVolt  = false;
volatile bool senseMaxRunReached  = false;

/* manual override (MOTOR_ON/MOTOR_OFF manual push) */
volatile bool manualOverride = false;
volatile bool searchArmed = false;   // NEW — search configured but not yet active

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
    searchActive = false;
    timerActive = false;
    manualOverride = false;
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

    UART_SendStatusPacket();
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
    if (on)
    {
        clear_all_modes();
        manualOverride = true;
        start_motor();
    }
    else
    {
        clear_all_modes();
        stop_motor_keep_modes();
    }
}

/* ============================================================
   DRY SENSOR CHECK (YOU CONFIRMED LOGIC)
   TRUE  => WATER
   FALSE => DRY
   ============================================================ */
void ModelHandle_CheckDryRun(void)
{
    /* Ignore dry-run completely during countdown */
    if (countdownActive) {
        senseDryRun = true;     // force WATER PRESENT
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
    if (!manualActive)
    {
        clear_all_modes();
        manualActive = true;
        manualOverride = true;
        start_motor();
    }
    else
    {
        manualActive = false;
        manualOverride = false;
        stop_motor_keep_modes();
    }
}
void ModelHandle_StopAllModesAndMotor(void)
{
    clear_all_modes();
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
            searchActive ||
            timerActive);
}
void ModelHandle_SoftDryRunHandler(void)
{
    uint32_t now = now_ms();

    /* Always refresh dry-run sensor */
    ModelHandle_CheckDryRun();  // TRUE = WATER, FALSE = DRY

    /* 🔥 Do NOT run dry-run FSM when SEARCH mode is active */
    if (searchActive)
        return;

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

    /* ------------------------------------------------------------
       IMPORTANT:
       If a mode (search, manual, semi-auto) wants MOTOR ON,
       we DO NOT block it here. We allow motor to turn ON first,
       then we evaluate DRY logic in NORMAL state.
       ------------------------------------------------------------ */

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
                if (!searchActive && !manualActive && !semiAutoActive)
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
                    if (!searchActive && !manualActive && !semiAutoActive)
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

void ModelHandle_StartTwist(uint16_t on_s, uint16_t off_s)
{
    clear_all_modes();

    if (on_s == 0)  on_s  = 1;
    if (off_s == 0) off_s = 1;

    twistSettings.onDurationSeconds  = on_s;
    twistSettings.offDurationSeconds = off_s;
    twistSettings.twistActive = true;
    twistActive = true;

    twist_on_phase = false;  // start in OFF phase
    twist_deadline = now_ms() + off_s * 1000UL;
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
   SEARCH MODE  — Fully Rewritten (Stable & Working)
   ============================================================ */

SearchSettings searchSettings;

typedef enum {
    SEARCH_GAP = 0,
    SEARCH_PROBE,
    SEARCH_RUN
} SearchState;

static SearchState search_state = SEARCH_GAP;
static uint32_t search_deadline = 0;

void ModelHandle_StartSearch(uint16_t gap_s, uint16_t probe_s,
                             uint8_t onH, uint8_t onM,
                             uint8_t offH, uint8_t offM)
{
    clear_all_modes();

    searchSettings.gapSeconds   = gap_s;
    searchSettings.probeSeconds = probe_s;

    searchSettings.onHour       = onH;
    searchSettings.onMinute     = onM;
    searchSettings.offHour      = offH;
    searchSettings.offMinute    = offM;

    searchArmed = true;   // Waiting for ON-time
    searchActive = false;

    searchSettings.searchActive = false;
    search_state = SEARCH_GAP;  // Initial idle state
}

void ModelHandle_StopSearch(void)
{
    searchActive = false;
    searchSettings.searchActive = false;

    stop_motor_keep_modes();
    search_state = SEARCH_GAP;
}

/* ============================================================
   SEARCH MODE TICK — Call every loop
   ============================================================ */
static void search_tick(void)
{
    uint32_t now = now_ms();

    /* ----------------------------------------------------------
       1. Activate Search Mode ONLY at ON time
       ---------------------------------------------------------- */
    if (searchArmed && !searchActive)
    {
        if (time.hour == searchSettings.onHour &&
            time.minutes == searchSettings.onMinute)
        {
            searchArmed = false;
            searchActive = true;
            searchSettings.searchActive = true;

            /* Start first probe immediately */
            start_motor();
            search_state = SEARCH_PROBE;
            search_deadline = now + (searchSettings.probeSeconds * 1000UL);
            return;
        }
        return; // Not ON time yet
    }

    /* Not active → do nothing */
    if (!searchActive)
        return;

    /* ----------------------------------------------------------
       2. Stop Search Mode at OFF time
       ---------------------------------------------------------- */
    if (time.hour == searchSettings.offHour &&
        time.minutes == searchSettings.offMinute)
    {
        ModelHandle_StopSearch();
        return;
    }

    /* ----------------------------------------------------------
       3. SEARCH MODE FSM
       ---------------------------------------------------------- */
    switch (search_state)
    {
        /* ======================================================
           GAP STATE — Motor OFF waiting between probes
           ====================================================== */
        case SEARCH_GAP:
            stop_motor_keep_modes();

            if ((int32_t)(now - search_deadline) >= 0)
            {
                /* Start a new probing cycle */
                start_motor();
                search_state = SEARCH_PROBE;
                search_deadline = now + (searchSettings.probeSeconds * 1000UL);
            }
            break;

        /* ======================================================
           PROBE STATE — Motor ON checking for water
           ====================================================== */
        case SEARCH_PROBE:

            /* Water detected → go to continuous run */
            if (senseDryRun == true)
            {
                search_state = SEARCH_RUN;
                break;
            }

            /* Probe timeout -- still dry → go to GAP */
            if ((int32_t)(now - search_deadline) >= 0)
            {
                stop_motor_keep_modes();
                search_state = SEARCH_GAP;
                search_deadline = now + (searchSettings.gapSeconds * 1000UL);
            }
            break;

        /* ======================================================
           RUN STATE — Water present, motor runs continuously
           ====================================================== */
        case SEARCH_RUN:

            /* Water lost → go back to GAP */
            if (senseDryRun == false)
            {
                stop_motor_keep_modes();
                search_state = SEARCH_GAP;
                search_deadline = now + (searchSettings.gapSeconds * 1000UL);
                break;
            }

            /* Water present: ensure motor ON */
            if (!Motor_GetStatus())
                start_motor();

            /* Tank full → keep motor OFF but stay in RUN */
            if (isTankFull())
            {
                stop_motor_keep_modes();
            }
            break;
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

static void semi_auto_tick(void)
{
    if (!semiAutoActive) return;

    /* full tank terminates mode */
    if (isTankFull())
    {
        ModelHandle_StopAllModesAndMotor();
        return;
    }

    /* stay ON */
    if (!Motor_GetStatus())
        start_motor();
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

void ModelHandle_Process(void)
{
    /* -------------------------------
       1) Update dry-run input state
       ------------------------------- */
    ModelHandle_SoftDryRunHandler();

    /* -------------------------------
       2) Tick active modes
       ------------------------------- */
    if (twistActive)         twist_tick();
    if (countdownActive)     countdown_tick();
//    if (searchActive)        search_tick();
    search_tick();
    if (timerActive)         timer_tick();

    if (semiAutoActive)      semi_auto_tick();

    /* -------------------------------
       3) Protection tick
       ------------------------------- */
    protections_tick();

    /* -------------------------------
       4) LEDs update
       ------------------------------- */
    leds_from_model();

    /* -------------------------------
       5) UART: Send throttled status
       ------------------------------- */
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
            twistSettings.offDurationSeconds
        );
    }
    else if (strcmp(cmd, "TWIST_STOP") == 0)
    {
        ModelHandle_StopTwist();
    }

    /* ---------- SEARCH ---------- */
    else if (strcmp(cmd, "SEARCH_START") == 0)
    {
        ModelHandle_StartSearch(
            searchSettings.gapSeconds,
            searchSettings.probeSeconds,
            searchSettings.onHour,
            searchSettings.onMinute,
            searchSettings.offHour,
            searchSettings.offMinute
        );
    }

    else if (strcmp(cmd, "SEARCH_STOP") == 0)
    {
        ModelHandle_StopSearch();
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
    searchActive    = false;
    timerActive     = false;
    semiAutoActive  = false;
    manualActive    = false;

    /* Reset mode settings */
    twistSettings.onDurationSeconds  = 5;
    twistSettings.offDurationSeconds = 5;

    searchSettings.gapSeconds   = 6;
    searchSettings.probeSeconds = 4;


    countdownDuration = 0;

    for (int i = 0; i < 5; i++)
    {
        timerSlots[i].enabled = false;
        timerSlots[i].onHour = 0;
        timerSlots[i].onMinute = 0;
        timerSlots[i].offHour = 0;
        timerSlots[i].offMinute = 0;
    }

    UART_SendStatusPacket();
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
    else if (searchActive)    s.mode = 4;
    else if (countdownActive) s.mode = 5;
    else if (twistActive)     s.mode = 6;
    else                      s.mode = 0;

    s.motor = Motor_GetStatus();

    /* twist */
    s.twistOn  = twistSettings.onDurationSeconds;
    s.twistOff = twistSettings.offDurationSeconds;

    /* search (only gap + probe stored!) */
    s.searchGap   = searchSettings.gapSeconds;
    s.searchProbe = searchSettings.probeSeconds;


    /* countdown */
    s.countdownMin = countdownDuration;
    s.countdownRep = 1;

    /* Save only Timer Slot 0 */
    memcpy(&s.timerSlots[0], &timerSlots[0], sizeof(TimerSlot));

    RTC_SavePersistentState(&s);

    lastWriteTick = now;
}
