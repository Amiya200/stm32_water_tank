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
    float v = adcData.voltages[0];

    /* New logic:
       TRUE  = water present only if ADC ch0 ≈ 0V
       FALSE = dry for any non-zero voltage
    */

    if (v <= 0.01f)    // 0 – 10mV means WATER
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

    /* Update polarity-correct dry sensor */
    ModelHandle_CheckDryRun();  // TRUE=water, FALSE=dry

    /* Skip dry-run handling in TIMER mode */
    if (timerActive)
        return;

    /* If no modes active → motor OFF and idle state */
    if (!isAnyModeActive())
    {
        stop_motor_keep_modes();
        dryState = DRY_IDLE;
        dryConfirming = false;
        return;
    }

    /* ============================================================
       STATE MACHINE
       ============================================================ */

    switch (dryState)
    {
        /* --------------------------------------------------------
           DRY_IDLE → motor OFF, waiting to probe for water
           -------------------------------------------------------- */
        case DRY_IDLE:

            stop_motor_keep_modes();   // ensure motor is OFF

            /* If water is present immediately → jump to NORMAL RUN */
            if (senseDryRun == true)   // water detected
            {
                start_motor();
                dryState = DRY_NORMAL;
                dryConfirming = false;
                break;
            }

            /* schedule next probe */
            if ((int32_t)(dryDeadline - now) <= 0)
            {
                start_motor();
                dryState = DRY_PROBE;
                dryDeadline = now + DRY_PROBE_ON_MS;
                dryConfirming = false;
            }
            break;

        /* --------------------------------------------------------
           DRY_PROBE → motor ON for DRY_PROBE_ON_MS
           -------------------------------------------------------- */
        case DRY_PROBE:

            /* If water appears ANYTIME during probe → NORMAL */
            if (senseDryRun == true)
            {
                dryState = DRY_NORMAL;
                dryConfirming = false;
                break;
            }

            /* Probe timed out, still dry → go back to idle */
            if ((int32_t)(dryDeadline - now) <= 0)
            {
                stop_motor_keep_modes();
                dryState = DRY_IDLE;
                dryDeadline = now + DRY_PROBE_OFF_MS;
                dryConfirming = false;
            }
            break;

        /* --------------------------------------------------------
           DRY_NORMAL → continuous running while water present
           -------------------------------------------------------- */
        case DRY_NORMAL:

            /* Keep motor ON */
            if (!Motor_GetStatus())
                start_motor();

            /* If we detect DRY → confirm before switching to IDLE */
            if (senseDryRun == false)   // DRY condition
            {
                if (!dryConfirming)
                {
                    dryConfirming = true;
                    dryConfirmStart = now;
                }

                /* Confirm dry state for DRY_CONFIRM_MS */
                if ((int32_t)(now - dryConfirmStart) >= (int32_t)DRY_CONFIRM_MS)
                {
                    stop_motor_keep_modes();
                    dryState = DRY_IDLE;
                    dryDeadline = now + DRY_PROBE_OFF_MS;
                    dryConfirming = false;
                }
            }
            else
            {
                /* reset confirm if water restored */
                dryConfirming = false;
            }
            break;
    }
}

void ModelHandle_ProcessDryRun(void)
{
	/* 1) Update dry sensor only if NOT in timer mode */
	if (!timerActive)
	{
	    ModelHandle_SoftDryRunHandler();
	}

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

    /* Phase transition */
    if ((int32_t)(twist_deadline - now) <= 0)
    {
        twist_on_phase = !twist_on_phase;

        if (twist_on_phase)
        {
            /* ON phase */
            if (senseDryRun == true)  // water present
            {
                start_motor();
            }
            else
            {
                /* dry → skip ON phase entirely */
                twist_on_phase = false;
            }
            twist_deadline = now + twistSettings.onDurationSeconds * 1000UL;
        }
        else
        {
            /* OFF phase */
            stop_motor_keep_modes();
            twist_deadline = now + twistSettings.offDurationSeconds * 1000UL;
        }
    }

    /* During ON — ensure water present */
    if (twist_on_phase)
    {
        if (senseDryRun == false)  // DRY detected
        {
            stop_motor_keep_modes();
            twist_on_phase = false;
            twist_deadline = now + twistSettings.offDurationSeconds * 1000UL;
        }
        else
        {
            if (!Motor_GetStatus())
                start_motor();
        }
    }
}
/* ============================================================
   SEARCH MODE
   ------------------------------------------------------------
   Packet:
       @SEARCH:SET:gap:probe#
   ============================================================ */

SearchSettings searchSettings;

typedef enum { SEARCH_GAP = 0, SEARCH_PROBE, SEARCH_RUN } SearchState;

static SearchState search_state = SEARCH_GAP;
static uint32_t search_deadline = 0;

void ModelHandle_StartSearch(uint16_t gap_s, uint16_t probe_s)
{
    clear_all_modes();

    if (gap_s == 0)   gap_s = 3;
    if (probe_s == 0) probe_s = 3;

    searchSettings.testingGapSeconds   = gap_s;
    searchSettings.dryRunTimeSeconds   = probe_s;
    searchSettings.searchActive        = true;

    searchActive = true;

    stop_motor_keep_modes();
    search_state = SEARCH_GAP;
    search_deadline = now_ms() + gap_s * 1000UL;
}

void ModelHandle_StopSearch(void)
{
    searchSettings.searchActive = false;
    searchActive = false;
    stop_motor_keep_modes();
    search_state = SEARCH_GAP;
}

/* tick */
static void search_tick(void)
{
    if (!searchActive) return;

    uint32_t now = now_ms();

    /* Tank full → stop */
    if (isTankFull())
    {
        ModelHandle_StopAllModesAndMotor();
        return;
    }

    switch (search_state)
    {
        case SEARCH_GAP:
            stop_motor_keep_modes();

            if ((int32_t)(search_deadline - now) <= 0)
            {
                /* start probe */
                search_state = SEARCH_PROBE;
                start_motor();
                search_deadline = now + searchSettings.dryRunTimeSeconds * 1000UL;
            }
            break;

        case SEARCH_PROBE:
            if (senseDryRun == true)
            {
                /* water detected → go to RUN */
                search_state = SEARCH_RUN;
                break;
            }

            /* probe expired → still dry → back to gap */
            if ((int32_t)(search_deadline - now) <= 0)
            {
                stop_motor_keep_modes();
                search_state = SEARCH_GAP;
                search_deadline = now + searchSettings.testingGapSeconds * 1000UL;
            }
            break;

        case SEARCH_RUN:
            /* water must remain present */
            if (senseDryRun == false)
            {
                /* lost water → reset to gap */
                stop_motor_keep_modes();
                search_state = SEARCH_GAP;
                search_deadline = now + searchSettings.testingGapSeconds * 1000UL;
                break;
            }

            /* keep ON */
            if (!Motor_GetStatus())
                start_motor();
            break;
    }
}/* ============================================================
   TIMER MODE — FINAL, CLEAN, FULLY FIXED VERSION
   ============================================================ */

TimerSlot timerSlots[5];

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
void ModelHandle_TimerRecalculateNow(void)
{
    if (!timerActive)
        return;

    bool shouldRun = timer_any_slot_should_run();

    if (shouldRun)
        start_motor();
    else
        stop_motor_keep_modes();
}

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
       1) Update water / dry state
       ------------------------------- */
    ModelHandle_SoftDryRunHandler();

    /* -------------------------------
       2) Tickers for all modes
       (ONLY when mode is active)
       ------------------------------- */
    if (twistActive)         twist_tick();
    if (countdownActive)     countdown_tick();
    if (searchActive)        search_tick();
    if (timerActive)        timer_tick();

    if (semiAutoActive)      semi_auto_tick();

    /* -------------------------------
       3) Protections
       ------------------------------- */
    protections_tick();

    /* -------------------------------
       4) LEDs update
       ------------------------------- */
    leds_from_model();

    /* -------------------------------
       5) Throttled UART update
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
            searchSettings.testingGapSeconds,
            searchSettings.dryRunTimeSeconds
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

/* ============================================================
   RESET ALL + EEPROM DEFAULTS
   ============================================================ */

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

    /* Reset timers/mode settings */
    twistSettings.onDurationSeconds  = 5;
    twistSettings.offDurationSeconds = 5;

    searchSettings.testingGapSeconds = 6;
    searchSettings.dryRunTimeSeconds = 4;

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

/* ============================================================
   EEPROM — SAVE CURRENT STATE
   ============================================================ */

void ModelHandle_SaveCurrentStateToEEPROM(void)
{
    static uint32_t lastWriteTick = 0;
    uint32_t now = now_ms();

    /* Write no faster than every 15 seconds */
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

    /* motor */
    s.motor = Motor_GetStatus();

    /* twist */
    s.twistOn  = twistSettings.onDurationSeconds;
    s.twistOff = twistSettings.offDurationSeconds;

    /* search */
    s.searchGap   = searchSettings.testingGapSeconds;
    s.searchProbe = searchSettings.dryRunTimeSeconds;

    /* countdown */
    s.countdownMin = countdownDuration;
    s.countdownRep = 1;

    /* timer slot 0 only */
    memcpy(&s.timerSlots[0], &timerSlots[0], sizeof(TimerSlot));

    /* Save to EEPROM (rtc_i2c.c handles CRC) */
    RTC_SavePersistentState(&s);

    lastWriteTick = now;
}

/* ============================================================
   END OF MODEL HANDLE
   ============================================================ */
