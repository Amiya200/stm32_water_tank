/***************************************************************
 *  HELONIX Water Pump Controller
 *  FINAL MODEL HANDLE (Option 1 - Keep SoftDryRun)
 *  Clean Auto Mode + All Modes Stable + All Errors Fixed
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
   EXTERNALS
   ============================================================ */
extern ADC_Data adcData;
extern RTC_Time_t time;

/* ============================================================
   GLOBAL MODE FLAGS
   ============================================================ */
volatile bool manualActive    = false;
volatile bool semiAutoActive  = false;
volatile bool countdownActive = false;
volatile bool twistActive     = false;
volatile bool timerActive     = false;
volatile bool autoActive      = false;

/* ============================================================
   MOTOR STATUS
   ============================================================ */
volatile uint8_t motorStatus = 0;

/* ============================================================
   SAFETY FLAGS
   ============================================================ */
volatile bool senseDryRun         = false;  // TRUE = WATER
volatile bool senseOverLoad       = false;
volatile bool senseOverUnderVolt  = false;
volatile bool senseMaxRunReached  = false;

/* ============================================================
   MANUAL OVERRIDE
   ============================================================ */
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
    autoActive = false;
    manualOverride = false;
}

/* ============================================================
   MOTOR CONTROL
   ============================================================ */
static inline void motor_apply(bool on)
{
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

void ModelHandle_SetMotor(bool on)
{
    clear_all_modes();
    manualOverride = true;
    senseDryRun = true;

    if (on) start_motor();
    else    stop_motor();
}

/* ============================================================
   TANK FULL DETECTION
   ============================================================ */
static inline bool isTankFull(void)
{
    static uint32_t stableStart = 0;
    static bool lastState = false;

    /* --- CHECK ALL 5 LEVEL SENSORS (ADC1-ADC5) --- */
    bool allZero = true;
    for (int i = 1; i <= 5; i++)
    {
        if (adcData.voltages[i] > 0.10f)   // any sensor touched water
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
            stableStart = now;    // start 1-second stability timer
            lastState = true;
        }

        if (now - stableStart >= 1000)
            return true;          // ★ Tank FULL confirmed
    }
    else
    {
        lastState = false;
    }

    return false;
}

/* ============================================================
   DRY RUN CHECK (TRUE=Water / FALSE=Dry)
   ============================================================ */
void ModelHandle_CheckDryRun(void)
{
    if (manualActive || semiAutoActive || countdownActive)
    {
        senseDryRun = true; // ignore dry in these
        return;
    }

    float v = adcData.voltages[0];
    senseDryRun = (v <= 0.01f); // water=1 / dry=0
}
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
void ModelHandle_StopAllModesAndMotor(void)
{
    manualActive    = false;
    semiAutoActive  = false;
    countdownActive = false;
    twistActive     = false;
    timerActive     = false;
    autoActive      = false;
    manualOverride  = false;

    stop_motor();
}
void ModelHandle_ProcessDryRun(void)
{
    ModelHandle_SoftDryRunHandler();
}


/**********************************************************************
 *  SOFT DRY RUN FSM (KEEP AS IS — OPTION 1)
 **********************************************************************/
typedef enum {
    DRY_IDLE = 0,
    DRY_PROBE,
    DRY_NORMAL
} DryFSMState;

static DryFSMState dryState = DRY_IDLE;
static bool        dryConfirming = false;
static uint32_t    dryDeadline = 0;
static uint32_t    dryConfirmStart = 0;

#define DRY_PROBE_ON_MS   5000UL
#define DRY_PROBE_OFF_MS  10000UL
#define DRY_CONFIRM_MS     1500UL

static inline bool isAnyModeActive(void)
{
    return (manualActive || semiAutoActive || countdownActive ||
            twistActive || timerActive || autoActive);
}

void ModelHandle_SoftDryRunHandler(void)
{
    uint32_t now = now_ms();
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
            if (senseDryRun)
            {
                start_motor();
                dryState = DRY_NORMAL;
                break;
            }

            if (now >= dryDeadline)
            {
                start_motor();
                dryState = DRY_PROBE;
                dryDeadline = now + DRY_PROBE_ON_MS;
            }
            break;

        case DRY_PROBE:
            if (senseDryRun)
            {
                dryState = DRY_NORMAL;
                break;
            }

            if (now >= dryDeadline)
            {
                if (!manualActive && !semiAutoActive && !autoActive)
                    stop_motor();

                dryState = DRY_IDLE;
                dryDeadline = now + DRY_PROBE_OFF_MS;
            }
            break;

        case DRY_NORMAL:
            if (!senseDryRun)
            {
                if (!dryConfirming)
                {
                    dryConfirming = true;
                    dryConfirmStart = now;
                }

                if ((now - dryConfirmStart) >= DRY_CONFIRM_MS)
                {
                    if (!manualActive && !semiAutoActive && !autoActive)
                        stop_motor();

                    dryState = DRY_IDLE;
                    dryDeadline = now + DRY_PROBE_OFF_MS;
                    dryConfirming = false;
                }
            }
            else dryConfirming = false;
            break;
    }
}
/* ============================================================
   COUNTDOWN MODE  — FINAL SMOOTH VERSION
   ============================================================ */

volatile uint32_t countdownDuration = 0;   // remaining seconds (for LCD)

static uint32_t cd_deadline = 0;

/* STOP Countdown */
void ModelHandle_StopCountdown(void)
{
    countdownActive = false;
    countdownDuration = 0;
    stop_motor();
}

/* START Countdown */
void ModelHandle_StartCountdown(uint32_t seconds)
{
    clear_all_modes();

    if (seconds == 0) {
        countdownActive = false;
        countdownDuration = 0;
        return;
    }

    countdownActive = true;
    countdownDuration = seconds;
    cd_deadline = now_ms() + seconds * 1000UL;

    start_motor();
}

/* TICK – MUST BE CALLED EVERY LOOP */
static void countdown_tick(void)
{
    if (!countdownActive)
        return;

    uint32_t now = now_ms();

    if (isTankFull()) {
        ModelHandle_StopCountdown();
        return;
    }

    if (now < cd_deadline) {
        countdownDuration = (cd_deadline - now) / 1000UL;
    } else {
        countdownDuration = 0;
        // stays at 0 until user presses SW4 to stop
    }
}


/* ============================================================
   TWIST MODE
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

    twistSettings.onDurationSeconds = on_s;
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

static void twist_time_logic(void)
{
    if (!twistSettings.twistArmed) return;

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

    if (twistActive &&
        time.hour == twistSettings.offHour &&
        time.minutes == twistSettings.offMinute)
    {
        twistActive = false;
        twistSettings.twistActive = false;
        stop_motor();
    }
}

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
            twist_on_phase = false;
            stop_motor();
            twist_deadline = now + twistSettings.offDurationSeconds * 1000UL;
        }
        else
        {
            twist_on_phase = true;
            start_motor();
            twist_deadline = now + twistSettings.onDurationSeconds * 1000UL;
        }
    }

    if (twist_on_phase)
        start_motor();
    else
        stop_motor();
}

/* ============================================================
   TIMER MODE
   ============================================================ */

TimerSlot timerSlots[5];

static bool timer_any_slot_should_run(void)
{
    RTC_GetTimeDate();
    uint32_t nowS = time.hour * 3600UL + time.minutes * 60UL;

    for (int i = 0; i < 5; i++)
    {
        if (!timerSlots[i].enabled) continue;

        uint32_t onS  = timerSlots[i].onHour  * 3600UL +
                        timerSlots[i].onMinute * 60UL;

        uint32_t offS = timerSlots[i].offHour * 3600UL +
                        timerSlots[i].offMinute * 60UL;

        if (onS < offS)
        {
            if (nowS >= onS && nowS < offS) return true;
        }
        else
        {
            if (nowS >= onS || nowS < offS) return true;
        }
    }
    return false;
}

void ModelHandle_TimerRecalculateNow(void)
{
    if (!timerActive) return;

    if (timer_any_slot_should_run())
        start_motor();
    else
        stop_motor();
}
void ModelHandle_StartTimer(void)
{
    clear_all_modes();
    timerActive = true;

    if (timer_any_slot_should_run())
        start_motor();
    else
        stop_motor();
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

    if (timer_any_slot_should_run()) start_motor();
    else stop_motor();
}

/* ============================================================
   SEMI-AUTO MODE
   ============================================================ */

void ModelHandle_StartSemiAuto(void)
{
    clear_all_modes();
    semiAutoActive = true;

    if (!isTankFull()) start_motor();
}

void ModelHandle_StopSemiAuto(void)
{
    semiAutoActive = false;
    stop_motor();
}
/* ============================================================
   ★★★★★ NEW AUTO MODE WITH DELAYED DRY RUN LOGIC (Option B)
   ============================================================ */

/* USER SETTINGS (from UART/menu) */
static uint16_t auto_gap_s        = 60;   // Gap wait seconds
static uint16_t auto_maxrun_min   = 12;   // Max run minutes
static uint16_t auto_retry_limit  = 5;    // User input
static uint8_t  auto_retry_count  = 0;

/* AUTO FSM */
typedef enum {
    AUTO_IDLE = 0,
    AUTO_ON_WAIT,      // Motor ON → Wait gap
    AUTO_DRY_CHECK,    // After ON wait, check dry-run
    AUTO_OFF_WAIT      // Motor OFF → Wait gap → Retry
} AutoState;

static AutoState autoState = AUTO_IDLE;
static uint32_t  autoDeadline = 0;
static uint32_t  autoRunStart = 0;

/* ========== START AUTO ========== */
void ModelHandle_StartAuto(uint16_t gap_s, uint16_t maxrun_min, uint16_t retry)
{
    clear_all_modes();
    autoActive = true;

    /* Load user parameters */
    auto_gap_s       = gap_s;
    auto_maxrun_min  = maxrun_min;
    auto_retry_limit = retry;
    auto_retry_count = 0;

    /* Initialize FSM */
    autoState = AUTO_ON_WAIT;

    start_motor();
    autoDeadline = now_ms() + (auto_gap_s * 1000UL);
    autoRunStart = now_ms();
}

/* ========== STOP AUTO ========== */
void ModelHandle_StopAuto(void)
{
    autoActive = false;
    autoState = AUTO_IDLE;
    stop_motor();
}

/* ========== AUTO TICK ========== */
static void auto_tick(void)
{
    if (!autoActive)
        return;

    uint32_t now = now_ms();

    /* ------------ PROTECTION CHECK ------------ */
    if (isTankFull() ||
        senseOverLoad ||
        senseOverUnderVolt ||
        senseMaxRunReached)
    {
        ModelHandle_StopAuto();
        return;
    }


    /* ------------ MAX RUN PROTECTION ------------ */
    if (now - autoRunStart >= (auto_maxrun_min * 60000UL))
    {
        ModelHandle_StopAuto();
        return;
    }

    /* ------------ AUTO FSM ------------ */
    switch (autoState)
    {
        /* ------------------------------
           MOTOR ON → WAIT FOR GAP TIME
           ------------------------------ */
        case AUTO_ON_WAIT:
            if (now >= autoDeadline)
            {
                autoState = AUTO_DRY_CHECK;
            }
            break;

        /* ------------------------------
           CHECK DRY AFTER ON WAIT
           ------------------------------ */
        case AUTO_DRY_CHECK:

            ModelHandle_CheckDryRun();

            if (senseDryRun)
            {
                /* WATER FOUND → Continue running normally */
                autoState = AUTO_ON_WAIT;
                autoDeadline = now + (auto_gap_s * 1000UL);
            }
            else
            {
                /* DRY FOUND → Stop motor → Go to off-wait */
                stop_motor();
                autoState = AUTO_OFF_WAIT;
                autoDeadline = now + (auto_gap_s * 1000UL);
            }
            break;

        /* ------------------------------
           MOTOR OFF → WAIT GAP → RETRY
           ------------------------------ */
        case AUTO_OFF_WAIT:
            if (now >= autoDeadline)
            {
                auto_retry_count++;

                /* Retry limit exceeded */
                if (auto_retry_limit != 0 &&
                    auto_retry_count > auto_retry_limit)
                {
                    ModelHandle_StopAuto();
                    return;
                }

                /* Do a retry */
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

/* ============================================================
   PROTECTIONS
   ============================================================ */
static void protections_tick(void)
{
    if (senseOverLoad || senseOverUnderVolt)
    {
        ModelHandle_StopAuto();
        ModelHandle_StopTimer();
        ModelHandle_StopTwist();
        ModelHandle_StopCountdown();
        ModelHandle_StopSemiAuto();
        stop_motor();
        return;
    }
}

/* ============================================================
   LED SYSTEM
   ============================================================ */

static void leds_from_model(void)
{
    LED_ClearAllIntents();

    if (Motor_GetStatus())
        LED_SetIntent(LED_COLOR_GREEN, LED_MODE_STEADY, 0);

    if (!senseDryRun && Motor_GetStatus())
        LED_SetIntent(LED_COLOR_RED, LED_MODE_STEADY, 0);

    if (countdownActive)
        LED_SetIntent(LED_COLOR_GREEN, LED_MODE_BLINK, 400);

    if (senseOverLoad)
        LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 350);

    if (senseOverUnderVolt)
        LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_BLINK, 350);

    LED_ApplyIntents();
}
void ModelHandle_CountdownTick(void)
{
    countdown_tick();
}

/* ============================================================
   MAIN PROCESS LOOP
   ============================================================ */
void ModelHandle_Process(void)
{
    uint32_t now = HAL_GetTick();
    extern void ModelHandle_CountdownTick(void);
    ModelHandle_CountdownTick();

    /* ============================================================
       AUTO MODE — FIXED LOGIC
       ============================================================ */
    if (autoActive)
    {
        /* ---- AUTO ignores dry-run ---- */
        senseDryRun = true;

        /* ---- AUTO turns ON motor immediately when tank NOT full ---- */
        if (!isTankFull())
        {
            if (!Motor_GetStatus())
            {
                start_motor();      // turn ON motor immediately
            }
        }
        else
        {
            /* ---- Tank full → motor OFF ---- */
            if (Motor_GetStatus())
            {
                stop_motor();
            }
        }

        /* No further processing is required */
        return;
    }

    /* ============================================================
       SEMI-AUTO MODE
       ============================================================ */
    if (semiAutoActive)
    {
        /* Semi-auto also ignores dry-run */
        senseDryRun = true;

        if (!isTankFull())
        {
            if (!Motor_GetStatus())
            {
                start_motor();
            }
        }
        else
        {
            stop_motor();
            semiAutoActive = false;    // End semi-auto cycle after full tank
        }

        return;
    }

    /* ============================================================
       TIMER MODE
       ============================================================ */
    if (timerActive)
    {
        if (isTankFull())
        {
            stop_motor();
            return;
        }

    }

    /* ============================================================
       COUNTDOWN MODE
       ============================================================ */
    if (countdownActive)
    {
        countdown_tick();
        return;
    }

    /* ============================================================
       TWIST MODE
       ============================================================ */
    if (twistActive)
    {
        twist_tick();
        return;
    }

    /* ============================================================
       MANUAL MODE
       ============================================================ */
    if (manualActive)
    {
        /* Manual completely ignores tank full */
        return;
    }

    /* ============================================================
       IDLE — no active mode
       ============================================================ */
    stop_motor();
}

/* ============================================================
   RESET ALL
   ============================================================ */
void ModelHandle_ResetAll(void)
{
    clear_all_modes();
    stop_motor();

    senseDryRun = false;
    senseOverLoad = false;
    senseOverUnderVolt = false;
    senseMaxRunReached = false;

    countdownDuration = 0;

    UART_SendStatusPacket();
}

/* ============================================================
   AUTO SETTINGS INTERFACE FOR SCREEN.C (FINAL & CORRECT)
   ============================================================ */

void ModelHandle_SetAutoSettings(uint16_t gap_s, uint16_t maxrun_min, uint8_t retry_count)
{
    /* These variables already exist in your file */
    auto_gap_s       = gap_s;
    auto_maxrun_min  = maxrun_min;
    auto_retry_limit = retry_count;

    /* Reset retry counter when new settings are applied */
    auto_retry_count = 0;
}

bool ModelHandle_IsAutoActive(void)
{
    return autoActive;
}


/* ============================================================
   EEPROM SAVE
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

