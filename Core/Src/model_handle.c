/***************************************************************
 *  HELONIX Water Pump Controller
 *  FINAL MODEL HANDLE – Fully Updated (Timer + Auto + Safety)
 *  STM32F103 (HAL) – RTC_I2C v2.0 Compatible
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

/***************************************************************
 *  EXTERNAL MODULES
 ***************************************************************/
extern ADC_Data adcData;
extern RTC_Time_t time;

/***************************************************************
 *  GLOBAL MODE FLAGS
 ***************************************************************/
volatile bool manualActive    = false;
volatile bool semiAutoActive  = false;
volatile bool countdownActive = false;
volatile bool twistActive     = false;
volatile bool timerActive     = false;
volatile bool autoActive      = false;

/***************************************************************
 *  MOTOR STATUS
 ***************************************************************/
volatile uint8_t motorStatus = 0;

/***************************************************************
 *  SAFETY FLAGS
 ***************************************************************/
volatile bool senseDryRun         = false;
volatile bool senseOverLoad       = false;
volatile bool senseOverUnderVolt  = false;
volatile bool senseMaxRunReached  = false;

/***************************************************************
 *  MANUAL OVERRIDE
 ***************************************************************/
volatile bool manualOverride = false;

/***************************************************************
 *  UTILITY
 ***************************************************************/
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

/***************************************************************
 *  MOTOR CONTROL (ANTI-CHATTER)
 ***************************************************************/
static inline void motor_apply(bool on)
{
    if (on == Motor_GetStatus())
        return;

    Relay_Set(1, on);
    motorStatus = on ? 1 : 0;

    UART_SendStatusPacket();
}

static inline void start_motor(void) { motor_apply(true); }
static inline void stop_motor(void)  { motor_apply(false); }

bool Motor_GetStatus(void)
{
    return (motorStatus == 1);
}

/***************************************************************
 *  MANUAL CONTROL
 ***************************************************************/
void ModelHandle_SetMotor(bool on)
{
    clear_all_modes();
    manualOverride = true;

    /* In manual mode, dry-run MUST be ignored */
    senseDryRun = false;

    if (on) start_motor();
    else    stop_motor();
}

/* Reset pump logic for SW1 single press */
void reset(void)
{
    if (Motor_GetStatus())
    {
        stop_motor();
        HAL_Delay(2000);
        start_motor();
    }
    else
    {
        start_motor();
        HAL_Delay(2000);
        stop_motor();
    }
}

/***************************************************************
 *  TANK FULL DETECTION
 ***************************************************************/
static inline bool isTankFull(void)
{
    static uint32_t stableStart = 0;
    static bool lastState = false;

    bool allZero = true;

    /* ADC1..ADC5 are level sensors */
    for (int i = 1; i <= 5; i++)
    {
        if (adcData.voltages[i] > 0.10f)
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
            return true;
    }
    else
    {
        lastState = false;
    }

    return false;
}

/***************************************************************
 *  DRY RUN CHECK (Voltage > 0.01 → DRY)
 ***************************************************************/
void ModelHandle_CheckDryRun(void)
{
    /* ======================================================
       IGNORE dry-run completely in these modes:
       - Manual Mode
       - Semi-Auto Mode
       - Countdown Mode   (as per your selection: IGNORE DRY RUN)
       ====================================================== */
    if (manualActive || semiAutoActive || countdownActive)
    {
        senseDryRun = false;   // Always false (no dry-run trigger)
        return;
    }

    /* ======================================================
       For Auto Mode, Timer Mode, Twist Mode:
       DRY RUN MUST WORK
       ====================================================== */

    float v = adcData.voltages[0];

    /* Correct sensor polarity:
       Voltage <= 0.01 → WATER PRESENT  → DryRun = false
       Voltage > 0.01  → DRY           → DryRun = true
    */
    if (v > 0.01f)
        senseDryRun = false;     // Water present
    else
        senseDryRun = true;      // Dry condition
}

/***************************************************************
 *  DRY RUN SOFT PROBE HANDLER (FSM)
 ***************************************************************/
typedef enum {
    DRY_IDLE = 0,
    DRY_PROBE,
    DRY_NORMAL
} DryFSMState;

static DryFSMState dryState = DRY_IDLE;
static bool dryConfirming   = false;

static uint32_t dryDeadline     = 0;
static uint32_t dryConfirmStart = 0;

#define DRY_PROBE_ON_MS      5000UL
#define DRY_PROBE_OFF_MS    10000UL
#define DRY_CONFIRM_MS       1500UL

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
    uint32_t now = HAL_GetTick();

    /* ============================================================
       1) FULL BYPASS DURING COUNTDOWN MODE
       ------------------------------------------------------------
       - No dry-run detection
       - No ON/OFF toggling
       - No FSM actions
       - Motor must remain ON continuously
       ============================================================ */
    if (countdownActive)
    {
        /* Reset FSM so no pending timeouts will toggle motor */
        dryState       = DRY_IDLE;
        dryConfirming  = false;
        dryDeadline    = 0;
        dryConfirmStart= 0;

        return;   // *** critical: don't run any dry-run logic ***
    }

    /* ============================================================
       2) NORMAL DRY-RUN CHECK FOR OTHER MODES
       ============================================================ */
    ModelHandle_CheckDryRun();

    /* If NO mode is active → stop motor and reset FSM */
    if (!isAnyModeActive())
    {
        stop_motor();
        dryState       = DRY_IDLE;
        dryConfirming  = false;
        dryDeadline    = 0;
        return;
    }

    /* ============================================================
       3) DRY-RUN FSM OPERATION (for auto / timer / twist)
       ============================================================ */
    switch (dryState)
    {
    case DRY_IDLE:
        if (senseDryRun)     // water present
        {
            start_motor();
            dryState = DRY_NORMAL;
        }
        else                 // dry
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
        if (senseDryRun)      // water found
        {
            dryState = DRY_NORMAL;
        }
        else if (now >= dryDeadline)
        {
            stop_motor();
            dryState = DRY_IDLE;
            dryDeadline = now + DRY_PROBE_OFF_MS;
        }
        break;

    case DRY_NORMAL:
        if (!senseDryRun)     // dry condition
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
                dryConfirming = false;
                dryDeadline = now + DRY_PROBE_OFF_MS;
            }
        }
        else
        {
            dryConfirming = false;
        }
        break;
    }
}

/* Legacy external call */
void ModelHandle_ProcessDryRun(void)
{
    ModelHandle_SoftDryRunHandler();
}
void ModelHandle_StartTimerNearestSlot(void)
{
    clear_all_modes();
    timerActive = true;

    /* Force RTC update */
    RTC_GetTimeDate();

    uint32_t nowS = time.hour * 3600UL + time.min * 60UL;

    int nearest = -1;
    uint32_t nearestDiff = 86400UL;

    for (int i = 0; i < 5; i++)
    {
        if (!timerSlots[i].enabled)
            continue;

        uint32_t onS = timerSlots[i].onHour * 3600UL +
                       timerSlots[i].onMinute * 60UL;

        uint32_t diff = (onS >= nowS) ? (onS - nowS) : (86400UL - (nowS - onS));

        if (diff < nearestDiff)
        {
            nearest = i;
            nearestDiff = diff;
        }
    }

    if (nearest >= 0)
    {
        /* Immediately match engine to slot logic */
        ModelHandle_TimerRecalculateNow();
    }
    else
    {
        stop_motor();
    }
}

/***************************************************************
 *  MANUAL TOGGLE MODE
 ***************************************************************/
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

/***************************************************************
 *  STOP EVERYTHING
 ***************************************************************/
void ModelHandle_StopAllModesAndMotor(void)
{
    clear_all_modes();
    stop_motor();
}
/***************************************************************
 *  ==================== COUNTDOWN MODE ========================
 ***************************************************************/

volatile uint32_t countdownDuration = 0;
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
        ModelHandle_StopCountdown();   // <--- AUTO STOP HERE
        return;
    }

}

/***************************************************************
 *  ======================== TWIST MODE ========================
 ***************************************************************/

TwistSettings twistSettings;

static bool     twist_on_phase = false;
static uint32_t twist_deadline = 0;

/* Start twist ON/OFF cycling with time-of-day activation */
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

/* Time-of-day activation logic */
static void twist_time_logic(void)
{
    if (!twistSettings.twistArmed)
        return;

    /* Start twist at ON time */
    if (!twistActive &&
        time.hour == twistSettings.onHour &&
        time.min  == twistSettings.onMinute)
    {
        twistActive = true;
        twistSettings.twistActive = true;

        twist_on_phase = true;
        twist_deadline = now_ms() + twistSettings.onDurationSeconds * 1000UL;

        start_motor();
    }

    /* Stop twist at OFF time */
    if (twistActive &&
        time.hour == twistSettings.offHour &&
        time.min  == twistSettings.offMinute)
    {
        ModelHandle_StopTwist();
    }
}

/* ON-phase/OFF-phase cycling */
static void twist_tick(void)
{
    if (!twistActive) return;

    uint32_t now = now_ms();

    if (isTankFull())
    {
        ModelHandle_StopTwist();
        return;
    }

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

    /* Enforce correct output */
    if (twist_on_phase) start_motor();
    else                stop_motor();
}

/***************************************************************
 *  ========================= TIMER MODE ========================
 *  (Part 1: Slot evaluation, day mask, active window)
 ***************************************************************/

/* Convert RTC weekDay (1–7) → bitmask (0–6) */
// Convert DS1307 dow (1–7) → bitmask Mon..Sun (bit0..bit6)
static inline uint8_t get_today_mask(void)
{
    uint8_t rtcDay = time.dow;      // DS1307: 1=Mon ... 7=Sun
    if (rtcDay < 1 || rtcDay > 7)
        rtcDay = 1;                 // safety fallback → Monday

    uint8_t idx = rtcDay - 1;       // 0–6
    return (1u << idx);             // Mon=1<<0, ..., Sun=1<<6
}


/* TIMER SLOT STORAGE (5 slots) */
TimerSlot timerSlots[5];

/* Check if a single slot is active (time + dayMask + enabled) */
static bool timer_slot_should_run(TimerSlot *t)
{
    if (!t->enabled)
        return false;

    uint8_t todayMask = get_today_mask();

    if ((t->dayMask & todayMask) == 0)
        return false;

    uint32_t nowS = time.hour * 3600UL + time.min * 60UL;

    uint32_t onS  = t->onHour  * 3600UL + t->onMinute  * 60UL;
    uint32_t offS = t->offHour * 3600UL + t->offMinute * 60UL;

    /* Normal window: ON < OFF */
    if (onS < offS)
        return (nowS >= onS && nowS < offS);

    /* Overnight window: ON > OFF */
    return (nowS >= onS || nowS < offS);
}

/* Check if ANY of the 5 slots are active */
static bool timer_any_active_slot(void)
{
    for (int i = 0; i < 5; i++)
    {
        if (timer_slot_should_run(&timerSlots[i]))
            return true;
    }
    return false;
}
/***************************************************************
 *  ========================= TIMER MODE ========================
 *  (Part 2: Start/Stop/Recalculate/Tick)
 ***************************************************************/

/* Re-evaluate timer state immediately */
void ModelHandle_TimerRecalculateNow(void)
{
    if (!timerActive)
        return;

    bool runWindow = timer_any_active_slot();

    if (runWindow)
    {
        start_motor();
        ModelHandle_CheckDryRun();

        if (senseDryRun)     // DRY detected -> STOP MOTOR
            stop_motor();
    }
    else
    {
        stop_motor();
    }
}

/* Start TIMER mode */
void ModelHandle_StartTimer(void)
{
    clear_all_modes();
    timerActive = true;

    bool runWindow = timer_any_active_slot();

    if (runWindow)
    {
        /* Motor must be ON first */
        start_motor();

        /* Dry-check second */
        ModelHandle_CheckDryRun();

        if (senseDryRun)
            stop_motor();
    }
    else
    {
        stop_motor();
    }
}

/* Stop TIMER mode entirely */
void ModelHandle_StopTimer(void)
{
    timerActive = false;
    stop_motor();
}

/* TIMER tick engine */
static void timer_tick(void)
{
    if (!timerActive)
        return;

    if (isTankFull())
    {
        stop_motor();
        return;
    }

    bool runWindow = timer_any_active_slot();

    if (runWindow)
    {
        /* Step 1: Motor ON always first */
        start_motor();

        /* Step 2: Dry-run check */
        ModelHandle_CheckDryRun();

        if (senseDryRun)     // DRY → stop motor, stay in window
            stop_motor();
    }
    else
    {
        stop_motor();
    }
}

/***************************************************************
 *  ======================== SEMI-AUTO MODE ====================
 ***************************************************************/
void ModelHandle_StartSemiAuto(void)
{
    clear_all_modes();
    semiAutoActive = true;

    /* semi-auto ignores dry-run */
    senseDryRun = false;

    if (!isTankFull())
        start_motor();
}

void ModelHandle_StopSemiAuto(void)
{
    semiAutoActive = false;
    stop_motor();
}

/***************************************************************
 *  ======================== AUTO MODE =========================
 ***************************************************************/

/* Auto parameters */
static uint16_t auto_gap_s       = 10;   // seconds between retries
static uint16_t auto_maxrun_min  = 12;   // max runtime limit
static uint8_t  auto_retry_limit = 5;    // retry limit
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

/* Start AUTO mode */
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

/* Stop AUTO mode */
void ModelHandle_StopAuto(void)
{
    autoActive = false;
    autoState = AUTO_IDLE;
    auto_retry_count = 0;
    stop_motor();
}

/* AUTO tick engine */
static void auto_tick(void)
{
    if (!autoActive)
        return;

    uint32_t now = now_ms();

    /* Global protections first */
    if (isTankFull() ||
        senseOverLoad ||
        senseOverUnderVolt ||
        senseMaxRunReached)
    {
        ModelHandle_StopAuto();
        return;
    }

    /* Max runtime protection */
    if (now - autoRunStart >= (auto_maxrun_min * 60000UL))
    {
        senseMaxRunReached = true;
        ModelHandle_StopAuto();
        return;
    }

    switch (autoState)
    {
        case AUTO_ON_WAIT:
            if (now >= autoDeadline)
                autoState = AUTO_DRY_CHECK;
            break;

        case AUTO_DRY_CHECK:
            ModelHandle_CheckDryRun();

            if (!senseDryRun)  // DRY detected
            {
                stop_motor();
                autoState = AUTO_OFF_WAIT;
                autoDeadline = now + (auto_gap_s * 1000UL);
            }
            else               // WATER present
            {
                autoState = AUTO_ON_WAIT;
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
 *  ======================== PROTECTIONS ========================
 ***************************************************************/
static void protections_tick(void)
{
    /* Overload OR voltage fault → everything OFF immediately */
    if (senseOverLoad || senseOverUnderVolt)
    {
        ModelHandle_StopAllModesAndMotor();
        return;
    }
}

/***************************************************************
 *  ======================== LED SYSTEM =========================
 ***************************************************************/
static void leds_from_model(void)
{
    LED_ClearAllIntents();

    /* MOTOR ON → GREEN solid */
    if (Motor_GetStatus())
        LED_SetIntent(LED_COLOR_GREEN, LED_MODE_STEADY, 0);

    /* AUTO/TIMER dry-run retry → GREEN BLINK */
    if (!senseDryRun && (autoActive || timerActive))
        LED_SetIntent(LED_COLOR_GREEN, LED_MODE_BLINK, 350);

    /* Pump OFF due to dry-run → RED solid */
    if (!senseDryRun && !Motor_GetStatus())
        LED_SetIntent(LED_COLOR_RED, LED_MODE_STEADY, 0);

    /* Max-run reached → RED blink */
    if (senseMaxRunReached)
        LED_SetIntent(LED_COLOR_RED, LED_MODE_BLINK, 300);

    /* Overload → BLUE blink */
    if (senseOverLoad)
        LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 350);

    /* Over/Under voltage → PURPLE blink */
    if (senseOverUnderVolt)
        LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_BLINK, 350);

    LED_ApplyIntents();
}

/***************************************************************
 *  ======================= MASTER FSM ==========================
 ***************************************************************/
void ModelHandle_Process(void)
{
    /* Global protections FIRST */
    protections_tick();

    /* Twist time-of-day activation */
    twist_time_logic();

    /***********************************************************
     * AUTO MODE
     ***********************************************************/
    if (autoActive)
    {
        auto_tick();
        leds_from_model();
        return;     // IMPORTANT: prevents double-processing
    }

    /***********************************************************
     * SEMI-AUTO MODE
     ***********************************************************/
    if (semiAutoActive)
    {
        senseDryRun = false;   // semi-auto ignores dry-run

        if (!isTankFull())
        {
            if (!Motor_GetStatus())
                start_motor();
        }
        else
        {
            stop_motor();
            semiAutoActive = false;  // auto-terminate
        }

        leds_from_model();
        return;
    }

    /***********************************************************
     * TIMER MODE
     ***********************************************************/
    if (timerActive)
    {
        timer_tick();
        leds_from_model();
        return;
    }

    /***********************************************************
     * COUNTDOWN MODE
     ***********************************************************/
    /***********************************************************
     * COUNTDOWN MODE  — Motor must stay ON continuously
     * No dry-run, no auto-stops except tank full or timeout
     ***********************************************************/
    if (countdownActive)
    {
        /* Update time left */
        countdown_tick();

        /* Stop if tank is full */
        if (isTankFull())
        {
            ModelHandle_StopCountdown();
            leds_from_model();
            return;
        }

        /* Keep motor ON ALWAYS during countdown */
        if (!Motor_GetStatus())
            start_motor();

        /* Countdown finished? Motor must remain ON */


        leds_from_model();
        return;
    }


    /***********************************************************
     * TWIST MODE (duration-based ON/OFF)
     ***********************************************************/
    if (twistActive)
    {
        twist_tick();
        leds_from_model();
        return;
    }

    /***********************************************************
     * MANUAL MODE
     ***********************************************************/
    if (manualActive)
    {
        leds_from_model();
        return;
    }

    /***********************************************************
     * NO MODE ACTIVE → MOTOR OFF
     ***********************************************************/
    stop_motor();
    leds_from_model();
}

/***************************************************************
 *  ======================== RESET ALL =========================
 ***************************************************************/
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

/***************************************************************
 *  ==================== AUTO SETTINGS UPDATE ==================
 ***************************************************************/
void ModelHandle_SetAutoSettings(uint16_t gap_s,
                                 uint16_t maxrun_min,
                                 uint8_t retry_count)
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

/***************************************************************
 *  =================== EEPROM SAVE CURRENT STATE ==============
 ***************************************************************/
void ModelHandle_SaveCurrentStateToEEPROM(void)
{
    RTC_PersistState s;
    memset(&s, 0, sizeof(s));

    /* encode current mode for persistence */
    if      (manualActive)    s.mode = 1;
    else if (semiAutoActive)  s.mode = 2;
    else if (timerActive)     s.mode = 3;
    else if (countdownActive) s.mode = 4;
    else if (twistActive)     s.mode = 5;
    else if (autoActive)      s.mode = 6;
    else                      s.mode = 0;

    RTC_SavePersistentState(&s);
}
