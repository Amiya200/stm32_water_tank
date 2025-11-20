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
    int submerged = 0;
    for (int i = 1; i <= 5; i++)
        if (adcData.voltages[i] < 0.1f) submerged++;

    return (submerged >= 4);
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
   COUNTDOWN MODE
   ============================================================ */

volatile uint32_t countdownDuration = 0;
volatile bool countdownMode = false;

static uint32_t cd_deadline_ms = 0;
static uint32_t cd_run_seconds = 0;

void ModelHandle_StopCountdown(void)
{
    countdownActive = false;
    countdownMode = false;
    countdownDuration = 0;
    stop_motor();
}

void ModelHandle_StartCountdown(uint32_t seconds)
{
    clear_all_modes();

    if (seconds == 0)
    {
        ModelHandle_StopCountdown();
        return;
    }

    cd_run_seconds = seconds;
    countdownActive = true;
    countdownMode   = true;

    cd_deadline_ms = now_ms() + (seconds * 1000UL);
    countdownDuration = seconds;

    start_motor();
}

static void countdown_tick(void)
{
    if (!countdownActive) return;

    uint32_t now = now_ms();

    if (isTankFull())
    {
        ModelHandle_StopCountdown();
        return;
    }

    if (now < cd_deadline_ms)
    {
        countdownDuration = (cd_deadline_ms - now) / 1000UL;
        return;
    }

    ModelHandle_StopCountdown();
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
   ★★★★★ NEW CLEAN AUTO MODE (Option 1)
   ============================================================ */

static uint16_t auto_gap_s        = 60;
static uint16_t auto_maxrun_min   = 12;
static uint16_t auto_retry_limit  = 5;
static uint16_t auto_retry_count  = 0;

static uint32_t auto_start_tick   = 0;
static uint32_t auto_next_retry   = 0;

void ModelHandle_StartAuto(uint16_t gap_s, uint16_t maxrun_min, uint16_t retry)
{
    clear_all_modes();

    /* Load user configuration */
    auto_gap_s       = gap_s;
    auto_maxrun_min  = maxrun_min;
    auto_retry_limit = retry;

    auto_retry_count = 0;
    auto_start_tick  = now_ms();
    auto_next_retry  = 0;

    autoActive = true;

    /* Start motor immediately */
    start_motor();
}


void ModelHandle_StopAuto(void)
{
    autoActive = false;
    stop_motor();
}

static void auto_tick(void)
{
    if (!autoActive) return;

    uint32_t now = now_ms();

    /* stop if tank full or faults */
    if (isTankFull() || senseOverLoad || senseOverUnderVolt || senseMaxRunReached)
    {
        ModelHandle_StopAuto();
        return;
    }

    /* max run */
    uint32_t maxrun_ms = auto_maxrun_min * 60000UL;
    if (now - auto_start_tick >= maxrun_ms)
    {
        ModelHandle_StopAuto();
        return;
    }

    /* water present → keep running */
    if (senseDryRun)
        return;

    /* dry → stop */
    stop_motor();

    /* wait for gap time */
    if (now < auto_next_retry) return;

    /* retry limit */
    if (auto_retry_limit != 0 && auto_retry_count >= auto_retry_limit)
    {
        ModelHandle_StopAuto();
        return;
    }

    /* retry */
    auto_retry_count++;
    auto_next_retry = now + auto_gap_s * 1000UL;
    start_motor();
    auto_start_tick = now;
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

/* ============================================================
   MAIN PROCESS LOOP
   ============================================================ */
void ModelHandle_Process(void)
{
    /* Dry FSM */
    ModelHandle_SoftDryRunHandler();

    /* Twist */
    twist_time_logic();
    if (twistActive) twist_tick();

    /* Auto mode */
    if (autoActive) auto_tick();

    /* Countdown */
    if (countdownActive) countdown_tick();

    /* Timer */
    if (timerActive) timer_tick();

    /* Protections */
    protections_tick();

    /* LEDs */
    leds_from_model();

    /* Status → UART */
    UART_SendStatusPacket();
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

