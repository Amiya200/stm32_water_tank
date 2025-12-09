/***************************************************************
 *  HELONIX Water Pump Controller
 *  FINAL MODEL HANDLE – FULLY UPDATED (2025)
 *  Supports:
 *    - Timer (5 Slots) with DayMask + Enabled
 *    - Semi-Auto, Auto, Countdown, Twist, Manual
 *    - Soft Dry-Run FSM
 *    - DS1307 DOW (1–7 → Mon–Sun)
 ***************************************************************/

#include "model_handle.h"
#include "relay.h"
#include "led.h"
#include "global.h"
#include "adc.h"
#include "rtc_i2c.h"
#include "uart_commands.h"
#include "stm32f1xx_hal.h"
#include "eeprom_i2c.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/***************************************************************
 *  EXTERNAL MODULE VARIABLES
 ***************************************************************/
extern ADC_Data adcData;
extern RTC_Time_t time;
TimerSlot timerSlots[5] = {0};

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
 *  MOTOR STATUS (0 = OFF, 1 = ON)
 ***************************************************************/
volatile uint8_t motorStatus = 0;

/***************************************************************
 *  SAFETY FLAGS
 ***************************************************************/
/* IMPORTANT:
 * senseDryRun == true  => DRY condition / no water / sensor open
 * senseDryRun == false => Water OK
 */
volatile bool senseDryRun         = false;
volatile bool senseOverLoad       = false;
volatile bool senseOverUnderVolt  = false;
volatile bool senseMaxRunReached  = false;
volatile bool senseUnderLoad      = false;

/***************************************************************
 *  PERSISTENT SYSTEM SETTINGS (DEVICE SETUP)
 *  (STRUCT IS DEFINED IN model_handle.h)
 ***************************************************************/
typedef struct {
    bool manual_on;
    bool semi_on;
    bool timer_on;
    bool countdown_on;
    bool twist_on;
    bool auto_on;
    bool motor_on;
} ModeState;

ModeState modeState;

/* Defaults match the menu document:
 * - Dry Run: disabled (0)
 * - Max Run: 300 min
 * - Hi/Low Volt: 270 / 190
 * - Load protections: disabled (0) until user sets
 */
SystemSettings sys = {
    .gap_time_s  = 0,      /* Dry-run gap disabled by default */
    .retry_count = 1,
    .uv_limit    = 190,
    .ov_limit    = 270,
    .overload    = 0.0f,
    .underload   = 0.0f,
    .maxrun_min  = 300
};

extern HAL_StatusTypeDef EEPROM_WriteBytes(uint16_t addr, uint8_t *data, uint16_t len);
extern HAL_StatusTypeDef EEPROM_ReadBytes(uint16_t addr, uint8_t *data, uint16_t len);

extern float g_currentA;
extern float g_voltageV;

/***************************************************************
 *  SETTINGS SAVE / LOAD
 ***************************************************************/
void ModelHandle_SaveSettingsToEEPROM(void)
{
    EEPROM_WriteBuffer(EE_ADDR_GAP_TIME,    (uint8_t*)&sys.gap_time_s,   sizeof(sys.gap_time_s));
    EEPROM_WriteBuffer(EE_ADDR_RETRY_COUNT, (uint8_t*)&sys.retry_count,  sizeof(sys.retry_count));
    EEPROM_WriteBuffer(EE_ADDR_UV_LIMIT,    (uint8_t*)&sys.uv_limit,     sizeof(sys.uv_limit));
    EEPROM_WriteBuffer(EE_ADDR_OV_LIMIT,    (uint8_t*)&sys.ov_limit,     sizeof(sys.ov_limit));
    EEPROM_WriteBuffer(EE_ADDR_OVERLOAD,    (uint8_t*)&sys.overload,     sizeof(sys.overload));
    EEPROM_WriteBuffer(EE_ADDR_UNDERLOAD,   (uint8_t*)&sys.underload,    sizeof(sys.underload));
    EEPROM_WriteBuffer(EE_ADDR_MAXRUN,      (uint8_t*)&sys.maxrun_min,   sizeof(sys.maxrun_min));

    uint16_t sig = SETTINGS_SIGNATURE;
    EEPROM_WriteBuffer(EE_ADDR_SIGNATURE, (uint8_t*)&sig, sizeof(sig));
}

void ModelHandle_LoadSettingsFromEEPROM(void)
{
    uint16_t sig = 0;
    EEPROM_ReadBuffer(EE_ADDR_SIGNATURE, (uint8_t*)&sig, sizeof(sig));

    if (sig != SETTINGS_SIGNATURE)
    {
        /* First boot → write defaults */
        ModelHandle_SaveSettingsToEEPROM();
        return;
    }

    EEPROM_ReadBuffer(EE_ADDR_GAP_TIME,    (uint8_t*)&sys.gap_time_s,   sizeof(sys.gap_time_s));
    EEPROM_ReadBuffer(EE_ADDR_RETRY_COUNT, (uint8_t*)&sys.retry_count,  sizeof(sys.retry_count));
    EEPROM_ReadBuffer(EE_ADDR_UV_LIMIT,    (uint8_t*)&sys.uv_limit,     sizeof(sys.uv_limit));
    EEPROM_ReadBuffer(EE_ADDR_OV_LIMIT,    (uint8_t*)&sys.ov_limit,     sizeof(sys.ov_limit));
    EEPROM_ReadBuffer(EE_ADDR_OVERLOAD,    (uint8_t*)&sys.overload,     sizeof(sys.overload));
    EEPROM_ReadBuffer(EE_ADDR_UNDERLOAD,   (uint8_t*)&sys.underload,    sizeof(sys.underload));
    EEPROM_ReadBuffer(EE_ADDR_MAXRUN,      (uint8_t*)&sys.maxrun_min,   sizeof(sys.maxrun_min));
}

/***************************************************************
 *  MODE STATE SAVE / LOAD (POWER RESTORE / LAST MODE)
 ***************************************************************/
void ModelHandle_SaveModeState(void)
{
    modeState.manual_on    = manualActive;
    modeState.semi_on      = semiAutoActive;
    modeState.timer_on     = timerActive;
    modeState.countdown_on = countdownActive;
    modeState.twist_on     = twistActive;
    modeState.auto_on      = autoActive;
    modeState.motor_on     = Motor_GetStatus();

    EEPROM_WriteBuffer(0x0200, (uint8_t*)&modeState, sizeof(modeState));
}

void ModelHandle_LoadModeState(void)
{
    EEPROM_ReadBuffer(0x0200, (uint8_t*)&modeState, sizeof(modeState));

    manualActive    = modeState.manual_on;
    semiAutoActive  = modeState.semi_on;
    timerActive     = modeState.timer_on;
    countdownActive = modeState.countdown_on;
    twistActive     = modeState.twist_on;
    autoActive      = modeState.auto_on;

    if (modeState.motor_on)
        ModelHandle_SetMotor(true);
}

/***************************************************************
 *  PUBLIC GETTERS FOR DEVICE SETUP VALUES
 ***************************************************************/
uint16_t ModelHandle_GetGapTime(void)        { return sys.gap_time_s;   }
uint8_t  ModelHandle_GetRetryCount(void)     { return sys.retry_count;  }
uint16_t ModelHandle_GetUnderVolt(void)      { return sys.uv_limit;     }
uint16_t ModelHandle_GetOverVolt(void)       { return sys.ov_limit;     }
float    ModelHandle_GetOverloadLimit(void)  { return sys.overload;     }
float    ModelHandle_GetUnderloadLimit(void) { return sys.underload;    }
uint16_t ModelHandle_GetMaxRunTime(void)     { return sys.maxrun_min;   }

/* Called from Screen/App when user changes Device Setup */
void ModelHandle_SetUserSettings(
    uint16_t gap_s,
    uint8_t  retry,
    uint16_t uv,
    uint16_t ov,
    float    overload,
    float    underload,
    uint16_t maxrun_min
){
    sys.gap_time_s  = gap_s;       // Dry-run gap (0 = disable dry-run)
    sys.retry_count = retry;       // Not heavily used; protections have own logic
    sys.uv_limit    = uv;          // Low volt limit (0 = disabled)
    sys.ov_limit    = ov;          // High volt limit (0 = disabled)
    sys.overload    = overload;    // Overload current (A) (0 = disabled)
    sys.underload   = underload;   // Underload current (A) (0 = disabled)
    sys.maxrun_min  = maxrun_min;  // Global max run (0 = disabled)

    ModelHandle_SaveSettingsToEEPROM();
}

/* Full factory reset for settings (as per document defaults) */
void ModelHandle_FactoryReset(void)
{
    sys.gap_time_s  = 0;       // Set Dry Run: disabled by default
    sys.retry_count = 1;
    sys.uv_limit    = 190;     // Low volt default
    sys.ov_limit    = 270;     // High volt default
    sys.overload    = 0.0f;    // Load protections disabled by default
    sys.underload   = 0.0f;
    sys.maxrun_min  = 300;     // Max Run default

    ModelHandle_SaveSettingsToEEPROM();
}

/***************************************************************
 *  INTERNAL UTILITY
 ***************************************************************/
volatile bool manualOverride = false;

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
 *  MOTOR CONTROL (ANTI-CHATTER + MAX RUN TRACKING)
 ***************************************************************/
static uint32_t motorOnStartMs = 0;

static inline bool Motor_GetStatusInternal(void)
{
    return (motorStatus == 1);
}

bool Motor_GetStatus(void)
{
    return Motor_GetStatusInternal();
}

static inline void motor_apply(bool on)
{
    bool current = Motor_GetStatusInternal();
    if (on == current)
        return;

    if (on)
        motorOnStartMs = HAL_GetTick();

    Relay_Set(1, on);
    motorStatus = on ? 1 : 0;

    UART_SendStatusPacket();
}

static inline void start_motor(void) { motor_apply(true);  }
static inline void stop_motor(void)  { motor_apply(false); }

/* Global Max Run protection – applies to ALL modes */
static void check_max_run(void)
{
    if (sys.maxrun_min == 0)
        return;
    if (!Motor_GetStatus())
        return;

    uint32_t now   = HAL_GetTick();
    uint32_t limit = (uint32_t)sys.maxrun_min * 60000UL;

    if (now - motorOnStartMs >= limit)
    {
        senseMaxRunReached = true;
        /* Max Run: restart by user only ⇒ stop everything and stay OFF */
        clear_all_modes();
        stop_motor();
        ModelHandle_SaveModeState();
    }
}

/***************************************************************
 *  MANUAL MODE CONTROL
 ***************************************************************/
void ModelHandle_SetMotor(bool on)
{
    clear_all_modes();
    manualOverride = true;

    /* In manual mode, dry-run is effectively ignored */
    senseDryRun = false;

    if (on) start_motor();
    else    stop_motor();
}

static inline bool isTankFull(void)
{
    static uint32_t stableStart = 0;
    static bool     lastState   = false;

    bool allZero = true;

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
            lastState   = true;
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
/* RESET pump logic (SW1 short press) */
void reset(void)
{
    /* If any protection fault is active, do not attempt reset run */
    if (senseOverLoad || senseUnderLoad || senseOverUnderVolt || senseMaxRunReached)
    {
        stop_motor();
        return;
    }

    /* Treat reset as a special manual test — no other modes */
    clear_all_modes();
    manualOverride = true;

    /* 1) Start motor and run up to 5s to check for water (dry-run sensor) */
    start_motor();

    uint32_t start = HAL_GetTick();
    senseDryRun = true;  // assume dry until we see water

    while ((HAL_GetTick() - start) < 5000UL)
    {
        /* Update dry-run status based on sensor voltage */
        ModelHandle_CheckDryRun();

        /* If water is detected (dry-run cleared), break early */
        if (!senseDryRun)
        {
            break;
        }

        HAL_Delay(100);
    }

    /* After 5s: if still dry → stop motor and exit */
    if (senseDryRun)
    {
        stop_motor();
        manualOverride = false;
        return;
    }

    /* 2) Water is available → keep motor ON until tank is full or a fault occurs */
    while (!isTankFull())
    {
        /* Update protection FSM and check faults */
        ModelHandle_CheckLoadFault();
        if (senseOverLoad || senseUnderLoad || senseOverUnderVolt || senseMaxRunReached)
        {
            stop_motor();
            manualOverride = false;
            return;
        }

        /* Optional: if dry-run becomes true again while running, stop as a safety */
        ModelHandle_CheckDryRun();
        if (senseDryRun)
        {
            stop_motor();
            manualOverride = false;
            return;
        }

        HAL_Delay(100);
    }

    /* Tank full → stop motor */
    stop_motor();
    manualOverride = false;
}


/***************************************************************
 *  DRY-RUN FSM CONSTANTS
 ***************************************************************/
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

/***************************************************************
 *  OVERLOAD + UNDERLOAD + VOLTAGE PROTECTION FSM
 ***************************************************************/
static uint32_t faultLockTimestamp = 0;
static bool     faultLocked        = false;

#define LOAD_FAULT_CONFIRM_MS     3000UL
#define LOAD_RETRY_RUN_MS         3000UL
#define LOAD_LOCK_DURATION_MS     (20UL * 60UL * 1000UL)   // 20 minutes

typedef enum {
    LOAD_NORMAL = 0,
    LOAD_FAULT_WAIT,
    LOAD_FAULT_LOCK,
    LOAD_RETRY_RUN
} LoadFaultState;

static LoadFaultState loadState = LOAD_NORMAL;
static uint32_t       loadTimer = 0;
static uint8_t        loadRetryCount = 0;
#define LOAD_MAX_RETRY           1

void ModelHandle_CheckLoadFault(void)
{
    float    I  = g_currentA;
    float    V  = g_voltageV;
    float    ol = ModelHandle_GetOverloadLimit();
    float    ul = ModelHandle_GetUnderloadLimit();
    uint16_t uv = ModelHandle_GetUnderVolt();
    uint16_t ov = ModelHandle_GetOverVolt();

    uint32_t now = HAL_GetTick();

    /* Enable / disable protections based on configured values */
    bool overloadEnabled  = (ol > 0.1f);
    bool underloadEnabled = (ul > 0.1f);
    bool voltEnabled      = (uv > 0 || ov > 0);

    bool overload  = overloadEnabled  && (I > ol);
    bool underload = underloadEnabled && (I < ul);
    bool voltFault = false;

    if (voltEnabled)
    {
        if (uv > 0 && ov > 0)
            voltFault = (V < uv || V > ov);
        else if (uv > 0)
            voltFault = (V < uv);
        else if (ov > 0)
            voltFault = (V > ov);
    }

    bool loadFault = (overload || underload);

    /* Update global flags */
    senseOverLoad      = overload;
    senseUnderLoad     = underload;
    senseOverUnderVolt = voltFault;

    /***********************************************************
     * LED INDICATION PRIORITY (BLUE/PURPLE handled centrally)
     ***********************************************************/
    /* Note: final LED decisions are done in leds_from_model().
       Here we only maintain fault flags and FSM. */

    /***********************************************************
     * COMMON FSM FOR LOAD + VOLTAGE
     ***********************************************************/
    bool fault = loadFault || voltFault;

    switch (loadState)
    {
        case LOAD_NORMAL:
            loadRetryCount = 0;
            if (fault && Motor_GetStatus())
            {
                loadState = LOAD_FAULT_WAIT;
                loadTimer = now;
            }
            break;

        case LOAD_FAULT_WAIT:
            if (!fault)
            {
                loadState = LOAD_NORMAL;
                break;
            }
            if (now - loadTimer >= LOAD_FAULT_CONFIRM_MS)
            {
                stop_motor();
                loadState           = LOAD_FAULT_LOCK;
                loadTimer           = now;
                faultLocked         = true;
                faultLockTimestamp  = now;
            }
            break;

        case LOAD_FAULT_LOCK:
            if (!fault)
            {
                /* Fault cleared; user / modes may restart pump if they wish */
                loadState      = LOAD_NORMAL;
                faultLocked    = false;
                loadRetryCount = 0;
                break;
            }

            if ((now - loadTimer >= LOAD_LOCK_DURATION_MS) &&
                (loadRetryCount < LOAD_MAX_RETRY) &&
                isAnyModeActive())
            {
                /* One retry after 20 minutes, respecting active modes */
                start_motor();
                loadState      = LOAD_RETRY_RUN;
                loadTimer      = now;
                loadRetryCount++;
            }
            break;

        case LOAD_RETRY_RUN:
            if (now - loadTimer >= LOAD_RETRY_RUN_MS)
            {
                if (fault)
                {
                    stop_motor();
                    loadState   = LOAD_FAULT_LOCK;
                    loadTimer   = now;
                    faultLocked = true;
                }
                else
                {
                    loadState      = LOAD_NORMAL;
                    faultLocked    = false;
                    loadRetryCount = 0;
                }
            }
            break;
    }
}

/***************************************************************
 *  TANK FULL DETECTION
 *  (5 ADC probes = level sensors)
 ***************************************************************/
void ModelHandle_StopAllModesAndMotor(void)
{
    clear_all_modes();
    stop_motor();
    ModelHandle_SaveModeState();
}


/***************************************************************
 *  DRY-RUN SENSOR CHECK
 *  Voltage <= 0.01f → WATER PRESENT → senseDryRun = false
 *  Voltage >  0.01f → DRY          → senseDryRun = true
 ***************************************************************/
void ModelHandle_CheckDryRun(void)
{
    /* Manual, Semi-Auto, Countdown ignore dry-run feature */
    if (manualActive || semiAutoActive || countdownActive)
    {
        senseDryRun = false;
        return;
    }

    float v = adcData.voltages[0];

    if (v > 0.01f)
        senseDryRun = true;   /* DRY */
    else
        senseDryRun = false;  /* WATER OK */
}

/***************************************************************
 *  MANUAL TOGGLE (SW logic)
 ***************************************************************/
void ModelHandle_ToggleManual(void)
{
    /* Stop all other modes before manual */
    timerActive     = false;
    semiAutoActive  = false;
    countdownActive = false;
    twistActive     = false;
    autoActive      = false;

    manualActive = !manualActive;
    ModelHandle_SaveModeState();

    if (manualActive)
    {
        manualOverride = true;
        start_motor();
    }
    else
    {
        manualOverride = false;
        stop_motor();
    }
}

/***************************************************************
 *  SOFT DRY-RUN FSM
 ***************************************************************/
typedef enum {
    DRY_IDLE = 0,
    DRY_PROBE,
    DRY_NORMAL
} DryFSMState;

static DryFSMState dryState      = DRY_IDLE;
static bool        dryConfirming = false;

void ModelHandle_SoftDryRunHandler(void)
{
    uint32_t now = now_ms();

    /***********************************************************
     * 0) Global enable/disable by gap time
     *    If sys.gap_time_s == 0 ⇒ dry-run logic disabled
     ***********************************************************/
    if (sys.gap_time_s == 0)
    {
        senseDryRun = false;
        return;
    }

    /***********************************************************
     * 1) COUNTDOWN / MANUAL / SEMI-AUTO → Dry-run bypass
     ***********************************************************/
    if (countdownActive || manualActive || semiAutoActive)
    {
        dryState        = DRY_IDLE;
        dryConfirming   = false;
        dryDeadline     = 0;
        dryConfirmStart = 0;
        senseDryRun     = false;
        return;
    }

    /***********************************************************
     * 2) Normal dry-run check
     ***********************************************************/
    ModelHandle_CheckDryRun();

    /* No active mode → motor off + reset FSM */
    if (!isAnyModeActive())
    {
        stop_motor();
        dryState        = DRY_IDLE;
        dryConfirming   = false;
        dryDeadline     = 0;
        return;
    }

    /***********************************************************
     * 3) FSM OPERATION
     *    senseDryRun == true  => DRY
     *    senseDryRun == false => WATER OK
     ***********************************************************/
    switch (dryState)
    {
        case DRY_IDLE:
            if (!senseDryRun)  /* water present */
            {
                start_motor();
                dryState = DRY_NORMAL;
            }
            else               /* dry */
            {
                if (now >= dryDeadline)
                {
                    start_motor();
                    dryState    = DRY_PROBE;
                    dryDeadline = now + DRY_PROBE_ON_MS;
                }
            }
            break;

        case DRY_PROBE:
            if (!senseDryRun)   /* water found */
            {
                dryState = DRY_NORMAL;
            }
            else if (now >= dryDeadline)
            {
                stop_motor();
                dryState    = DRY_IDLE;
                dryDeadline = now + DRY_PROBE_OFF_MS;
            }
            break;

        case DRY_NORMAL:
            if (senseDryRun)   /* DRY detected */
            {
                if (!dryConfirming)
                {
                    dryConfirming   = true;
                    dryConfirmStart = now;
                }
                else if (now - dryConfirmStart >= DRY_CONFIRM_MS)
                {
                    stop_motor();
                    dryState        = DRY_IDLE;
                    dryConfirming   = false;
                    dryDeadline     = now + DRY_PROBE_OFF_MS;
                }
            }
            else
            {
                dryConfirming = false;
            }
            break;
    }
}

/***************************************************************
 *  LEGACY ENTRY POINT FOR DRY-RUN HANDLER
 ***************************************************************/
void ModelHandle_ProcessDryRun(void)
{
    ModelHandle_SoftDryRunHandler();
}

/***************************************************************
 * ======================== TIMER MODE =========================
 * DS1307 dow: 1=Mon to 7=Sun
 * dayMask: bit0=Mon to bit6=Sun
 ***************************************************************/

/* Convert RTC DOW into bit mask */
static inline uint8_t get_today_mask(void)
{
    uint8_t dow = time.dow;  /* valid range 1..7 */
    if (dow < 1 || dow > 7)
        dow = 1;
    return (1u << (dow - 1)); /* Mon->bit0 ... Sun->bit6 */
}

/* Check if slot active at current HH:MM */
static bool slot_is_active_now(const TimerSlot *t)
{
    if (!t->enabled)
        return false;

    /* Day match */
    uint8_t todayMask = get_today_mask();
    if ((t->dayMask & todayMask) == 0)
        return false;

    /* Time compare HH:MM only */
    uint16_t nowHM = time.hour * 60 + time.min;
    uint16_t onHM  = t->onHour * 60 + t->onMinute;
    uint16_t offHM = t->offHour * 60 + t->offMinute;

    if (onHM < offHM)
    {
        return (nowHM >= onHM && nowHM < offHM);
    }
    else
    {
        /* Overnight slot */
        return (nowHM >= onHM || nowHM < offHM);
    }
}

/* Check if ANY slot is active */
static bool timer_any_active_slot(void)
{
    for (int i = 0; i < 5; i++)
    {
        if (slot_is_active_now(&timerSlots[i]))
            return true;
    }
    return false;
}

/***************************************************************
 * Start nearest timer slot (SW3 short press)
 ***************************************************************/
void ModelHandle_StartTimerNearestSlot(void)
{
    clear_all_modes();
    timerActive = true;

    RTC_GetTimeDate();

    uint16_t nowHM = time.hour * 60 + time.min;
    uint16_t bestDiff = 20000;

    for (int i = 0; i < 5; i++)
    {
        TimerSlot *t = &timerSlots[i];
        if (!t->enabled) continue;

        uint8_t todayMask = get_today_mask();
        if (!(t->dayMask & todayMask)) continue;

        uint16_t onHM = t->onHour * 60 + t->onMinute;
        uint16_t diff = (onHM >= nowHM) ? (onHM - nowHM)
                                        : (1440 - (nowHM - onHM));

        if (diff < bestDiff)
        {
            bestDiff = diff;
        }
    }

    /* We don't have to store 'best'; we always evaluate slots on RTC */
    ModelHandle_SaveModeState();
    ModelHandle_ProcessTimerSlots();
}

/***************************************************************
 * MAIN TIMER PROCESSOR (called every loop)
 ***************************************************************/
void ModelHandle_ProcessTimerSlots(void)
{
    if (!timerActive)
        return;

    ModelHandle_SaveModeState();

    if (timer_any_active_slot())
        start_motor();
    else
        stop_motor();
}

/***************************************************************
 * Force timer recalculation
 ***************************************************************/
void ModelHandle_TimerRecalculateNow(void)
{
    if (!timerActive)
        return;

    if (timer_any_active_slot())
        start_motor();
    else
        stop_motor();
}

/***************************************************************
 * Start / Stop Timer Mode
 ***************************************************************/
void ModelHandle_StartTimer(void)
{
    clear_all_modes();
    timerActive = true;

    ModelHandle_TimerRecalculateNow();
}

void ModelHandle_StopTimer(void)
{
    timerActive = false;
    stop_motor();
    ModelHandle_SaveModeState();
}

/***************************************************************
 * Auto timer activation when time enters a slot
 ***************************************************************/
void ModelHandle_CheckAutoTimerActivation(void)
{
    if (timerActive)
        return;

    if (timer_any_active_slot())
    {
        timerActive = true;
        start_motor();
    }
}

/***************************************************************
 * ====================== SEMI-AUTO MODE =======================
 ***************************************************************/
void ModelHandle_StartSemiAuto(void)
{
    clear_all_modes();
    semiAutoActive = true;
    ModelHandle_SaveModeState();

    /* Semi-auto ignores dry-run, but respects tank full */
    senseDryRun = false;

    if (!isTankFull())
        start_motor();
}

void ModelHandle_StopSemiAuto(void)
{
    semiAutoActive = false;
    stop_motor();
    ModelHandle_SaveModeState();
}

/***************************************************************
 * ======================== AUTO MODE ==========================
 ***************************************************************/

/* Auto-specific settings (used in Auto logic only) */
static uint16_t auto_gap_s       = 10;
static uint16_t auto_maxrun_min  = 12;
static uint8_t  auto_retry_limit = 5;
static uint8_t  auto_retry_count = 0;

typedef enum {
    AUTO_IDLE = 0,
    AUTO_ON_WAIT,
    AUTO_DRY_CHECK,
    AUTO_OFF_WAIT
} AutoState;

static AutoState autoState     = AUTO_IDLE;
static uint32_t  autoDeadline  = 0;
static uint32_t  autoRunStart  = 0;

/* Start AUTO mode */
void ModelHandle_StartAuto(uint16_t gap_s, uint16_t maxrun_min, uint16_t retry)
{
    clear_all_modes();

    autoActive       = true;
    auto_gap_s       = gap_s;
    auto_maxrun_min  = maxrun_min;
    auto_retry_limit = (uint8_t)retry;
    auto_retry_count = 0;
    autoState        = AUTO_ON_WAIT;

    ModelHandle_SaveModeState();

    start_motor();
    autoRunStart = now_ms();
    autoDeadline = now_ms() + (gap_s * 1000UL);
}

/* Stop AUTO */
void ModelHandle_StopAuto(void)
{
    autoActive       = false;
    autoState        = AUTO_IDLE;
    auto_retry_count = 0;

    ModelHandle_SaveModeState();
    stop_motor();
}

/***************************************************************
 *  AUTO MODE TICK ENGINE (Dry-run only; protection & max-run are global)
 ***************************************************************/
static void auto_tick(void)
{
    if (!autoActive)
        return;

    uint32_t now = now_ms();

    /*************************************************
     * 1) Tank full → stop Auto mode
     *************************************************/
    if (isTankFull())
    {
        ModelHandle_StopAuto();
        return;
    }

    /*************************************************
     * 2) Auto State Machine (DRY RUN ONLY)
     *    senseDryRun == true  => DRY
     *    senseDryRun == false => WATER OK
     *************************************************/
    switch (autoState)
    {
        case AUTO_ON_WAIT:
            if (now >= autoDeadline)
                autoState = AUTO_DRY_CHECK;
            break;

        case AUTO_DRY_CHECK:
            ModelHandle_CheckDryRun();

            if (senseDryRun)   /* DRY → stop → gap wait */
            {
                stop_motor();
                autoState    = AUTO_OFF_WAIT;
                autoDeadline = now + (auto_gap_s * 1000UL);
            }
            else               /* Water available → continue */
            {
                autoState    = AUTO_ON_WAIT;
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
                autoState    = AUTO_ON_WAIT;
                autoDeadline = now + (auto_gap_s * 1000UL);
            }
            break;

        default:
            break;
    }
}

/***************************************************************
 * ======================== COUNTDOWN MODE ======================
 ***************************************************************/
volatile uint32_t countdownDuration = 0;
static uint32_t   cd_deadline       = 0;

void ModelHandle_StopCountdown(void)
{
    countdownActive   = false;
    countdownDuration = 0;
    stop_motor();
    ModelHandle_SaveModeState();
}

void ModelHandle_StartCountdown(uint32_t seconds)
{
    clear_all_modes();

    if (seconds == 0)
    {
        countdownActive   = false;
        countdownDuration = 0;
        return;
    }

    countdownActive   = true;
    countdownDuration = seconds;
    cd_deadline       = now_ms() + seconds * 1000UL;

    ModelHandle_SaveModeState();
    start_motor();   /* countdown ALWAYS forces motor ON */
}

/* Update remaining time */
static void countdown_tick(void)
{
    if (!countdownActive)
        return;

    uint32_t now = now_ms();

    /* Tank full → stop immediately */
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
        ModelHandle_StopCountdown();
        return;
    }
}

/***************************************************************
 * ========================= TWIST MODE =========================
 ***************************************************************/
TwistSettings twistSettings;

static bool     twist_on_phase = false;
static uint32_t twist_deadline = 0;

void ModelHandle_StartTwist(uint16_t on_s, uint16_t off_s,
                            uint8_t onH, uint8_t onM,
                            uint8_t offH, uint8_t offM)
{
    clear_all_modes();

    if (on_s  == 0) on_s  = 1;
    if (off_s == 0) off_s = 1;

    twistSettings.onDurationSeconds  = on_s;
    twistSettings.offDurationSeconds = off_s;

    twistSettings.onHour   = onH;
    twistSettings.onMinute = onM;
    twistSettings.offHour  = offH;
    twistSettings.offMinute= offM;

    twistSettings.twistArmed  = true;
    twistSettings.twistActive = false;

    twist_on_phase = true;
    twist_deadline = now_ms() + (on_s * 1000UL);

    start_motor();
}

void ModelHandle_StopTwist(void)
{
    twistActive = false;
    twistSettings.twistActive = false;
    stop_motor();
    ModelHandle_SaveModeState();
}

/* Check if twist should start / stop based on RTC clock */
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

/* Twist ON/OFF cycling handler */
static void twist_tick(void)
{
    if (!twistActive)
        return;

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

    /* Enforce correct state */
    if (twist_on_phase)
        start_motor();
    else
        stop_motor();
}

/***************************************************************
 * ======================== PROTECTIONS =========================
 ***************************************************************/
static void protections_tick(void)
{
    /* Overload / voltage faults are handled in FSM; here we only
     * enforce Max Run latch (senseMaxRunReached) */
    if (senseMaxRunReached)
    {
        stop_motor();
    }
}

/***************************************************************
 * =========================== LEDS =============================
 ***************************************************************/
/*
 * Document mapping: :contentReference[oaicite:2]{index=2}
 * - Green steady: motor ON
 * - Green blink : motor ON with dry-run timing
 * - Red steady  : motor OFF due to dry run
 * - Red blink   : Max Run error
 * - Blue blink  : overload/underload
 * - Purple blink: over / under voltage
 */
static void leds_from_model(void)
{
    LED_ClearAllIntents();

    bool motorOn = Motor_GetStatus();

    /* Base indication: motor ON → Green steady */
    if (motorOn)
    {
        LED_SetIntent(LED_COLOR_GREEN, LED_MODE_STEADY, 0);
    }

    /* Dry-run indications */
    if (senseDryRun)
    {
        if (motorOn)
        {
            /* Motor ON with dry-run timing → Green blink */
            LED_SetIntent(LED_COLOR_GREEN, LED_MODE_BLINK, 350);
        }
        else
        {
            /* Motor OFF due to dry run → Red steady */
            LED_SetIntent(LED_COLOR_RED, LED_MODE_STEADY, 0);
        }
    }

    /* Max run reached → Red blink (overrides Red steady meaning) */
    if (senseMaxRunReached)
    {
        LED_SetIntent(LED_COLOR_RED, LED_MODE_BLINK, 300);
    }

    /* Overload / underload → Blue blink */
    if (senseOverLoad || senseUnderLoad)
    {
        LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 350);
    }

    /* Over/under voltage → Purple blink */
    if (senseOverUnderVolt)
    {
        LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_BLINK, 350);
    }

    LED_ApplyIntents();
}

/***************************************************************
 * ========================== MASTER FSM ========================
 ***************************************************************/
void ModelHandle_Process(void)
{
    /*****************************************************
     * 0. FIRST UPDATE ALL FAULTS (MUST RUN EVERY LOOP)
     *****************************************************/
    ModelHandle_CheckLoadFault();     /* Overload/Underload/Volt FSM */
    protections_tick();               /* Max Run latch enforcement   */
    twist_time_logic();               /* Time-based twist control    */
    check_max_run();                  /* Global Max Run              */

    if (senseMaxRunReached)
    {
        /* After max run, everything stays OFF until user re-selects a mode */
        leds_from_model();
        return;
    }

    /*****************************************************
     * 1. MANUAL MODE (Highest Priority)
     *****************************************************/
    if (manualActive)
    {
        /* Stop motor immediately on ANY critical fault */
        if (senseOverLoad || senseUnderLoad || senseOverUnderVolt)
        {
            stop_motor();
            manualActive   = false;
            manualOverride = false;

            ModelHandle_SaveModeState();
            leds_from_model();
            return;
        }

        /* Manual ignores dry-run & tank full */
        if (!Motor_GetStatus())
            start_motor();

        leds_from_model();
        return;
    }

    /*****************************************************
     * 2. AUTO MODE
     *****************************************************/
    if (autoActive)
    {
        auto_tick();
        leds_from_model();
        return;
    }

    /*****************************************************
     * 3. SEMI-AUTO MODE
     *****************************************************/
    if (semiAutoActive)
    {
        senseDryRun = false;   /* semi-auto ignores dry-run */

        if (!isTankFull())
        {
            if (!Motor_GetStatus())
                start_motor();
        }
        else
        {
            stop_motor();
            semiAutoActive = false;
        }

        leds_from_model();
        return;
    }

    /*****************************************************
     * 4. TIMER MODE
     *****************************************************/
    if (timerActive)
    {
        ModelHandle_ProcessTimerSlots();
        leds_from_model();
        return;
    }

    /*****************************************************
     * 5. COUNTDOWN MODE
     *****************************************************/
    if (countdownActive)
    {
        countdown_tick();

        if (isTankFull())
        {
            ModelHandle_StopCountdown();
            leds_from_model();
            return;
        }

        if (!Motor_GetStatus())
            start_motor();   /* countdown always forces ON */

        leds_from_model();
        return;
    }

    /*****************************************************
     * 6. TWIST MODE
     *****************************************************/
    if (twistActive)
    {
        twist_tick();
        leds_from_model();
        return;
    }

    /*****************************************************
     * 7. IDLE STATE
     *****************************************************/
    stop_motor();
    leds_from_model();
}

/***************************************************************
 * ========================== RESET ALL =========================
 ***************************************************************/
void ModelHandle_ResetAll(void)
{
    clear_all_modes();
    stop_motor();

    senseDryRun         = false;
    senseOverLoad       = false;
    senseOverUnderVolt  = false;
    senseUnderLoad      = false;
    senseMaxRunReached  = false;

    countdownDuration   = 0;

    UART_SendStatusPacket();
}

/***************************************************************
 * ===================== AUTO SETTINGS API ======================
 ***************************************************************/
void ModelHandle_SetAutoSettings(uint16_t gap_s, uint16_t maxrun_min, uint8_t retry)
{
    auto_gap_s       = gap_s;
    auto_maxrun_min  = maxrun_min;
    auto_retry_limit = retry;

    /* Optional: store for power loss */
    EEPROM_WriteBuffer(0x0300, (uint8_t*)&auto_gap_s,      sizeof(auto_gap_s));
    EEPROM_WriteBuffer(0x0302, (uint8_t*)&auto_maxrun_min, sizeof(auto_maxrun_min));
    EEPROM_WriteBuffer(0x0304, (uint8_t*)&auto_retry_limit,sizeof(auto_retry_limit));
}

void ModelHandle_LoadAutoSettings(void)
{
    EEPROM_ReadBuffer(0x0300, (uint8_t*)&auto_gap_s,      sizeof(auto_gap_s));
    EEPROM_ReadBuffer(0x0302, (uint8_t*)&auto_maxrun_min, sizeof(auto_maxrun_min));
    EEPROM_ReadBuffer(0x0304, (uint8_t*)&auto_retry_limit,sizeof(auto_retry_limit));
}

bool ModelHandle_IsAutoActive(void)
{
    return autoActive;
}

/***************************************************************
 * ============= EEPROM STATE SAVE (ONLY MODE FLAG) =============
 ***************************************************************/
void ModelHandle_SaveCurrentStateToEEPROM(void)
{
    RTC_PersistState s;
    memset(&s, 0, sizeof(s));

    /* Encode mode */
    if      (manualActive)    s.mode = 1;
    else if (semiAutoActive)  s.mode = 2;
    else if (timerActive)     s.mode = 3;
    else if (countdownActive) s.mode = 4;
    else if (twistActive)     s.mode = 5;
    else if (autoActive)      s.mode = 6;
    else                      s.mode = 0;

    RTC_SavePersistentState(&s);
}

/***************************************************************
 * ===================== TIMER SLOT SETTER ======================
 ***************************************************************/
void ModelHandle_SetTimerSlot(uint8_t slot,
                              uint8_t onH, uint8_t onM,
                              uint8_t offH, uint8_t offM)
{
    if (slot >= 5) return;

    timerSlots[slot].onHour    = onH;
    timerSlots[slot].onMinute  = onM;
    timerSlots[slot].offHour   = offH;
    timerSlots[slot].offMinute = offM;
}
