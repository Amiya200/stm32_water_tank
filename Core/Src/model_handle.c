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
volatile bool senseDryRun         = false;   // TRUE = WATER PRESENT
volatile bool senseOverLoad       = false;
volatile bool senseOverUnderVolt  = false;
volatile bool senseMaxRunReached  = false;
// USER CONFIGURABLE SETTINGS (persistent)
static uint16_t user_gap_s        = 10;     // Default
static uint8_t  user_retry_count  = 3;
static uint16_t user_uv_limit     = 180;
static uint16_t user_ov_limit     = 260;
static float    user_overload     = 6.5f;
static float    user_underload    = 0.5f;
static uint16_t user_maxrun_min   = 120;
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

SystemSettings sys = {
    .gap_time_s = 10,
    .retry_count = 3,
    .uv_limit = 180,
    .ov_limit = 260,
    .overload = 6.5f,
    .underload = 0.5f,
    .maxrun_min = 120
};

extern HAL_StatusTypeDef EEPROM_WriteBytes(uint16_t addr, uint8_t *data, uint16_t len);
extern HAL_StatusTypeDef EEPROM_ReadBytes(uint16_t addr, uint8_t *data, uint16_t len);

uint16_t sys_gap_time_s;
uint8_t  sys_retry_count;

uint16_t sys_uv_limit;
uint16_t sys_ov_limit;
extern float g_currentA;
extern float g_voltageV;

float sys_overload_limit;
float sys_underload_limit;

uint16_t sys_maxrun_min;
void ModelHandle_SaveSettingsToEEPROM(void)
{
    EEPROM_WriteBuffer(EE_ADDR_GAP_TIME,       (uint8_t*)&sys.gap_time_s,    sizeof(sys.gap_time_s));
    EEPROM_WriteBuffer(EE_ADDR_RETRY_COUNT,    (uint8_t*)&sys.retry_count,   sizeof(sys.retry_count));
    EEPROM_WriteBuffer(EE_ADDR_UV_LIMIT,       (uint8_t*)&sys.uv_limit,      sizeof(sys.uv_limit));
    EEPROM_WriteBuffer(EE_ADDR_OV_LIMIT,       (uint8_t*)&sys.ov_limit,      sizeof(sys.ov_limit));
    EEPROM_WriteBuffer(EE_ADDR_OVERLOAD,       (uint8_t*)&sys.overload,      sizeof(sys.overload));
    EEPROM_WriteBuffer(EE_ADDR_UNDERLOAD,      (uint8_t*)&sys.underload,     sizeof(sys.underload));
    EEPROM_WriteBuffer(EE_ADDR_MAXRUN,         (uint8_t*)&sys.maxrun_min,    sizeof(sys.maxrun_min));

    uint16_t sig = SETTINGS_SIGNATURE;
    EEPROM_WriteBuffer(EE_ADDR_SIGNATURE, (uint8_t*)&sig, sizeof(sig));
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

void ModelHandle_LoadSettingsFromEEPROM(void)
{
    uint16_t sig = 0;
    EEPROM_ReadBuffer(EE_ADDR_SIGNATURE, (uint8_t*)&sig, sizeof(sig));

    if (sig != SETTINGS_SIGNATURE)
    {
        // First boot → save defaults
        ModelHandle_SaveSettingsToEEPROM();
        return;
    }

    EEPROM_ReadBuffer(EE_ADDR_GAP_TIME,    (uint8_t*)&sys.gap_time_s,    sizeof(sys.gap_time_s));
    EEPROM_ReadBuffer(EE_ADDR_RETRY_COUNT, (uint8_t*)&sys.retry_count,   sizeof(sys.retry_count));
    EEPROM_ReadBuffer(EE_ADDR_UV_LIMIT,    (uint8_t*)&sys.uv_limit,      sizeof(sys.uv_limit));
    EEPROM_ReadBuffer(EE_ADDR_OV_LIMIT,    (uint8_t*)&sys.ov_limit,      sizeof(sys.ov_limit));
    EEPROM_ReadBuffer(EE_ADDR_OVERLOAD,    (uint8_t*)&sys.overload,      sizeof(sys.overload));
    EEPROM_ReadBuffer(EE_ADDR_UNDERLOAD,   (uint8_t*)&sys.underload,     sizeof(sys.underload));
    EEPROM_ReadBuffer(EE_ADDR_MAXRUN,      (uint8_t*)&sys.maxrun_min,    sizeof(sys.maxrun_min));
}
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

/***************************************************************
 *  MANUAL OVERRIDE FLAG
 ***************************************************************/
volatile bool manualOverride = false;

/***************************************************************
 *  INTERNAL UTILITY
 ***************************************************************/
static inline uint32_t now_ms(void)
{
    return HAL_GetTick();
}
uint16_t ModelHandle_GetGapTime(void) {
    return user_gap_s;
}

uint8_t ModelHandle_GetRetryCount(void) {
    return user_retry_count;
}

uint16_t ModelHandle_GetUnderVolt(void) {
    return user_uv_limit;
}

uint16_t ModelHandle_GetOverVolt(void) {
    return user_ov_limit;
}

float ModelHandle_GetOverloadLimit(void) {
    return user_overload;
}

float ModelHandle_GetUnderloadLimit(void) {
    return user_underload;
}

uint16_t ModelHandle_GetMaxRunTime(void) {
    return user_maxrun_min;
}
void ModelHandle_SetUserSettings(
    uint16_t gap_s,
    uint8_t retry,
    uint16_t uv,
    uint16_t ov,
    float overload,
    float underload,
    uint16_t maxrun_min
){
    sys.gap_time_s = gap_s;
    sys.retry_count = retry;
    sys.uv_limit = uv;
    sys.ov_limit = ov;
    sys.overload = overload;
    sys.underload = underload;
    sys.maxrun_min = maxrun_min;

    ModelHandle_SaveSettingsToEEPROM();
}

void ModelHandle_FactoryReset(void)
{
    user_gap_s       = 10;
    user_retry_count = 3;
    user_uv_limit    = 180;
    user_ov_limit    = 260;
    user_overload    = 6.5f;
    user_underload   = 0.5f;
    user_maxrun_min  = 120;

    // Optional: EEPROM_SaveDefaults();
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
 *  MANUAL MODE CONTROL
 ***************************************************************/
void ModelHandle_SetMotor(bool on)
{
    clear_all_modes();
    manualOverride = true;

    /* In manual mode dry-run is disabled */
    senseDryRun = false;

    if (on) start_motor();
    else    stop_motor();
}

/* RESET pump logic (SW1 short press) */
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
 *  OVERLOAD + UNDERLOAD CHECK FUNCTION (Modes stay active)
 ***************************************************************/
static uint32_t faultLockTimestamp = 0;
static bool faultLocked = false;

#define LOAD_FAULT_CONFIRM_MS     3000   // confirm overload/underload for 3 seconds
#define LOAD_RETRY_RUN_MS         3000   // run motor for 3 seconds during retry
#define LOAD_LOCK_DURATION_MS     (30UL * 60UL * 1000UL)   // 30 minutes

typedef enum {
    LOAD_NORMAL = 0,
    LOAD_FAULT_WAIT,
    LOAD_FAULT_LOCK,
    LOAD_RETRY_RUN
} LoadFaultState;

static LoadFaultState loadState = LOAD_NORMAL;
static uint32_t loadTimer = 0;


void ModelHandle_CheckLoadFault(void)
{
    float I = g_currentA;
    float V = g_voltageV;

    float ol = ModelHandle_GetOverloadLimit();
    float ul = ModelHandle_GetUnderloadLimit();
    uint16_t uv = ModelHandle_GetUnderVolt();
    uint16_t ov = ModelHandle_GetOverVolt();

    uint32_t now = HAL_GetTick();

    bool overload  = (I > ol);
    bool underload = (I < ul);
    bool voltFault = (V < uv || V > ov);
    bool loadFault = (overload || underload);

    /***********************************************************
     * LED INDICATION PRIORITY LOGIC
     ***********************************************************/
    if (voltFault)
    {
        LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_BLINK, 250);
    }
    else if (loadFault)
    {
        LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 250);
    }
    else
    {
        LED_SetIntent(LED_COLOR_BLUE, LED_MODE_OFF, 0); // Ensure blue off
        LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_OFF, 0); // Ensure purple off
    }

    /***********************************************************
     * LOAD + VOLTAGE PROTECTION — USE SAME FSM
     ***********************************************************/
    bool fault = loadFault || voltFault;

    switch (loadState)
    {
        case LOAD_NORMAL:
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
                loadState = LOAD_FAULT_LOCK;
                loadTimer = now;
            }
            break;

        case LOAD_FAULT_LOCK:
            if (now - loadTimer >= LOAD_LOCK_DURATION_MS)
            {
                if (isAnyModeActive())
                {
                    start_motor();
                    loadState = LOAD_RETRY_RUN;
                    loadTimer = now;
                }
            }
            break;

        case LOAD_RETRY_RUN:
            if (now - loadTimer >= LOAD_RETRY_RUN_MS)
            {
                if (fault)
                {
                    stop_motor();
                    loadState = LOAD_FAULT_LOCK;
                    loadTimer = now;
                }
                else
                {
                    loadState = LOAD_NORMAL;
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
    manualActive     = false;
    semiAutoActive   = false;
    timerActive      = false;
    countdownActive  = false;
    twistActive      = false;
    autoActive       = false;
    ModelHandle_SaveModeState();
    ModelHandle_SetMotor(false);   // turn OFF motor safely
}

static inline bool isTankFull(void)
{
    static uint32_t stableStart = 0;
    static bool lastState = false;

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
 *  DRY-RUN SENSOR CHECK
 *  Voltage <=0.01f → WATER PRESENT → dry=false
 *  Voltage > 0.01f → DRY → dry=true
 ***************************************************************/
void ModelHandle_CheckDryRun(void)
{
    /* Countdown, Semi-Auto, Manual ignore dry-run */
    if (manualActive || semiAutoActive || countdownActive)
    {
        senseDryRun = false;
        return;
    }

    float v = adcData.voltages[0];

    if (v > 0.01f)
        senseDryRun = false;   // WATER PRESENT
    else
        senseDryRun = true;    // DRY
}
void ModelHandle_ToggleManual(void)
{
    // Stop all other modes before manual
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

static DryFSMState dryState = DRY_IDLE;
static bool dryConfirming   = false;


void ModelHandle_SoftDryRunHandler(void)
{
    uint32_t now = now_ms();

    /***********************************************************
     * 1) COUNTDOWN MODE → Dry-run fully bypassed
     ***********************************************************/
    if (countdownActive)
    {
        dryState       = DRY_IDLE;
        dryConfirming  = false;
        dryDeadline    = 0;
        dryConfirmStart= 0;
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
        dryState       = DRY_IDLE;
        dryConfirming  = false;
        dryDeadline    = 0;
        return;
    }

    /***********************************************************
     * 3) FSM OPERATION
     ***********************************************************/
    switch (dryState)
    {
    case DRY_IDLE:
        if (senseDryRun)  // water present
        {
            start_motor();
            dryState = DRY_NORMAL;
        }
        else              // dry
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
        if (senseDryRun)   // water found
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
        if (!senseDryRun)   // DRY detected
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

/***************************************************************
 *  LEGACY ENTRY POINT
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
    clear_all_modes(); /* original must NOT clear timerActive */
    timerActive = true;

    RTC_GetTimeDate();

    uint16_t nowHM = time.hour * 60 + time.min;

    uint16_t bestDiff = 20000;
    int best = -1;

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
            best = i;
        }
    }

    /* Apply */
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

    senseDryRun = true; /* ignore dry run */

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

    senseDryRun = true;

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
    senseDryRun = true;

    ModelHandle_TimerRecalculateNow();
}

void ModelHandle_StopTimer(void)
{
    timerActive = false;
    stop_motor();
}

/***************************************************************
 * Legacy timer tick (kept minimal)
 ***************************************************************/
static void timer_tick(void)
{
    ModelHandle_ProcessTimerSlots();
}
void ModelHandle_CheckAutoTimerActivation(void)
{
    if (timerActive)
        return;

    if (timer_any_active_slot())
    {
        timerActive = true;
        senseDryRun = true;
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
    /* Semi-auto ignores dry-run completely */
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
 * ======================== AUTO MODE ==========================
 ***************************************************************/

/* Auto settings */
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

static AutoState autoState = AUTO_IDLE;
static uint32_t autoDeadline = 0;
static uint32_t autoRunStart = 0;

/* Start AUTO mode */
void ModelHandle_StartAuto(uint16_t gap_s, uint16_t maxrun_min, uint16_t retry)
{
    clear_all_modes();

    autoActive = true;
    ModelHandle_SaveModeState();
    auto_gap_s = gap_s;
    auto_maxrun_min = maxrun_min;
    auto_retry_limit = retry;
    auto_retry_count = 0;

    autoState = AUTO_ON_WAIT;

    start_motor();
    autoRunStart = now_ms();
    autoDeadline = now_ms() + (gap_s * 1000UL);
}

/* Stop AUTO */
void ModelHandle_StopAuto(void)
{
    autoActive = false;
    autoState = AUTO_IDLE;
    auto_retry_count = 0;

    stop_motor();
}

/***************************************************************
 *  AUTO MODE TICK ENGINE
 ***************************************************************/
static void auto_tick(void)
{
    if (!autoActive)
        return;

    uint32_t now = now_ms();

    /* Protection events → immediate shutdown */
    if (senseOverLoad || senseOverUnderVolt)
    {
        ModelHandle_StopAuto();
        return;
    }

    /* Tank full → normal stop */
    if (isTankFull())
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

        if (!senseDryRun)     // DRY condition
        {
            stop_motor();
            autoState = AUTO_OFF_WAIT;
            autoDeadline = now + (auto_gap_s * 1000UL);
        }
        else                  // water present
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
 * ======================== COUNTDOWN MODE ======================
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
    ModelHandle_SaveModeState();
    cd_deadline = now_ms() + seconds * 1000UL;

    start_motor();   // countdown ALWAYS forces motor ON
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

    if (on_s == 0)  on_s = 1;
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
    if (senseOverLoad || senseOverUnderVolt)
    {
        ModelHandle_StopAllModesAndMotor();
        return;
    }
}

/***************************************************************
 * =========================== LEDS =============================
 ***************************************************************/
static void leds_from_model(void)
{
    LED_ClearAllIntents();

    /* Motor ON → GREEN steady */
    if (Motor_GetStatus())
        LED_SetIntent(LED_COLOR_GREEN, LED_MODE_STEADY, 0);

    /* Auto/timer retry (dry-run) → GREEN blink */
    if (!senseDryRun && (autoActive || timerActive))
        LED_SetIntent(LED_COLOR_GREEN, LED_MODE_BLINK, 350);

    /* Pump OFF due to dry-run → RED steady */
    if (!senseDryRun && !Motor_GetStatus())
        LED_SetIntent(LED_COLOR_RED, LED_MODE_STEADY, 0);

    /* Max run reached → RED blink */
    if (senseMaxRunReached)
        LED_SetIntent(LED_COLOR_RED, LED_MODE_BLINK, 300);

    /* Overload → BLUE blink */
    if (senseOverLoad)
        LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 350);

    /* Voltage fault → PURPLE blink */
    if (senseOverUnderVolt)
        LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_BLINK, 350);

    LED_ApplyIntents();
}

/***************************************************************
 * ========================== MASTER FSM ========================
 ***************************************************************/
void ModelHandle_Process(void)
{
    protections_tick();
    twist_time_logic();

    /*********************** AUTO *************************/
    if (autoActive)
    {
        auto_tick();
        leds_from_model();
        return;
    }

    /********************* SEMI-AUTO ***********************/
    if (semiAutoActive)
    {
        senseDryRun = false;   // ignored in semi-auto

        if (!isTankFull())
        {
            if (!Motor_GetStatus())
                start_motor();
        }
        else
        {
            stop_motor();
            semiAutoActive = false;   // auto-finish
        }

        leds_from_model();
        return;
    }

    /*********************** TIMER *************************/
    if (timerActive)
    {
        timer_tick();
        leds_from_model();
        return;
    }



    /********************* COUNTDOWN ***********************/
    if (countdownActive)
    {
        countdown_tick();

        /* Tank full cancels countdown */
        if (isTankFull())
        {
            ModelHandle_StopCountdown();
            leds_from_model();
            return;
        }

        /* ALWAYS keep motor ON */
        if (!Motor_GetStatus())
            start_motor();

        leds_from_model();
        return;
    }

    /************************ TWIST ************************/
    if (twistActive)
    {
        twist_tick();
        leds_from_model();
        return;
    }

    /************************ MANUAL ***********************/
    if (manualActive)
    {
        leds_from_model();
        return;
    }

    /********************** IDLE ***************************/
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
    senseMaxRunReached  = false;

    countdownDuration = 0;

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

    // Optional: save to EEPROM
    EEPROM_WriteBuffer(0x0300, (uint8_t*)&auto_gap_s, sizeof(auto_gap_s));
    EEPROM_WriteBuffer(0x0302, (uint8_t*)&auto_maxrun_min, sizeof(auto_maxrun_min));
    EEPROM_WriteBuffer(0x0304, (uint8_t*)&auto_retry_limit, sizeof(auto_retry_limit));
}
void ModelHandle_LoadAutoSettings()
{
    EEPROM_ReadBuffer(0x0300, (uint8_t*)&auto_gap_s, sizeof(auto_gap_s));
    EEPROM_ReadBuffer(0x0302, (uint8_t*)&auto_maxrun_min, sizeof(auto_maxrun_min));
    EEPROM_ReadBuffer(0x0304, (uint8_t*)&auto_retry_limit, sizeof(auto_retry_limit));
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

