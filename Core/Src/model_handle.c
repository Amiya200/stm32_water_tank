/***************************************************************
 *  HELONIX Water Pump Controller
 *  MODEL HANDLE – FINAL (2025)
 *  ✔ ALL MODES WORKING
 *  ✔ TIMER MODE FIXED
 *  ✔ EEPROM SAFE (RESET PROOF)
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
#include "main.h"          // BUZZER: LED5
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
extern float g_currentA;
extern float g_voltageV;

/***************************************************************
 * ================= TIMER EEPROM SAFE BLOCK ===================
 ***************************************************************/
#define TIMER_EE_SIGNATURE  0x544D   /* 'TM' */
#define TIMER_EE_VERSION    1
#define EE_ADDR_TIMER_BLOCK 0x7000    // last EEPROM page (safe)


typedef struct {
    uint16_t signature;
    uint8_t  version;
    uint8_t  reserved;
    TimerSlot slots[5];
    uint16_t crc;
} TimerEEPROMBlock;

/***************************************************************
 * CRC16 (EEPROM SAFETY)
 ***************************************************************/
static uint16_t Timer_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

/***************************************************************
 * TIMER SLOTS (5)
 ***************************************************************/
TimerSlot timerSlots[5] = {0};
typedef enum {
    TIMER_RUN = 0,
    TIMER_GAP_WAIT
} TimerDryState;

static TimerDryState timerDryState = TIMER_RUN;
static uint32_t timerGapDeadline = 0;

/***************************************************************
 * MODE FLAGS
 ***************************************************************/
volatile bool manualActive    = false;
volatile bool semiAutoActive  = false;
volatile bool countdownActive = false;
volatile bool twistActive     = false;
volatile bool timerActive     = false;
volatile bool autoActive      = false;

/***************************************************************
 * MOTOR STATUS
 ***************************************************************/
volatile uint8_t motorStatus = 0;

/***************************************************************
 * PROTECTION FLAGS
 ***************************************************************/
volatile bool senseDryRun         = false;
volatile bool senseOverLoad       = false;
volatile bool senseUnderLoad      = false;
volatile bool senseOverUnderVolt  = false;
volatile bool senseMaxRunReached  = false;
volatile bool manualOverride      = false;

/* Legacy externs */
volatile uint16_t auto_retry_counter = 0;
volatile bool     countdownMode      = false;
volatile uint32_t countdownDuration  = 0;

/***************************************************************
 * TWIST SETTINGS
 ***************************************************************/
TwistSettings twistSettings;
/***************************************************************
 * ===================== DEVICE SETTINGS =======================
 ***************************************************************/
typedef struct {
    bool    manual_on;
    bool    semi_on;
    bool    timer_on;
    bool    countdown_on;
    bool    twist_on;
    bool    auto_on;
    bool    motor_on;
    uint8_t power_restore_mode; /* 0=YES, 1=NO, 2=LAST */
} ModeState;

static ModeState modeState;
#define EE_ADDR_COUNTDOWN_BLOCK 0x1100
#define CD_SIGNATURE 0xCD55

typedef struct{
    uint16_t sig;
    uint32_t remaining;
    uint8_t  active;
    uint16_t crc;
} CountdownBlock;

/* Power Restore runtime copy */
static uint8_t powerRestoreMode = 0;

/* System defaults */
SystemSettings sys = {
    .gap_time_s  = 0,
    .retry_count = 0,
    .uv_limit    = 190,
    .ov_limit    = 270,
    .overload    = 0.0f,
    .underload   = 0.0f,
    .maxrun_min  = 300
};
/***************************************************************
 * ========== INTERNAL FORWARD DECLARATIONS (FIX) ==============
 ***************************************************************/

/* Motor helpers */
static inline void start_motor(void);
static inline void stop_motor(void);

/* Mode helpers */
static inline void clear_all_modes(void);
static inline bool isAnyModeActive(void);

/* Dry-run */
void ModelHandle_SoftDryRunHandler(void);

/* Timer helpers */
static bool     timer_any_active_slot(void);
static uint16_t get_active_timer_gap_minutes(void);
static uint8_t  get_today_mask(void);
static bool     slot_is_active_now(const TimerSlot *t);

/* Tank */
static bool isTankFull(void);

static void SaveCountdown(void);
/* FSM ticks */
static void auto_tick(void);
static void countdown_tick(void);
static void twist_tick(void);
static void twist_time_logic(void);
static void protections_tick(void);
static void leds_from_model(void);

/***************************************************************
 * ================= EEPROM SETTINGS SAVE ======================
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

/***************************************************************
 * ================= EEPROM SETTINGS LOAD ======================
 ***************************************************************/
void ModelHandle_LoadSettingsFromEEPROM(void)
{
    uint16_t sig = 0;
    EEPROM_ReadBuffer(EE_ADDR_SIGNATURE, (uint8_t*)&sig, sizeof(sig));

    if (sig != SETTINGS_SIGNATURE)
    {
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
 * ================= MODE STATE SAVE ===========================
 ***************************************************************/
void ModelHandle_SaveModeState(void)
{
    modeState.manual_on          = manualActive;
    modeState.semi_on            = semiAutoActive;
    modeState.timer_on           = timerActive;
    modeState.countdown_on       = countdownActive;
    modeState.twist_on           = twistActive;
    modeState.auto_on            = autoActive;
    modeState.motor_on           = (motorStatus == 1);
    modeState.power_restore_mode = powerRestoreMode;

    EEPROM_WriteBuffer(0x0200, (uint8_t*)&modeState, sizeof(modeState));
}

/***************************************************************
 * ================= MODE STATE LOAD ===========================
 ***************************************************************/
void ModelHandle_LoadModeState(void)
{
    EEPROM_ReadBuffer(0x0200, (uint8_t*)&modeState, sizeof(modeState));

    powerRestoreMode = modeState.power_restore_mode;
    if (powerRestoreMode > 2) powerRestoreMode = 0;

    if (powerRestoreMode == 1) /* NO */
    {
        manualActive = semiAutoActive = countdownActive =
        twistActive  = timerActive = autoActive = false;
        return;
    }

    manualActive    = modeState.manual_on;
    semiAutoActive  = modeState.semi_on;
    timerActive     = modeState.timer_on;
    countdownActive = modeState.countdown_on;
    twistActive     = modeState.twist_on;
    autoActive      = modeState.auto_on;
}

/***************************************************************
 * ================= TIMER EEPROM SAVE (SAFE) =================
 ***************************************************************/
void ModelHandle_SaveTimerToEEPROM(void)
{
    TimerEEPROMBlock blk;
    memset(&blk, 0, sizeof(blk));

    blk.signature = TIMER_EE_SIGNATURE;
    blk.version   = TIMER_EE_VERSION;
    memcpy(blk.slots, timerSlots, sizeof(timerSlots));

    blk.crc = Timer_CRC16((uint8_t*)&blk,
                          sizeof(blk) - sizeof(uint16_t));

    EEPROM_WriteBuffer(EE_ADDR_TIMER_BLOCK,
                       (uint8_t*)&blk,
                       sizeof(blk));
}

/***************************************************************
 * ================= TIMER EEPROM LOAD (SAFE) =================
 ***************************************************************/
void ModelHandle_LoadTimerFromEEPROM(void)
{
    TimerEEPROMBlock blk;
    EEPROM_ReadBuffer(EE_ADDR_TIMER_BLOCK,(uint8_t*)&blk,sizeof(blk));

    if(blk.signature != TIMER_EE_SIGNATURE) return;

    uint16_t crc = Timer_CRC16((uint8_t*)&blk, sizeof(blk)-2);
    if(blk.crc != crc) return;     // <<< CRITICAL

    memcpy(timerSlots, blk.slots, sizeof(timerSlots));
}


void Timer_EEPROM_EnsureValid(void)
{
    TimerEEPROMBlock blk;
    EEPROM_ReadBuffer(EE_ADDR_TIMER_BLOCK,(uint8_t*)&blk,sizeof(blk));

    uint16_t crc = Timer_CRC16((uint8_t*)&blk,sizeof(blk)-2);

    if(blk.signature!=TIMER_EE_SIGNATURE || blk.crc!=crc)
    {
        memset(timerSlots,0,sizeof(timerSlots));
        ModelHandle_SaveTimerToEEPROM();
    }
}



/***************************************************************
 * ============== FORWARD DECLARATIONS (REQUIRED) ==============
 ***************************************************************/

/* TIMER helpers */
static bool     timer_any_active_slot(void);
static uint16_t get_active_timer_gap_minutes(void);
static uint8_t  get_today_mask(void);
static bool     slot_is_active_now(const TimerSlot *t);

/* Tank level */
static bool     isTankFull(void);

/* Mode helpers */
static inline bool isAnyModeActive(void);

/* FSM ticks */
static void     auto_tick(void);
static void     countdown_tick(void);
static void     twist_tick(void);
static void     twist_time_logic(void);
static void     protections_tick(void);
static void     leds_from_model(void);
/***************************************************************
 * ================= TANK FULL DETECTION ======================
 ***************************************************************/
static bool isTankFull(void)
{
    static uint32_t stableStart = 0;
    static bool lastState = false;

    bool allZero = true;

    /* ADC channels 1..5 = tank probes */
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

        if ((now - stableStart) >= 1000)
            return true;
    }
    else
    {
        lastState = false;
    }

    return false;
}
/***************************************************************
 * ============ MISSING PUBLIC API RESTORE ====================
 ***************************************************************/

/* Called from main.c / screen.c */
void ModelHandle_TimerRecalculateNow(void)
{
    if (!timerActive) return;
    ModelHandle_ProcessTimerSlots();
}

void ModelHandle_ProcessDryRun(void)
{
    ModelHandle_SoftDryRunHandler();
}

/* User settings */
uint16_t ModelHandle_GetGapTime(void)        { return sys.gap_time_s; }
uint8_t  ModelHandle_GetRetryCount(void)     { return sys.retry_count; }
uint16_t ModelHandle_GetUnderVolt(void)      { return sys.uv_limit; }
uint16_t ModelHandle_GetOverVolt(void)       { return sys.ov_limit; }
float    ModelHandle_GetOverloadLimit(void)  { return sys.overload; }
float    ModelHandle_GetUnderloadLimit(void) { return sys.underload; }
uint16_t ModelHandle_GetMaxRunTime(void)     { return sys.maxrun_min; }

uint8_t ModelHandle_GetPowerRestoreMode(void)
{
    return powerRestoreMode;
}

void ModelHandle_SetPowerRestoreMode(uint8_t mode)
{
    if (mode > 2) mode = 0;
    powerRestoreMode = mode;
    ModelHandle_SaveModeState();
}

/* Manual */
void ModelHandle_ToggleManual(void)
{
    manualActive = !manualActive;
    if (manualActive) start_motor();
    else stop_motor();
    ModelHandle_SaveModeState();
}

void ModelHandle_StopAllModesAndMotor(void)
{
    clear_all_modes();
    stop_motor();
    ModelHandle_SaveModeState();
}

/* Factory reset */
void ModelHandle_FactoryReset(void)
{
    sys.gap_time_s  = 0;
    sys.retry_count = 0;
    sys.uv_limit    = 190;
    sys.ov_limit    = 270;
    sys.overload    = 0.0f;
    sys.underload   = 0.0f;
    sys.maxrun_min  = 300;

    ModelHandle_SaveSettingsToEEPROM();
}

/* Timer helper */
void ModelHandle_StartTimerNearestSlot(void)
{
    clear_all_modes();
    timerActive = true;
    ModelHandle_ProcessTimerSlots();
}
static inline void Buzzer_SetPin(bool on)
{
    HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint32_t buzzerAlertUntil = 0;

static void Buzzer_TriggerAlert(void)
{
    buzzerAlertUntil = HAL_GetTick() + 30000UL;
}

static void Buzzer_Update(void)
{
    uint32_t now = HAL_GetTick();
    bool motorOn = Motor_GetStatus();
    bool alert   = (now < buzzerAlertUntil);

    static bool buzzerState = false;
    bool newState = false;

    if (alert)
    {
        newState = ((now % 600UL) < 200UL);
    }
    else if (motorOn)
    {
        newState = ((now % 800UL) < 150UL);
    }

    if (newState != buzzerState)
    {
        buzzerState = newState;
        Buzzer_SetPin(buzzerState);
    }
}
/* Reset key (SW1) */
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
            break;

        HAL_Delay(100);
    }

    /* After 5s: if still dry → stop motor and exit */
    if (senseDryRun)
    {
        stop_motor();
        manualOverride = false;
        Buzzer_TriggerAlert();      // <<< BUZZER: tank empty / dry run
        return;
    }

    /* 2) Water available → keep motor ON until tank is full or fault */
    while (!isTankFull())
    {
        ModelHandle_CheckLoadFault();
        if (senseOverLoad || senseUnderLoad || senseOverUnderVolt || senseMaxRunReached)
        {
            stop_motor();
            manualOverride = false;
            Buzzer_TriggerAlert();  // <<< BUZZER: load/volt fault
            return;
        }

        ModelHandle_CheckDryRun();
        if (senseDryRun)
        {
            stop_motor();
            manualOverride = false;
            Buzzer_TriggerAlert();  // <<< BUZZER: dry run while reset
            return;
        }

        HAL_Delay(100);
    }

    /* Tank full → stop motor */
    stop_motor();
    manualOverride = false;
    Buzzer_TriggerAlert();          // <<< BUZZER: tank full
}


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
    countdownMode   = false;
}

/***************************************************************
 * ================= MOTOR CONTROL =============================
 ***************************************************************/
static uint32_t motorOnStartMs = 0;
static uint32_t powerOnMs     = 0;

void ModelHandle_OnPowerUp(void)
{
    powerOnMs = HAL_GetTick();
}

static bool Motor_StartAllowed(void)
{
    return (HAL_GetTick() - powerOnMs) >= 7000UL; /* 7s delay */
}

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
    {
        if (!Motor_StartAllowed())
            return;

        motorOnStartMs = HAL_GetTick();
    }

    Relay_Set(1, on);
    motorStatus = on ? 1 : 0;
    UART_SendStatusPacket();
}

static inline void start_motor(void) { motor_apply(true); }
static inline void stop_motor(void)  { motor_apply(false); }

/***************************************************************
 * ================= BUZZER DRIVER =============================
 ***************************************************************/


/***************************************************************
 * ================= MAX RUN PROTECTION ========================
 ***************************************************************/
static void check_max_run(void)
{
    if (sys.maxrun_min == 0 || !Motor_GetStatus())
        return;

    if (countdownActive)
        return;

    uint32_t limit = (uint32_t)sys.maxrun_min * 60000UL;
    if ((HAL_GetTick() - motorOnStartMs) >= limit)
    {
        senseMaxRunReached = true;
        clear_all_modes();
        stop_motor();
        ModelHandle_SaveModeState();
        Buzzer_TriggerAlert();
    }
}

/***************************************************************
 * ================= DRY RUN SENSOR ============================
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

    if (v < 0.01f)
        senseDryRun = true;   /* DRY */
    else
        senseDryRun = false;  /* WATER OK */
}


/***************************************************************
 * ================= SOFT DRY RUN FSM ==========================
 ***************************************************************/
#define DRY_PROBE_ON_MS   5000UL
#define DRY_CONFIRM_MS    1500UL

static uint32_t dryDeadline     = 0;
static uint32_t dryConfirmStart = 0;
static uint32_t dryOffGapMs     = 10000UL;

typedef enum {
    DRY_IDLE = 0,
    DRY_PROBE,
    DRY_NORMAL
} DryFSMState;

static DryFSMState dryState      = DRY_IDLE;
static bool        dryConfirming = false;

static inline bool isAnyModeActive(void)
{
    return (manualActive || semiAutoActive || countdownActive ||
            twistActive || timerActive || autoActive);
}

void ModelHandle_SoftDryRunHandler(void)
{
    uint32_t now = now_ms();

    if (timerActive)
        return;   // timer has its own dry-run logic

    uint16_t gap_s = sys.gap_time_s;
    if (timerActive)
    {
        uint16_t slotGap = get_active_timer_gap_minutes();
        if (slotGap > 0) gap_s = slotGap * 60U;
    }

    ModelHandle_CheckDryRun();
    if (gap_s == 0) return;

    dryOffGapMs = (uint32_t)gap_s * 1000UL;

    if (manualActive || semiAutoActive || countdownActive)
    {
        dryState = DRY_IDLE;
        dryConfirming = false;
        return;
    }

    if (!isAnyModeActive())
    {
        stop_motor();
        dryState = DRY_IDLE;
        return;
    }

    switch (dryState)
    {
        case DRY_IDLE:
            if (!senseDryRun)
            {
                start_motor();
                dryState = DRY_NORMAL;
            }
            else if (now >= dryDeadline)
            {
                start_motor();
                dryState = DRY_PROBE;
                dryDeadline = now + DRY_PROBE_ON_MS;
            }
            break;

        case DRY_PROBE:
            if (!senseDryRun)
            {
                dryState = DRY_NORMAL;
            }
            else if (now >= dryDeadline)
            {
                stop_motor();
                dryState = DRY_IDLE;
                dryDeadline = now + dryOffGapMs;
                Buzzer_TriggerAlert();
            }
            break;

        case DRY_NORMAL:
            if (senseDryRun)
            {
                if (!dryConfirming)
                {
                    dryConfirming = true;
                    dryConfirmStart = now;
                }
                else if ((now - dryConfirmStart) >= DRY_CONFIRM_MS)
                {
                    stop_motor();
                    dryState = DRY_IDLE;
                    dryConfirming = false;
                    dryDeadline = now + dryOffGapMs;
                    Buzzer_TriggerAlert();
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
 * ================= LOAD / VOLTAGE FSM ========================
 ***************************************************************/
#define LOAD_FAULT_CONFIRM_MS 3000UL
#define LOAD_RETRY_RUN_MS     3000UL
#define LOAD_LOCK_DEFAULT_MS (20UL * 60UL * 1000UL)
#define TIMER_RETRY_30MIN_MS (30UL * 60UL * 1000UL)

typedef enum {
    LOAD_NORMAL = 0,
    LOAD_FAULT_WAIT,
    LOAD_FAULT_LOCK,
    LOAD_RETRY_RUN
} LoadFaultState;

static LoadFaultState loadState = LOAD_NORMAL;
static uint32_t loadTimer = 0;
static uint8_t  loadRetryCount = 0;

static inline uint32_t get_load_lock_duration_ms(void)
{
    if (sys.retry_count == 0)
        return LOAD_LOCK_DEFAULT_MS;
    return (uint32_t)sys.retry_count * 60000UL;
}

void ModelHandle_CheckLoadFault(void)
{
    float I = g_currentA;
    float V = g_voltageV;

    bool overload  = (sys.overload > 0.1f) && (I > sys.overload);
    bool underload = (sys.underload > 0.001f) && (I < sys.underload);
    bool voltFault = ((sys.uv_limit && V < sys.uv_limit) ||
                      (sys.ov_limit && V > sys.ov_limit));

    senseOverLoad      = overload;
    senseUnderLoad     = underload;
    senseOverUnderVolt = voltFault;

    bool fault = overload || underload || voltFault;
    uint32_t now = HAL_GetTick();

    uint32_t lockMs = (timerActive && timer_any_active_slot())
                        ? TIMER_RETRY_30MIN_MS
                        : get_load_lock_duration_ms();

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
            }
            else if ((now - loadTimer) >= LOAD_FAULT_CONFIRM_MS)
            {
                stop_motor();
                loadState = LOAD_FAULT_LOCK;
                loadTimer = now;
                Buzzer_TriggerAlert();
            }
            break;

        case LOAD_FAULT_LOCK:
            if (!fault &&
                (autoActive || (timerActive && timer_any_active_slot())))
            {
                loadState = LOAD_NORMAL;
            }
            else if ((now - loadTimer) >= lockMs && loadRetryCount < 1)
            {
                loadRetryCount++;
                start_motor();
                loadState = LOAD_RETRY_RUN;
                loadTimer = now;
            }
            break;

        case LOAD_RETRY_RUN:
            if ((now - loadTimer) >= LOAD_RETRY_RUN_MS)
            {
                if (fault)
                {
                    stop_motor();
                    loadState = LOAD_FAULT_LOCK;
                    loadTimer = now;
                    Buzzer_TriggerAlert();
                }
                else
                {
                    loadState = LOAD_NORMAL;
                    loadRetryCount = 0;
                }
            }
            break;
    }
}
/***************************************************************
 * ======================= TIMER MODE ==========================
 * DS1307 DOW: 1 = Mon ... 7 = Sun
 * dayMask: bit0 = Mon ... bit6 = Sun
 ***************************************************************/
static uint8_t get_today_mask(void)
{
    uint8_t d = time.dow;
    if (d < 1 || d > 7) d = 1;
    return (1U << (d - 1));
}

static bool slot_is_active_now(const TimerSlot *t)
{
    if (!t->enabled) return false;
    if (!(t->dayMask & get_today_mask())) return false;

    uint16_t now = time.hour * 60 + time.min;
    uint16_t on  = t->onHour  * 60 + t->onMinute;
    uint16_t off = t->offHour * 60 + t->offMinute;

    return (on < off) ? (now >= on && now < off)
                      : (now >= on || now < off);
}

static bool timer_any_active_slot(void)
{
    for (int i = 0; i < 5; i++)
        if (slot_is_active_now(&timerSlots[i]))
            return true;
    return false;
}

static uint16_t get_active_timer_gap_minutes(void)
{
    if (!timerActive) return 0;

    uint16_t now = time.hour * 60 + time.min;
    uint8_t  dm  = get_today_mask();

    for (int i = 0; i < 5; i++)
    {
        TimerSlot *t = &timerSlots[i];
        if (!t->enabled || !(t->dayMask & dm)) continue;

        uint16_t on  = t->onHour * 60 + t->onMinute;
        uint16_t off = t->offHour * 60 + t->offMinute;

        bool active = (on < off) ?
                      (now >= on && now < off) :
                      (now >= on || now < off);

        if (active)
            return t->gapMinutes;
    }
    return 0;
}
static uint32_t timer_get_gap_ms(void)
{
    uint16_t gapMin = get_active_timer_gap_minutes();

    if (gapMin == 0)
        return 0;

    return (uint32_t)gapMin * 60UL * 1000UL;
}

/***************************************************************
 * TIMER PROCESS (CALLED EVERY LOOP)
 ***************************************************************/
typedef enum {
    TIMER_STATE_ON,
    TIMER_STATE_OFF,
    TIMER_STATE_WATER_CONTINUOUS
} TimerState;
static TimerState timerState = TIMER_STATE_ON;
static uint32_t   timerStateDeadline = 0;


void ModelHandle_ProcessTimerSlots(void)
{
    if (!timerActive)
        return;

    /* If slot not active → motor OFF */
    if (!timer_any_active_slot())
    {
        stop_motor();
        timerState = TIMER_STATE_ON;
        return;
    }

    /* Read dry-run sensor */
    ModelHandle_CheckDryRun();

    uint32_t now = HAL_GetTick();
    uint32_t gapMs = timer_get_gap_ms();   // gapMinutes → ms

    if (gapMs == 0)
    {
        /* No gap configured → normal timer behavior */
        start_motor();
        return;
    }

    /* ================= WATER AVAILABLE ================= */
    if (senseDryRun == true)
    {
        timerState = TIMER_STATE_WATER_CONTINUOUS;
        start_motor();
        return;
    }

    /* ================= NO WATER (CYCLIC TEST) ================= */
    switch (timerState)
    {
        case TIMER_STATE_ON:
            start_motor();

            if (timerStateDeadline == 0)
                timerStateDeadline = now + gapMs;

            if (now >= timerStateDeadline)
            {
                stop_motor();
                timerState = TIMER_STATE_OFF;
                timerStateDeadline = now + gapMs;
            }
            break;

        case TIMER_STATE_OFF:
            stop_motor();

            if (now >= timerStateDeadline)
            {
                timerState = TIMER_STATE_ON;
                timerStateDeadline = now + gapMs;
            }
            break;

        case TIMER_STATE_WATER_CONTINUOUS:
            /* Should never stay here if no water */
            timerState = TIMER_STATE_ON;
            timerStateDeadline = now + gapMs;
            break;
    }
}

/***************************************************************
 * START / STOP TIMER MODE
 ***************************************************************/
void ModelHandle_StartTimer(void)
{
    clear_all_modes();
    timerActive = true;
    ModelHandle_SaveTimerToEEPROM();
    ModelHandle_ProcessTimerSlots();
}

void ModelHandle_StopTimer(void)
{
    timerActive = false;
    stop_motor();
    ModelHandle_SaveModeState();
}

/***************************************************************
 * AUTO TIMER ACTIVATION
 ***************************************************************/
void ModelHandle_CheckAutoTimerActivation(void)
{
    if (timerActive) return;

    if (timer_any_active_slot())
    {
        timerActive = true;
        start_motor();
    }
}

/***************************************************************
 * ===================== SEMI AUTO MODE ========================
 ***************************************************************/
void ModelHandle_StartSemiAuto(void)
{
    clear_all_modes();
    semiAutoActive = true;
    ModelHandle_SaveModeState();

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

static AutoState autoState    = AUTO_IDLE;
static uint32_t  autoDeadline = 0;

void ModelHandle_StartAuto(uint16_t gap_s, uint16_t maxrun_min, uint16_t retry)
{
    clear_all_modes();

    autoActive       = true;
    auto_gap_s       = gap_s;
    auto_maxrun_min  = maxrun_min;
    auto_retry_limit = retry;
    auto_retry_count = 0;
    autoState        = AUTO_ON_WAIT;

    ModelHandle_SaveModeState();
    start_motor();
    autoDeadline = now_ms() + gap_s * 1000UL;
}

void ModelHandle_StopAuto(void)
{
    autoActive = false;
    autoState  = AUTO_IDLE;
    auto_retry_count = 0;

    ModelHandle_SaveModeState();
    stop_motor();
}

static void auto_tick(void)
{
    if (!autoActive) return;

    uint32_t now = now_ms();

    if (isTankFull())
    {
        ModelHandle_StopAuto();
        Buzzer_TriggerAlert();
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
            if (!senseDryRun)
            {
                stop_motor();
                autoState = AUTO_OFF_WAIT;
                autoDeadline = now + auto_gap_s * 1000UL;
                Buzzer_TriggerAlert();
            }
            else
            {
                autoState = AUTO_ON_WAIT;
                autoDeadline = now + auto_gap_s * 1000UL;
            }
            break;

        case AUTO_OFF_WAIT:
            if (now >= autoDeadline)
            {
                auto_retry_count++;
                auto_retry_counter = auto_retry_count;

                if (auto_retry_limit &&
                    auto_retry_count > auto_retry_limit)
                {
                    ModelHandle_StopAuto();
                    return;
                }

                start_motor();
                autoState = AUTO_ON_WAIT;
                autoDeadline = now + auto_gap_s * 1000UL;
            }
            break;

        default:
            break;
    }
}

/***************************************************************
 * ====================== COUNTDOWN MODE =======================
 ***************************************************************/
static uint32_t cd_deadline = 0;

void ModelHandle_StartCountdown(uint32_t seconds)
{
    clear_all_modes();
    if (seconds == 0) return;

    countdownActive   = true;
    countdownMode     = true;
    countdownDuration = seconds;
    cd_deadline       = now_ms() + seconds * 1000UL;

    ModelHandle_SaveModeState();
    start_motor();
}

void ModelHandle_StopCountdown(void)
{
    countdownActive = false;
    countdownMode   = false;
    countdownDuration = 0;
    SaveCountdown();                 // clear EEPROM
    stop_motor();
    ModelHandle_SaveModeState();
}

static void countdown_tick(void)
{
    if (!countdownActive) return;

    uint32_t now = now_ms();

    /* Tank full ends countdown */
    if (isTankFull())
    {
        ModelHandle_StopCountdown();
        Buzzer_TriggerAlert();
        return;
    }

    /* Deadline expired */
    if (now >= cd_deadline)
    {
        ModelHandle_StopCountdown();
        return;
    }

    /* Update remaining time */
    countdownDuration = (cd_deadline - now) / 1000UL;

    /* ===== EEPROM POWER CUT SAFE SAVE (every 5 seconds) ===== */
    static uint32_t lastSave = 0;
    if ((now - lastSave) >= 5000UL)
    {
        lastSave = now;
        SaveCountdown();     // stores remaining seconds safely
    }
}

static uint16_t CD_CRC(const uint8_t* d,uint16_t l){
    uint16_t c=0xFFFF;
    while(l--) c=(c>>1)^(*d++ + 0xA001);
    return c;
}
static void SaveCountdown(void)
{
    CountdownBlock b;
    b.sig = CD_SIGNATURE;
    b.remaining = countdownDuration;
    b.active = countdownActive;
    b.crc = CD_CRC((uint8_t*)&b,sizeof(b)-2);

    EEPROM_WriteBuffer(EE_ADDR_COUNTDOWN_BLOCK,(uint8_t*)&b,sizeof(b));
}

void ModelHandle_LoadCountdown(void)
{
    CountdownBlock b;
    EEPROM_ReadBuffer(EE_ADDR_COUNTDOWN_BLOCK,(uint8_t*)&b,sizeof(b));

    if(b.sig!=CD_SIGNATURE) return;
    if(CD_CRC((uint8_t*)&b,sizeof(b)-2)!=b.crc) return;

    if(b.active && b.remaining > 0)
    {
        countdownActive   = true;
        countdownMode     = true;
        countdownDuration = b.remaining;

        cd_deadline = now_ms() + b.remaining * 1000UL;   // <<< FIX
        start_motor();
    }
}

/***************************************************************
 * ========================= TWIST MODE ========================
 ***************************************************************/
static bool     twist_on_phase = false;
static uint32_t twist_deadline = 0;

void ModelHandle_StartTwist(uint16_t on_s, uint16_t off_s,
                            uint8_t onH, uint8_t onM,
                            uint8_t offH, uint8_t offM)
{
    clear_all_modes();

    uint32_t on_sec  = (on_s  ? on_s  : 1) * 60UL;
    uint32_t off_sec = (off_s ? off_s : 1) * 60UL;

    twistSettings.onDurationSeconds  = on_sec;
    twistSettings.offDurationSeconds = off_sec;
    twistSettings.onHour = onH;
    twistSettings.onMinute = onM;
    twistSettings.offHour = offH;
    twistSettings.offMinute = offM;
    twistSettings.twistArmed = true;
    twistSettings.twistActive = false;

    twist_on_phase = true;
    twist_deadline = now_ms() + on_sec * 1000UL;
    start_motor();
}

void ModelHandle_StopTwist(void)
{
    twistActive = false;
    twistSettings.twistActive = false;
    stop_motor();
    ModelHandle_SaveModeState();
}

static void twist_time_logic(void)
{
    if (!twistSettings.twistArmed) return;

    if (!twistActive &&
        time.hour == twistSettings.onHour &&
        time.min  == twistSettings.onMinute)
    {
        twistActive = true;
        twistSettings.twistActive = true;
        twist_on_phase = true;
        twist_deadline = now_ms() +
                         twistSettings.onDurationSeconds * 1000UL;
        start_motor();
    }

    if (twistActive &&
        time.hour == twistSettings.offHour &&
        time.min  == twistSettings.offMinute)
    {
        ModelHandle_StopTwist();
    }
}

static void twist_tick(void)
{
    if (!twistActive) return;

    if (isTankFull())
    {
        ModelHandle_StopTwist();
        Buzzer_TriggerAlert();
        return;
    }

    uint32_t now = now_ms();
    if (now >= twist_deadline)
    {
        twist_on_phase = !twist_on_phase;
        twist_deadline = now +
            (twist_on_phase ? twistSettings.onDurationSeconds
                            : twistSettings.offDurationSeconds) * 1000UL;
    }

    if (twist_on_phase) start_motor();
    else stop_motor();
}
/***************************************************************
 * ======================= PROTECTIONS =========================
 ***************************************************************/
static void protections_tick(void)
{
    if (senseMaxRunReached)
        stop_motor();
}

/***************************************************************
 * =========================== LEDS =============================
 ***************************************************************/
static void leds_from_model(void)
{
    LED_ClearAllIntents();

    bool motorOn = Motor_GetStatus();

    if (motorOn)
        LED_SetIntent(LED_COLOR_GREEN, LED_MODE_STEADY, 0);

    if (!senseDryRun)
    {
        if (motorOn)
            LED_SetIntent(LED_COLOR_GREEN, LED_MODE_BLINK, 350);
        else
            LED_SetIntent(LED_COLOR_RED, LED_MODE_STEADY, 0);
    }

    if (senseMaxRunReached)
        LED_SetIntent(LED_COLOR_RED, LED_MODE_BLINK, 300);

    if (senseOverLoad || senseUnderLoad)
        LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 350);

    if (senseOverUnderVolt)
        LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_BLINK, 350);

    LED_ApplyIntents();
}

/***************************************************************
 * ========================== MASTER FSM ========================
 ***************************************************************/
void ModelHandle_Process(void)
{
    /* ALWAYS first */
    ModelHandle_CheckLoadFault();
    protections_tick();
    twist_time_logic();
    check_max_run();
    ModelHandle_SoftDryRunHandler();

    if (senseMaxRunReached)
    {
        leds_from_model();
        Buzzer_Update();
        return;
    }

    if (manualActive)
    {
        if (senseOverLoad || senseUnderLoad || senseOverUnderVolt)
        {
            stop_motor();
//            manualActive = false;
//            manualOverride = false;
//            ModelHandle_SaveModeState();
            Buzzer_TriggerAlert();
        }
        else if (!Motor_GetStatus())
        {
            start_motor();
        }

        leds_from_model();
        Buzzer_Update();
        return;
    }

    if (autoActive)
    {
        auto_tick();
        leds_from_model();
        Buzzer_Update();
        return;
    }

    if (semiAutoActive)
    {
//        senseDryRun = false;

        if (!isTankFull())
        {
            if (!Motor_GetStatus())
                start_motor();
        }
        else
        {
            stop_motor();
            semiAutoActive = false;
            Buzzer_TriggerAlert();
        }

        leds_from_model();
        Buzzer_Update();
        return;
    }

    if (timerActive)
    {
    	ModelHandle_ProcessTimerSlots();
        leds_from_model();
        Buzzer_Update();
        return;
    }

    if (countdownActive)
    {
        countdown_tick();

        if (!Motor_GetStatus())
            start_motor();

        leds_from_model();
        Buzzer_Update();
        return;
    }

    if (twistActive)
    {
        twist_tick();
        leds_from_model();
        Buzzer_Update();
        return;
    }

    stop_motor();
    leds_from_model();
    Buzzer_Update();
}

/***************************************************************
 * =========================== RESET ============================
 ***************************************************************/
void ModelHandle_ResetAll(void)
{
    clear_all_modes();
    stop_motor();

    senseDryRun        = false;
    senseOverLoad      = false;
    senseUnderLoad     = false;
    senseOverUnderVolt = false;
    senseMaxRunReached = false;

    countdownDuration = 0;

    UART_SendStatusPacket();
}

/***************************************************************
 * ===================== AUTO SETTINGS API ======================
 ***************************************************************/
/***************************************************************
 * ================= USER SETTINGS APPLY ======================
 ***************************************************************/
void ModelHandle_SetUserSettings(uint16_t gap_s,
                                 uint8_t  retry,
                                 uint16_t uv_limit,
                                 uint16_t ov_limit,
                                 int16_t  overload,
                                 int16_t  underload,
                                 uint16_t maxrun_min)
{
    sys.gap_time_s  = gap_s;
    sys.retry_count = retry;
    sys.uv_limit    = uv_limit;
    sys.ov_limit    = ov_limit;

    /* Convert stored int16 -> float if system uses float */
    sys.overload  = (float)overload;
    sys.underload = (float)underload;

    sys.maxrun_min = maxrun_min;

    ModelHandle_SaveSettingsToEEPROM();
}

void ModelHandle_SetAutoSettings(uint16_t gap_s,
                                 uint16_t maxrun_min,
                                 uint8_t retry)
{
    auto_gap_s       = gap_s;
    auto_maxrun_min  = maxrun_min;
    auto_retry_limit = retry;

    EEPROM_WriteBuffer(0x0300, (uint8_t*)&auto_gap_s, sizeof(auto_gap_s));
    EEPROM_WriteBuffer(0x0302, (uint8_t*)&auto_maxrun_min, sizeof(auto_maxrun_min));
    EEPROM_WriteBuffer(0x0304, (uint8_t*)&auto_retry_limit, sizeof(auto_retry_limit));
}

void ModelHandle_LoadAutoSettings(void)
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
 * ===================== RTC MODE PERSIST ======================
 ***************************************************************/
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

    RTC_SavePersistentState(&s);
}

/***************************************************************
 * ================= TIMER SLOT SETTER API =====================
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
    timerSlots[slot].enabled   = 1;

    ModelHandle_SaveTimerToEEPROM();   // <<< CRITICAL
}


/***************************************************************
 * ================= PROTECTION ENABLE APIs ====================
 ***************************************************************/
void ModelHandle_SetDryRun(bool on)
{
    sys.gap_time_s = on ? (sys.gap_time_s ? sys.gap_time_s : 10) : 0;
}

void ModelHandle_SetOverLoad(bool on)
{
    sys.overload = on ? (sys.overload > 0.1f ? sys.overload : 9.0f) : 0.0f;
}

void ModelHandle_SetOverUnderVolt(bool on)
{
    if (!on)
    {
        sys.uv_limit = 0;
        sys.ov_limit = 0;
    }
    else
    {
        if (!sys.uv_limit) sys.uv_limit = 190;
        if (!sys.ov_limit) sys.ov_limit = 270;
    }
}

void ModelHandle_ClearMaxRunFlag(void)
{
    senseMaxRunReached = false;
}
/***************************************************************
 * ====== TIMER SLOT READ API (FOR SCREEN) =====================
 ***************************************************************/
const TimerSlot* ModelHandle_GetTimerSlots(void)
{
    return timerSlots;
}

/***************************************************************
 * ================= UART STUB ================================
 ***************************************************************/
void ModelHandle_ProcessUartCommand(const char* cmd)
{
    (void)cmd;
}
