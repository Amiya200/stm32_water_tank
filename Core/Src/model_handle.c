#include "model_handle.h"
#include "relay.h"
#include "led.h"
#include "global.h"
#include "adc.h"
#include "rtc_i2c.h"
#include "uart_commands.h"
#include "stm32f1xx_hal.h"
#include "eeprom_i2c.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
extern I2C_HandleTypeDef hi2c2;
extern ADC_Data adcData;
extern RTC_Time_t time;
extern float g_currentA;
extern float g_voltageV;
#define TIMER_EE_SIGNATURE  0x544D
#define TIMER_EE_VERSION    1
#define EE_ADDR_TIMER_BLOCK 0x0600

typedef struct {
    uint16_t signature;
    uint8_t  version;
    uint8_t  reserved;
    TimerSlot slots[5];
    uint16_t crc;
} TimerEEPROMBlock;
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
#define MOTOR_START_DELAY_MS   10000UL
#define TANK_FULL_DELAY_MS     10000UL
static uint32_t bootStartBlockUntil = 0;
static uint32_t tankFullDetectedAt  = 0;
static bool     tankFullPendingStop = false;
TimerSlot timerSlots[5];
static uint32_t autoRunStartMs = 0;
volatile bool manualActive    = false;
volatile bool semiAutoActive  = false;
volatile bool countdownActive = false;
volatile bool twistActive     = false;
volatile bool timerActive     = false;
volatile bool autoActive      = false;
#define DRY_PROBE_ON_MS   5000UL
#define DRY_CONFIRM_MS    1500UL
static uint32_t dryDeadline     = 0;
#define LOAD_FAULT_CONFIRM_MS 3000UL
#define LOAD_RETRY_RUN_MS     3000UL
#define LOAD_LOCK_DEFAULT_MS (20UL * 60UL * 1000UL)
#define TIMER_RETRY_30MIN_MS (30UL * 60UL * 1000UL)
static uint32_t buzzerAlertUntil = 0;
static DryFSMState dryState      = DRY_IDLE;
volatile uint8_t motorStatus = 0;
volatile bool senseDryRun         = false;
volatile bool groundWater         = false;
volatile bool senseOverLoad       = false;
volatile bool senseUnderLoad      = false;
volatile bool senseOverUnderVolt  = false;
volatile bool senseMaxRunReached  = false;
volatile bool manualOverride      = false;
volatile uint16_t auto_retry_counter = 0;
volatile bool     countdownMode      = false;
volatile uint32_t countdownDuration  = 0;
TwistSettings twistSettings;
typedef struct {
    bool    manual_on;
    bool    semi_on;
    bool    timer_on;
    bool    countdown_on;
    bool    twist_on;
    bool    auto_on;
    bool    motor_on;
    uint8_t power_restore_mode;
} ModeState;
static ModeState modeState;
#define EE_ADDR_COUNTDOWN_BLOCK 0x0040
#define EE_ADDR_MODE_BLOCK     0x0080
#define EE_ADDR_AUTO_BLOCK     0x00C0
#define CD_SIGNATURE 0xCD55
void ModelHandle_CheckDryRun(void);
void ModelHandle_CheckLoadFault(void);
typedef enum {
    TIMER_STATE_ON,
    TIMER_STATE_OFF,
    TIMER_STATE_WATER_CONTINUOUS
} TimerState;
static TimerState timerState = TIMER_STATE_ON;
static uint32_t   timerStateDeadline = 0;
typedef struct{
    uint16_t sig;
    uint32_t remaining;
    uint8_t  active;
    uint16_t crc;
} CountdownBlock;
static uint8_t powerRestoreMode = 0;
SystemSettings sys = {
    .gap_time_s  = 0,
    .retry_count = 0,
    .uv_limit    = 190,
    .ov_limit    = 270,
    .overload    = 0.0f,
    .underload   = 0.0f,
    .maxrun_min  = 300
};
static inline void start_motor(void);
static inline void stop_motor(void);
static inline void clear_all_modes(void);
void ModelHandle_SoftDryRunHandler(void);
static void SaveCountdown(void);
static bool     timer_any_active_slot(void);
static uint16_t get_active_timer_gap_minutes(void);
static uint8_t  get_today_mask(void);
static bool     slot_is_active_now(const TimerSlot *t);
static bool     isTankFull(void);
static inline bool isAnyModeActive(void);
static void     auto_tick(void);
static void     countdown_tick(void);
static void     twist_tick(void);
static void     leds_from_model(void);
typedef enum {
    MOTOR_OWNER_NONE = 0,
    MOTOR_OWNER_MANUAL,
    MOTOR_OWNER_SEMIAUTO,
    MOTOR_OWNER_TIMER,
    MOTOR_OWNER_COUNTDOWN,
    MOTOR_OWNER_TWIST,
    MOTOR_OWNER_AUTO,
    MOTOR_OWNER_RESTART
} MotorOwner;
static volatile MotorOwner motorOwner = MOTOR_OWNER_NONE;
typedef struct
{
    uint8_t pumpOnSound;
    uint8_t tankFullSound;
    uint8_t tankEmptySound;
} BuzzerSettings;
static BuzzerSettings buzzerSettings = {1,1,1};
static uint32_t cd_deadline = 0;
#define EE_ADDR_BUZZER_BLOCK 0x0500
typedef enum {
    LOAD_NORMAL = 0,
    LOAD_FAULT_WAIT,
    LOAD_FAULT_LOCK,
    LOAD_RETRY_RUN
} LoadFaultState;
static LoadFaultState loadState = LOAD_NORMAL;
static MotorOwner previousOwner = MOTOR_OWNER_NONE;
static bool previousManual    = false;
static bool previousSemi      = false;
static bool previousCountdown = false;
static bool previousAuto      = false;
static uint16_t auto_gap_s = 120;
static uint16_t auto_maxrun_min  = 12;
static uint8_t  auto_retry_limit = 5;
static uint8_t  auto_retry_count = 0;
typedef enum {
    AUTO_IDLE = 0,
    AUTO_ON_WAIT,
    AUTO_DRY_CHECK,
    AUTO_OFF_WAIT
} AutoState;
#define MODE_SWITCH_DELAY_MS  3000UL
static bool     modeSwitchPending = false;
static uint32_t modeSwitchTime    = 0;
static bool restartActive = false;
static MotorOwner pendingOwner    = MOTOR_OWNER_NONE;
static AutoState autoState    = AUTO_IDLE;
static uint32_t autoDeadline = 0;
#define EE_ADDR_AUTO_RUNTIME  0x0340
#define AUTO_SIG 0xA055
static bool     twist_on_phase = false;
static uint32_t bootBlockUntil = 0;
static uint32_t twist_deadline = 0;
typedef struct __attribute__((packed))
{
    uint16_t sig;
    uint8_t  active;
    uint8_t  state;
    uint8_t  retryCount;
    uint32_t remaining_ms;
    uint8_t  crc;
} AutoRuntimeBlock;
static bool suppressAutoOneCycle = false;
#define AUTO_START_LEVEL_PERCENT   50
#define AUTO_STOP_LEVEL_PERCENT    100
static uint32_t loadTimer = 0;
static uint8_t  loadRetryCount = 0;
#ifndef EEPROM_PAGE_SIZE
#define EEPROM_PAGE_SIZE 32
#endif
#ifndef EEPROM_I2C_ADDR
#define EEPROM_I2C_ADDR (0x50 << 1)
#endif
#ifndef EEPROM_ADDR_SIZE
#define EEPROM_ADDR_SIZE I2C_MEMADD_SIZE_16BIT
#endif
void EEPROM_WriteBlockSafe(uint16_t addr, uint8_t *data, uint16_t len)
{
    while (len)
    {
        uint16_t page_space = EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE);
        uint16_t chunk = (len < page_space) ? len : page_space;
        HAL_I2C_Mem_Write(&hi2c2, EEPROM_I2C_ADDR, addr,
                          EEPROM_ADDR_SIZE, data, chunk, 1000);
        HAL_Delay(6);
        addr += chunk;
        data += chunk;
        len  -= chunk;
    }
}

typedef struct __attribute__((packed))
{
    uint16_t sig;
    uint32_t gap;
    uint8_t  retry;
    uint16_t uv;
    uint16_t ov;
    uint16_t maxrun;
    uint16_t over10;
    uint16_t under10;
    uint16_t crc;
} SystemEEPROMBlock;

#define EE_ADDR_SYS_BLOCK  0x0000
#define SYS_SIG 0x5A5A
static uint16_t SYS_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    while (len--)
    {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 1) ?
                  (crc >> 1) ^ 0xA001 :
                  (crc >> 1);
    }

    return crc;
}

static bool autoBackgroundEnabled = true;
void ModelHandle_SaveSettingsToEEPROM(void)
{
    SystemEEPROMBlock b;
    memset(&b, 0, sizeof(b));

    b.sig     = SYS_SIG;
    b.gap     = sys.gap_time_s;
    b.retry   = sys.retry_count;
    b.uv      = sys.uv_limit;
    b.ov      = sys.ov_limit;
    b.maxrun  = sys.maxrun_min;
    b.over10  = (uint16_t)(sys.overload * 10.0f);
    b.under10 = (uint16_t)(sys.underload * 10.0f);

    b.crc = SYS_CRC16((uint8_t*)&b, sizeof(b) - 2);

    EEPROM_WriteBlockSafe(EE_ADDR_SYS_BLOCK,
                          (uint8_t*)&b,
                          sizeof(b));
}

void ModelHandle_LoadSettingsFromEEPROM(void)
{
    SystemEEPROMBlock b;
    EEPROM_ReadBuffer(EE_ADDR_SYS_BLOCK,
                      (uint8_t*)&b,
                      sizeof(b));

    uint16_t crc = SYS_CRC16((uint8_t*)&b, sizeof(b) - 2);

    if (b.sig != SYS_SIG || crc != b.crc)
    {
        // EEPROM invalid → write defaults once
        ModelHandle_SaveSettingsToEEPROM();
        return;
    }

    sys.gap_time_s  = b.gap;
    sys.retry_count = b.retry;
    sys.uv_limit    = b.uv;
    sys.ov_limit    = b.ov;
    sys.maxrun_min  = b.maxrun;
    sys.overload    = b.over10  / 10.0f;
    sys.underload   = b.under10 / 10.0f;
}

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
static void backup_current_mode(void)
{
    previousOwner     = motorOwner;
    previousManual    = manualActive;
    previousSemi      = semiAutoActive;
    previousCountdown = countdownActive;
    previousAuto      = autoActive;
}
static void restore_previous_mode(void)
{
    manualActive    = previousManual;
    semiAutoActive  = previousSemi;
    countdownActive = previousCountdown;
    autoActive      = previousAuto;

    if (previousManual)
        motorOwner = MOTOR_OWNER_MANUAL;
    else if (previousSemi)
        motorOwner = MOTOR_OWNER_SEMIAUTO;
    else if (previousCountdown)
        motorOwner = MOTOR_OWNER_COUNTDOWN;
    else if (previousAuto)
        motorOwner = MOTOR_OWNER_AUTO;
    else
    {
        motorOwner = MOTOR_OWNER_NONE;
        suppressAutoOneCycle = true;
        return;
    }

    /* 🔥 DO NOT START MOTOR IF TANK FULL */
    if (!isTankFull() && isAnyModeActive())
    {
        start_motor();
    }
    else
    {
        stop_motor();
    }
}

static void request_mode_switch(MotorOwner newOwner)
{
    stop_motor();                     // immediately stop
    modeSwitchPending = true;
    modeSwitchTime = HAL_GetTick() + MODE_SWITCH_DELAY_MS;
    pendingOwner = newOwner;
}
void ModelHandle_StopRestart(void)
{
    if (!restartActive)
        return;
    restartActive = false;
    stop_motor();
    restore_previous_mode();
    ModelHandle_SaveModeState();
}


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
#define PROBE_THRESHOLD 0.30f
#define SENSOR_STABLE_TIME_MS 1500UL
static bool is_schedule_allowing_run(void)
{
    if (!timer_any_active_slot())
        return false;

    return true;
}

static uint8_t read_raw_tank_level(void)
{
    uint8_t level = 0;

    if (adcData.voltages[0] < PROBE_THRESHOLD) level = 25;
    if (adcData.voltages[1] < PROBE_THRESHOLD) level = 50;
    if (adcData.voltages[2] < PROBE_THRESHOLD) level = 75;
    if (adcData.voltages[3] < PROBE_THRESHOLD) level = 100;

    return level;
}

static uint8_t get_tank_level_percent(void)
{
    static uint8_t stableLevel = 0;
    static uint8_t candidateLevel = 0;
    static uint32_t changeTime = 0;

    uint32_t now = HAL_GetTick();
    uint8_t rawLevel = read_raw_tank_level();

    if (rawLevel != stableLevel)
    {
        if (rawLevel != candidateLevel)
        {
            candidateLevel = rawLevel;
            changeTime = now;
        }

        if ((now - changeTime) >= SENSOR_STABLE_TIME_MS)
        {
            stableLevel = candidateLevel;
        }
    }

    return stableLevel;
}

static bool isTankFull(void)
{
    return (get_tank_level_percent() == 100);
}

void ModelHandle_StartRestart(void)
{
    if (isTankFull())
        return;

    backup_current_mode();

    restartActive = true;
    motorOwner    = MOTOR_OWNER_RESTART;

    senseMaxRunReached = false;

    /* Calculate remaining percentage */
    uint8_t currentLevel = get_tank_level_percent();
    uint8_t remaining = 100 - currentLevel;

    if (remaining == 0)
        return;

    start_motor();
}

void ModelHandle_TimerRecalculateNow(void)
{
    if (!timerActive) return;
    ModelHandle_ProcessTimerSlots();
}
void ModelHandle_ProcessDryRun(void)
{
    ModelHandle_SoftDryRunHandler();
}
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
void ModelHandle_ToggleManual(void)
{
    /* Cancel restart if running */
    if (restartActive)
    {
        restartActive = false;
        motorOwner = MOTOR_OWNER_NONE;
    }

    if (!manualActive)
    {
        clear_all_modes();
        manualActive = true;
        motorOwner = MOTOR_OWNER_MANUAL;
    }
    else
    {
        manualActive = false;
        motorOwner = MOTOR_OWNER_NONE;
        stop_motor();
    }

    ModelHandle_SaveModeState();
}

void ModelHandle_ManualToggleMotor(void)
{
    if (!manualActive)
        return;

    if (Motor_GetStatus())
        stop_motor();
    else
        start_motor();
}


void ModelHandle_StopAllModesAndMotor(void)
{
    clear_all_modes();
    stop_motor();
    ModelHandle_SaveModeState();
}
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
void ModelHandle_StartTimerNearestSlot(void)
{
    ModelHandle_ProcessTimerSlots();
}

static inline void Buzzer_SetPin(bool on)
{
//    HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin,
//                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void Buzzer_TriggerAlert(void)
{
    buzzerAlertUntil = HAL_GetTick() + 30000UL;  // 30 sec
}

static void Buzzer_Update(void)
{
    uint32_t now = HAL_GetTick();
    static bool buzzerState = false;
    bool newState = false;

    bool motorOn = Motor_GetStatus();

    /* ================= PUMP ON SOUND ================= */
    if (buzzerSettings.pumpOnSound && motorOn)
    {
        newState = true;  // continuous ON
    }

    /* ================= TANK FULL ================= */
    if (buzzerSettings.tankFullSound &&
        isTankFull() &&
        (now < buzzerAlertUntil))
    {
        newState = true;  // continuous long beep
    }

    /* ================= TANK EMPTY ================= */
    if (buzzerSettings.tankEmptySound &&
        get_tank_level_percent() == 0 &&
        (now < buzzerAlertUntil))
    {
        newState = ((now % 400UL) < 100UL);  // short beep pattern
    }

    if (newState != buzzerState)
    {
        buzzerState = newState;
        Buzzer_SetPin(buzzerState);
    }
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


static uint32_t motorOnStartMs = 0;
static uint32_t powerOnMs     = 0;

void ModelHandle_OnPowerUp(void)
{
    powerOnMs = HAL_GetTick();
    bootStartBlockUntil = powerOnMs + MOTOR_START_DELAY_MS;

    autoDeadline = 0;
    if (timer_any_active_slot())
    {
        timerActive = true;
        timerState  = TIMER_STATE_ON;
        timerStateDeadline = 0;
        motorOwner  = MOTOR_OWNER_TIMER;
    }

    /* Restore AUTO if active */
    if (autoActive)
    {
        motorOwner = MOTOR_OWNER_AUTO;
        autoState = AUTO_ON_WAIT;

        uint32_t gapSec = (sys.gap_time_s == 0) ? 120 : sys.gap_time_s;
        autoDeadline = HAL_GetTick() + (gapSec * 1000UL);
    }
}




static inline bool Motor_IsRelayOn(void)
{
    return HAL_GPIO_ReadPin(Relay1_GPIO_Port, Relay1_Pin) == GPIO_PIN_SET;
}
static bool Motor_StartAllowed(void)
{
    return true;   // TEMP: allow immediate start
}

bool ModelHandle_IsRestartActive(void)
{
    return restartActive;
}

static inline void motor_apply(bool on)
{
    if (timerActive && timerState == TIMER_STATE_OFF)
        on = false;

    bool relayNow = Motor_IsRelayOn();

    if (on)
    {
        if (!Motor_StartAllowed())
            return;

        if (!relayNow)
        {
            Relay_Set(1, true);
            motorOnStartMs = HAL_GetTick();
        }
        motorStatus = 1;
    }
    else
    {
        if (relayNow)
            Relay_Set(1, false);
        motorStatus = 0;
    }

    UART_SendStatusPacket();
}

bool Motor_GetStatus(void)
{
    return Motor_IsRelayOn();
}
static inline void start_motor(void)
{
    uint32_t now = HAL_GetTick();

    if (now < bootStartBlockUntil)
        return;

    if (motorOwner == MOTOR_OWNER_NONE)
        return;

    motor_apply(true);
}


static inline void stop_motor(void)
{
    motor_apply(false);
}
static void check_max_run(void)
{
    if (sys.maxrun_min == 0 || !Motor_GetStatus())
        return;

    if (countdownActive)
        return;
    if (restartActive)
        ;  // allow maxrun protection to work

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
void ModelHandle_CheckDryRun(void)
{
    if (manualActive || semiAutoActive || countdownActive)
    {
        senseDryRun = false;
        return;
    }
    float v = adcData.voltages[5];
    if (v < 0.01f)
        senseDryRun = true;
    else
        senseDryRun = false;
}
void ModelHandle_CheckGroundWater(void)
{
    float v = adcData.voltages[4];
    if (v < 0.01f)
        groundWater = true;
    else
        groundWater = false;
}
static inline bool isAnyModeActive(void)
{
    return (manualActive || semiAutoActive || countdownActive ||
            twistActive || timerActive || autoActive);
}
void ModelHandle_SoftDryRunHandler(void)
{
    if (countdownActive)
    {
        dryState = DRY_IDLE;
        return;
    }
    if (sys.gap_time_s == 0)
    {
        dryState = DRY_IDLE;
        return;
    }

    if (motorOwner != MOTOR_OWNER_TIMER)
    {
        dryState = DRY_IDLE;
        return;
    }


    uint32_t now   = HAL_GetTick();
    uint32_t gapMs = (uint32_t)sys.gap_time_s * 1000UL;

    ModelHandle_CheckDryRun();
    bool motorOn = Motor_GetStatus();

    /* ================= MOTOR RUNNING ================= */
    if (motorOn)
    {
        /* ✅ WATER PRESENT */
        if (senseDryRun == true)
        {
            dryState = DRY_IDLE;
            return;
        }

        /* ❌ DRY CONDITION → Start waiting */
        if (dryState != DRY_WAITING)
        {
            dryState = DRY_WAITING;
            dryDeadline = now + gapMs;
        }

        /* Timeout reached → Stop motor */
        if ((int32_t)(now - dryDeadline) >= 0)
        {
            stop_motor();
            dryState = DRY_FAULT;
            Buzzer_TriggerAlert();
        }
    }
    else
    {
        /* Motor OFF → Reset if water comes back */
        if (dryState == DRY_FAULT && senseDryRun == true)
        {
            dryState = DRY_IDLE;
        }
    }
}

static inline uint32_t get_load_lock_duration_ms(void)
{
    if (sys.retry_count == 0)
        return LOAD_LOCK_DEFAULT_MS;
    return (uint32_t)sys.retry_count * 60000UL;
}

void ModelHandle_CheckLoadFault(void)
{
	if (!Motor_GetStatus())
        return;

    float I = g_currentA;
    float V = g_voltageV;

    bool overload  = (sys.overload  > 0.1f)   && (I > sys.overload);
    bool underload = (sys.underload > 0.001f) && (I < sys.underload);
    bool voltFault = ((sys.uv_limit && V < sys.uv_limit) ||
                      (sys.ov_limit && V > sys.ov_limit));

    senseOverLoad      = overload;
    senseUnderLoad     = underload;
    senseOverUnderVolt = voltFault;

    bool fault = overload || underload || voltFault;
    uint32_t now = HAL_GetTick();

    uint32_t lockMs = get_load_lock_duration_ms();

    switch (loadState)
    {
        case LOAD_NORMAL:
            loadRetryCount = 0;
            if (fault)
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
            if (!fault)
            {
                loadState = LOAD_NORMAL;
            }
            else if ((now - loadTimer) >= lockMs && loadRetryCount < 1)
            {
                loadRetryCount++;

                if (isAnyModeActive())
                {
                    start_motor();
                }

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

static uint8_t get_today_mask(void)
{
    uint8_t d = time.dow;
    if (d < 1 || d > 7) d = 1;
    return (1U << (d - 1));
}
static bool slot_is_active_now(const TimerSlot *t)
{
    if (!t->enabled)
        return false;

    if (!(t->dayMask & get_today_mask()))
        return false;

    uint16_t now = time.hour * 60 + time.min;
    uint16_t on  = t->onHour  * 60 + t->onMinute;
    uint16_t off = t->offHour * 60 + t->offMinute;

    /* If ON == OFF → treat as disabled */
    if (on == off)
        return false;

    /* Normal same-day slot */
    if (on < off)
        return (now >= on && now < off);

    /* Overnight slot */
    return (now >= on || now < off);
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

void ModelHandle_SaveBuzzerSettings(void)
{
    EEPROM_WriteBuffer(EE_ADDR_BUZZER_BLOCK,
                       (uint8_t*)&buzzerSettings,
                       sizeof(buzzerSettings));
}
void ModelHandle_LoadBuzzerSettings(void)
{
    EEPROM_ReadBuffer(EE_ADDR_BUZZER_BLOCK,
                      (uint8_t*)&buzzerSettings,
                      sizeof(buzzerSettings));
}

void ModelHandle_ProcessTimerSlots(void)
{
    uint32_t now = HAL_GetTick();
    bool slotActive = timer_any_active_slot();

    /* ------------------ SLOT START ------------------ */
    if (!timerActive && slotActive)
    {
        backup_current_mode();
        clear_all_modes();
        senseMaxRunReached = false;

        timerActive = true;
        autoActive  = false;

        motorOwner  = MOTOR_OWNER_TIMER;
        timerState  = TIMER_STATE_ON;
        timerStateDeadline = 0;

        start_motor();
    }

    /* ------------------ SLOT END ------------------ */
    if (timerActive && !slotActive)
    {
        stop_motor();

        timerActive = false;
        timerState  = TIMER_STATE_ON;
        timerStateDeadline = 0;

        restore_previous_mode();
        ModelHandle_SaveModeState();
        return;
    }

    if (!timerActive)
        return;

    /* ------------------ SAFETY CHECKS ------------------ */
    if (isTankFull() ||
        senseOverLoad ||
        senseUnderLoad ||
        senseOverUnderVolt ||
        senseMaxRunReached)
    {
        stop_motor();
        return;
    }

    /* ------------------ GAP CALCULATION ------------------ */
    uint16_t slotGap = get_active_timer_gap_minutes();
    uint16_t mainGap = sys.gap_time_s / 60;

    uint16_t appliedGapMin = (slotGap > 0) ? slotGap : mainGap;
    uint32_t gapMs = appliedGapMin * 60UL * 1000UL;

    /* ------------------ CONTINUOUS MODE ------------------ */
    if (gapMs == 0)
    {
        motorOwner = MOTOR_OWNER_TIMER;
        start_motor();
        return;
    }

    /* ------------------ TIMER STATE MACHINE ------------------ */
    switch (timerState)
    {
        case TIMER_STATE_ON:
        {
            motorOwner = MOTOR_OWNER_TIMER;
            start_motor();

            if (timerStateDeadline == 0)
            {
                timerStateDeadline = now + gapMs;
            }

            if (now >= timerStateDeadline)
            {
                /* 🔥 Dry Run Condition Handling */
                if (!senseDryRun)
                {
                    // Normal behavior → go OFF
                    stop_motor();
                    timerState = TIMER_STATE_OFF;
                    timerStateDeadline = now + gapMs;
                }
                else
                {
                    // Dry Run active → Stay ON continuously
                    timerStateDeadline = now + gapMs;
                }
            }
        }
        break;

        case TIMER_STATE_OFF:
        {
            if (now >= timerStateDeadline)
            {
                timerState = TIMER_STATE_ON;
                timerStateDeadline = 0;
            }
        }
        break;

        default:
        {
            timerState = TIMER_STATE_ON;
            timerStateDeadline = 0;
        }
        break;
    }
}

void ModelHandle_StartTimer(void)
{
    ModelHandle_SaveTimerToEEPROM();
}

void ModelHandle_StopTimer(void)
{
    timerActive = false;
    timerStateDeadline = 0;   // ADD THIS
    stop_motor();
    ModelHandle_SaveModeState();
}

void ModelHandle_CheckAutoTimerActivation(void)
{
    if (timerActive) return;

    if (timer_any_active_slot())
    {
        timerActive = true;
        start_motor();
    }
}
void ModelHandle_StartSemiAuto(void)
{
    restartActive = false;
    clear_all_modes();
    timerActive = false;
    autoActive  = false;
    semiAutoActive = true;
    motorOwner     = MOTOR_OWNER_SEMIAUTO;
    start_motor();
    ModelHandle_SaveModeState();
}

void ModelHandle_StopSemiAuto(void)
{
    semiAutoActive = false;
    stop_motor();
    ModelHandle_SaveModeState();
}
void ModelHandle_StartAuto(uint16_t gap_s,
                           uint16_t maxrun_min,
                           uint8_t retry)
{
    if (autoActive)
        return;

    clear_all_modes();

    auto_maxrun_min  = maxrun_min;
    auto_retry_limit = retry;
    auto_retry_count = 0;

    autoActive = true;
    autoState  = AUTO_ON_WAIT;

    uint32_t now = HAL_GetTick();
    uint32_t gapSec = (sys.gap_time_s == 0) ? 120 : sys.gap_time_s;

    autoRunStartMs = now;
    autoDeadline   = now + (gapSec * 1000UL);

    motorOwner = MOTOR_OWNER_AUTO;   // 🔥 DIRECT ASSIGN
}

void ModelHandle_StopAuto(void)
{
    stop_motor();

    autoActive = false;
    autoState  = AUTO_IDLE;
    auto_retry_count = 0;
    autoDeadline = 0;

    ModelHandle_SaveModeState();
}

void ModelHandle_LoadAutoSettings(void)
{
    EEPROM_ReadBuffer(0x0300, (uint8_t*)&auto_gap_s, sizeof(auto_gap_s));
    EEPROM_ReadBuffer(0x0302, (uint8_t*)&auto_maxrun_min, sizeof(auto_maxrun_min));
    EEPROM_ReadBuffer(0x0304, (uint8_t*)&auto_retry_limit, sizeof(auto_retry_limit));

    /* HARD VALIDATION */

    if (auto_gap_s < 30 || auto_gap_s > 3600)
    {
        auto_gap_s = 120;   // force 2 min
        EEPROM_WriteBlockSafe(0x0300,
                              (uint8_t*)&auto_gap_s,
                              sizeof(auto_gap_s));
    }

    if (auto_maxrun_min == 0xFFFF || auto_maxrun_min > 600)
    {
        auto_maxrun_min = 30;
        EEPROM_WriteBlockSafe(0x0302,
                              (uint8_t*)&auto_maxrun_min,
                              sizeof(auto_maxrun_min));
    }

    if (auto_retry_limit == 0xFF || auto_retry_limit > 20)
    {
        auto_retry_limit = 3;
        EEPROM_WriteBlockSafe(0x0304,
                              (uint8_t*)&auto_retry_limit,
                              sizeof(auto_retry_limit));
    }
}


static void auto_mode_background_control(void)
{
    if (!autoBackgroundEnabled)
        return;
    if (timerActive)
        return;
    uint32_t now = HAL_GetTick();
    if (now < bootBlockUntil)
        return;
    if (manualActive || semiAutoActive ||
        countdownActive || twistActive ||
        autoActive)
        return;
    uint8_t level = get_tank_level_percent();
    if (!autoActive &&
        level <= AUTO_START_LEVEL_PERCENT)
    {
    	ModelHandle_StartAuto(
    	    sys.gap_time_s,
    	    auto_maxrun_min,
    	    auto_retry_limit
    	);

    }
    if (autoActive &&
        level >= AUTO_STOP_LEVEL_PERCENT)
    {
        ModelHandle_StopAuto();
    }
}

static void auto_tick(void)
{
    if (!autoActive)
        return;

    uint32_t now = HAL_GetTick();

    /* Use master gap from sys */
    uint16_t slotGapMin = get_active_timer_gap_minutes();
    uint32_t gapSec;

    if (slotGapMin > 0)
        gapSec = slotGapMin * 60UL;
    else
        gapSec = (sys.gap_time_s == 0) ? 120 : sys.gap_time_s;

    uint32_t gapMs  = gapSec * 1000UL;

    ModelHandle_CheckGroundWater();
    ModelHandle_CheckDryRun();

    bool protectionFault =
        senseOverLoad ||
        senseUnderLoad ||
        senseOverUnderVolt ||
        senseMaxRunReached;

    /* ===== HARD STOP CONDITIONS ===== */
    if (isTankFull() || protectionFault)
    {
        ModelHandle_StopAuto();
        Buzzer_TriggerAlert();
        return;
    }

    switch (autoState)
    {
    /* ================= ON PHASE ================= */
    case AUTO_ON_WAIT:
    {
        /* No groundwater → go OFF phase */
        if (!groundWater)
        {
            stop_motor();
            autoDeadline = now + gapMs;
            autoState = AUTO_OFF_WAIT;
            return;
        }

        /* Ensure motor is running */
        motorOwner = MOTOR_OWNER_AUTO;
        start_motor();

        /* Dry condition detected */
        if (!senseDryRun)
        {
            stop_motor();

            auto_retry_count++;

            /* Retry limit reached */
            if (auto_retry_count >= auto_retry_limit)
            {
                ModelHandle_StopAuto();
                Buzzer_TriggerAlert();
                return;
            }

            autoDeadline = now + gapMs;
            autoState = AUTO_OFF_WAIT;
            return;
        }

        /* Deadline reached */
        if (now >= autoDeadline)
        {
            if (senseDryRun)
            {
                /* Water present → continue running */
                autoDeadline = now + gapMs;
                /* stay in AUTO_ON_WAIT */
            }
            else
            {
                /* Dry exactly at deadline */
                stop_motor();

                auto_retry_count++;

                if (auto_retry_count >= auto_retry_limit)
                {
                    ModelHandle_StopAuto();
                    Buzzer_TriggerAlert();
                    return;
                }

                autoDeadline = now + gapMs;
                autoState = AUTO_OFF_WAIT;
            }
        }
    }
    break;

    /* ================= OFF PHASE ================= */
    case AUTO_OFF_WAIT:
    {
        stop_motor();

        if (now >= autoDeadline)
        {
            autoState = AUTO_ON_WAIT;
            autoDeadline = now + gapMs;
        }
    }
    break;

    default:
        autoState = AUTO_IDLE;
    break;
    }
}

void ModelHandle_StartCountdown(uint32_t seconds)
{
    if (seconds == 0)
        return;
    clear_all_modes();
    senseMaxRunReached = false;
    countdownActive   = true;
    countdownMode     = true;
    countdownDuration = seconds;
    cd_deadline       = now_ms() + seconds * 1000UL;
    motorOwner = MOTOR_OWNER_COUNTDOWN;
    start_motor();
    ModelHandle_SaveModeState();
}

void ModelHandle_StopCountdown(void)
{
    countdownActive = false;
    countdownMode   = false;
    countdownDuration = 0;
    SaveCountdown();
    stop_motor();
    ModelHandle_SaveModeState();
}

static void countdown_tick(void)
{
    if (!countdownActive)
        return;

    uint32_t now = now_ms();

    bool dryEnabled      = (sys.gap_time_s  > 0);
    bool overloadEnabled = (sys.overload    > 0.0f);
    bool underloadEnabled= (sys.underload   > 0.0f);
    bool voltEnabled     = (sys.uv_limit    > 0 ||
                            sys.ov_limit    > 0);
    bool maxrunEnabled   = (sys.maxrun_min  > 0);

    bool stopRequired = false;

    /* Tank Full always stops */
    if (isTankFull())
        stopRequired = true;

    /* Dry run only if enabled */
    if (dryEnabled && senseDryRun)
        stopRequired = true;

    /* Load protections only if enabled */
    if (overloadEnabled && senseOverLoad)
        stopRequired = true;

    if (underloadEnabled && senseUnderLoad)
        stopRequired = true;

    if (voltEnabled && senseOverUnderVolt)
        stopRequired = true;

    if (maxrunEnabled && senseMaxRunReached)
        stopRequired = true;

    if (stopRequired)
    {
        ModelHandle_StopCountdown();
        Buzzer_TriggerAlert();
        return;
    }

    /* Normal countdown expiration */
    if (now >= cd_deadline)
    {
        ModelHandle_StopCountdown();
        return;
    }

    uint32_t remainingMs = cd_deadline - now;
    countdownDuration = remainingMs / 1000UL;

    start_motor();  // ensure motor keeps running
}

static uint16_t CD_CRC(const uint8_t* d,uint16_t l){
    uint16_t c=0xFFFF;
    while(l--) c=(c>>1)^(*d++ + 0xA001);
    return c;
}

static void SaveCountdown(void)
{
    CountdownBlock b;
    memset(&b, 0, sizeof(b));
    b.sig = CD_SIGNATURE;
    b.active = countdownActive;
    if (countdownActive)
    {
        uint32_t now = now_ms();
        if (cd_deadline > now)
            b.remaining = (cd_deadline - now) / 1000UL;
        else
            b.remaining = 0;
    }
    else
    {
        b.remaining = 0;
    }
    b.crc = CD_CRC((uint8_t*)&b, sizeof(b)-2);
    EEPROM_WriteBuffer(EE_ADDR_COUNTDOWN_BLOCK,
                       (uint8_t*)&b,
                       sizeof(b));
}

void ModelHandle_LoadCountdown(void)
{
    CountdownBlock b;
    EEPROM_ReadBuffer(EE_ADDR_COUNTDOWN_BLOCK,
                      (uint8_t*)&b, sizeof(b));
    if (b.sig != CD_SIGNATURE)
        return;
    if (CD_CRC((uint8_t*)&b, sizeof(b)-2) != b.crc)
        return;
    if (b.active && b.remaining > 0)
    {
        countdownActive   = true;
        countdownMode     = true;
        countdownDuration = b.remaining;
        cd_deadline = now_ms() +
                      b.remaining * 1000UL;
        motorOwner = MOTOR_OWNER_COUNTDOWN;
    }
}

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
static void leds_from_model(void)
{
    LED_ClearAllIntents();
    bool motorOn = Motor_GetStatus();
    if (dryState == DRY_WAITING)
    {
        LED_SetIntent(LED_COLOR_RED, LED_MODE_BLINK, 400);
        LED_ApplyIntents();
        return;
    }
    if (dryState == DRY_FAULT)
    {
        LED_SetIntent(LED_COLOR_RED, LED_MODE_STEADY, 0);
        LED_ApplyIntents();
        return;
    }
    if (motorOn)
        LED_SetIntent(LED_COLOR_GREEN, LED_MODE_STEADY, 0);
    if (senseMaxRunReached)
        LED_SetIntent(LED_COLOR_RED, LED_MODE_BLINK, 300);
    if (senseOverLoad || senseUnderLoad)
        LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 350);
    if (senseOverUnderVolt)
        LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_BLINK, 350);
    LED_ApplyIntents();
}

void ModelHandle_Process(void)
{
    uint32_t now = HAL_GetTick();
    if (modeSwitchPending)
    {
        if (now >= modeSwitchTime)
        {
            modeSwitchPending = false;
            motorOwner = pendingOwner;
        }
        return;
    }
    ModelHandle_CheckGroundWater();
    ModelHandle_CheckDryRun();
    ModelHandle_CheckLoadFault();
    check_max_run();
    ModelHandle_SoftDryRunHandler();
    bool protectionFault =
        senseOverLoad ||
        senseUnderLoad ||
        senseOverUnderVolt ||
        senseMaxRunReached;
    ModelHandle_ProcessTimerSlots();
    if (restartActive)
        motorOwner = MOTOR_OWNER_RESTART;
    else if (timerActive)
        motorOwner = MOTOR_OWNER_TIMER;
    else if (manualActive)
        motorOwner = MOTOR_OWNER_MANUAL;
    else if (semiAutoActive)
        motorOwner = MOTOR_OWNER_SEMIAUTO;
    else if (countdownActive)
        motorOwner = MOTOR_OWNER_COUNTDOWN;
    else if (twistActive)
        motorOwner = MOTOR_OWNER_TWIST;
    else if (autoActive)
        motorOwner = MOTOR_OWNER_AUTO;
    else
        motorOwner = MOTOR_OWNER_NONE;
    if (Motor_GetStatus() && isTankFull())
    {
        if (!tankFullPendingStop)
        {
            tankFullPendingStop = true;
            tankFullDetectedAt = now;
        }
        if ((now - tankFullDetectedAt) >= TANK_FULL_DELAY_MS)
        {
            stop_motor();
            tankFullPendingStop = false;
        }
    }
    else
    {
        tankFullPendingStop = false;
    }
    switch (motorOwner)
    {
    case MOTOR_OWNER_RESTART:
    {
        bool dryEnabled      = (sys.gap_time_s  > 0);
        bool overloadEnabled = (sys.overload    > 0.0f);
        bool underloadEnabled= (sys.underload   > 0.0f);
        bool voltEnabled     = (sys.uv_limit    > 0 ||
                                sys.ov_limit    > 0);
        bool maxrunEnabled   = (sys.maxrun_min  > 0);

        bool restartFault = false;

        /* 100% tank always stops */
        if (isTankFull())
            restartFault = true;

        /* Dry run only if enabled */
        if (dryEnabled && senseDryRun)
            restartFault = true;

        /* Load protections only if enabled */
        if (overloadEnabled && senseOverLoad)
            restartFault = true;

        if (underloadEnabled && senseUnderLoad)
            restartFault = true;

        if (voltEnabled && senseOverUnderVolt)
            restartFault = true;

        if (maxrunEnabled && senseMaxRunReached)
            restartFault = true;

        if (restartFault)
        {
            ModelHandle_StopRestart();
            Buzzer_TriggerAlert();
        }
        else
        {
            start_motor();
        }
    }
    break;


        case MOTOR_OWNER_MANUAL:
        {
            if (!protectionFault)
                start_motor();
            else
                stop_motor();
        }
        break;
        case MOTOR_OWNER_SEMIAUTO:
        {
            if (isTankFull())
            {
                stop_motor();

                semiAutoActive = false;
                motorOwner     = MOTOR_OWNER_NONE;

                ModelHandle_SaveModeState();
                Buzzer_TriggerAlert();

                break;
            }
            if (protectionFault || senseDryRun)
            {
                stop_motor();
            }
            else
            {
                start_motor();
            }
        }
        break;

        case MOTOR_OWNER_TIMER:
        {
            if (protectionFault || isTankFull())
                stop_motor();
        }
        break;
        case MOTOR_OWNER_COUNTDOWN:
        {
            countdown_tick();
        }
        break;
        case MOTOR_OWNER_TWIST:
        {
            twist_tick();
        }
        break;
        case MOTOR_OWNER_AUTO:
        {
            auto_tick();
        }
        break;
        default:
        {
            stop_motor();
        }
        break;
    }
    if (motorOwner == MOTOR_OWNER_NONE ||
        motorOwner == MOTOR_OWNER_AUTO)
    {
        auto_mode_background_control();

        /* Re-evaluate auto after schedule re-opens */
        if (autoActive && is_schedule_allowing_run())
        {
            autoState = AUTO_ON_WAIT;
        }
    }

    leds_from_model();
    Buzzer_Update();
}

DryFSMState ModelHandle_GetDryState(void)
{
    return dryState;
}

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
void ModelHandle_SetUserSettings(uint32_t gap_seconds,
                                 uint8_t  retry,
                                 uint16_t uv_limit,
                                 uint16_t ov_limit,
                                 int16_t  overload,
                                 int16_t  underload,
                                 uint16_t maxrun_min)
{
    if (gap_seconds > 86400UL)
        gap_seconds = 86400UL;

    sys.gap_time_s  = gap_seconds;
    sys.retry_count = retry;
    sys.uv_limit    = uv_limit;
    sys.ov_limit    = ov_limit;
    sys.overload    = (float)overload;
    sys.underload   = (float)underload;
    sys.maxrun_min  = maxrun_min;

    ModelHandle_SaveSettingsToEEPROM();

    /* Live update AUTO deadline */
    if (autoActive)
    {
        uint32_t now = HAL_GetTick();
        uint32_t gapSec = (sys.gap_time_s == 0) ? 120 : sys.gap_time_s;
        autoDeadline = now + (gapSec * 1000UL);
    }
}

void ModelHandle_SetAutoSettings(uint16_t gap_s,
                                 uint16_t maxrun_min,
                                 uint8_t retry)
{
    auto_gap_s       = (gap_s < 30) ? 120 : gap_s;
    auto_maxrun_min  = maxrun_min;
    auto_retry_limit = retry;

    EEPROM_WriteBlockSafe(0x0300, (uint8_t*)&auto_gap_s, sizeof(auto_gap_s));
    EEPROM_WriteBlockSafe(0x0302, (uint8_t*)&auto_maxrun_min, sizeof(auto_maxrun_min));
    EEPROM_WriteBlockSafe(0x0304, (uint8_t*)&auto_retry_limit, sizeof(auto_retry_limit));
}


bool ModelHandle_IsAutoActive(void)
{
    return autoActive;
}
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
void ModelHandle_SetTimerSlot(uint8_t slot,
                              uint8_t onH, uint8_t onM,
                              uint8_t offH, uint8_t offM)
{
    timerSlots[slot].onHour    = onH;
    timerSlots[slot].onMinute  = onM;
    timerSlots[slot].offHour   = offH;
    timerSlots[slot].offMinute = offM;
    timerSlots[slot].enabled   = 1;
}

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
const TimerSlot* ModelHandle_GetTimerSlots(void)
{
    return timerSlots;
}
void ModelHandle_ProcessUartCommand(const char* cmd)
{
    (void)cmd;
}
bool ModelHandle_IsOverload(void)
{
    return senseOverLoad;
}
bool ModelHandle_IsUnderload(void)
{
    return senseUnderLoad;
}
bool ModelHandle_IsDryRunActive(void)
{
    return senseDryRun;
}
bool ModelHandle_IsVoltageFault(void)
{
    return senseOverUnderVolt;
}
bool ModelHandle_IsMaxRunReached(void)
{
    return senseMaxRunReached;
}

uint8_t ModelHandle_GetTankLevelPercent(void)
{
    return get_tank_level_percent();
}

bool ModelHandle_IsTankFull(void)
{
    return isTankFull();
}
bool ModelHandle_IsManualActive(void)
{
    return manualActive;
}
uint16_t ModelHandle_GetAutoGap(void)
{
    return auto_gap_s;
}

uint16_t ModelHandle_GetAutoMaxRun(void)
{
    return auto_maxrun_min;
}

uint8_t ModelHandle_GetAutoRetry(void)
{
    return auto_retry_limit;
}

