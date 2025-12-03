/***************************************************************
 *  HELONIX Water Pump Controller
 *  SCREEN.C — FINAL VERSION WITH SETTINGS (OPTION-B)
 *  PART A/6
 ***************************************************************/

#include "screen.h"
#include "lcd_i2c.h"
#include "switches.h"
#include "model_handle.h"
#include "adc.h"
#include "rtc_i2c.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

/* ================================================================
   UI ENUMS
   ================================================================ */

typedef enum {
    UI_WELCOME = 0,
    UI_DASH,
    UI_MENU,

    UI_MANUAL,
    UI_SEMI_AUTO,

    UI_TIMER,
    UI_TIMER_SLOT_SELECT,
    UI_TIMER_EDIT_SLOT_ON_H,
    UI_TIMER_EDIT_SLOT_ON_M,
    UI_TIMER_EDIT_SLOT_OFF_H,
    UI_TIMER_EDIT_SLOT_OFF_M,
    UI_TIMER_EDIT_SLOT_DAYS,
    UI_TIMER_EDIT_SLOT_ENABLE,

    UI_AUTO_MENU,
    UI_AUTO_EDIT_GAP,
    UI_AUTO_EDIT_MAXRUN,
    UI_AUTO_EDIT_RETRY,

    UI_COUNTDOWN,
    UI_COUNTDOWN_EDIT_MIN,

    UI_TWIST,
    UI_TWIST_EDIT_ON,
    UI_TWIST_EDIT_OFF,
    UI_TWIST_EDIT_ON_H,
    UI_TWIST_EDIT_ON_M,
    UI_TWIST_EDIT_OFF_H,
    UI_TWIST_EDIT_OFF_M,
	UI_NONE,
    /* NEW: SETTINGS LINEAR FLOW (Option-B) */
    UI_SETTINGS_GAP,
    UI_SETTINGS_RETRY,
    UI_SETTINGS_UV,
    UI_SETTINGS_OV,
    UI_SETTINGS_OL,
    UI_SETTINGS_UL,
    UI_SETTINGS_MAXRUN,
    UI_SETTINGS_FACTORY,

    UI_MAX_
} UiState;

typedef enum {
    BTN_NONE = 0,
    BTN_RESET,
    BTN_SELECT,
    BTN_UP,
    BTN_DOWN,

    BTN_RESET_LONG,
    BTN_SELECT_LONG,
    BTN_UP_LONG,
    BTN_DOWN_LONG
} UiButton;

/* ================================================================
   GLOBAL UI VARIABLES
   ================================================================ */

static UiState ui = UI_WELCOME;
static UiState last_ui = UI_MAX_;

static bool screenNeedsRefresh = false;
static bool cursorVisible = true;

static uint32_t lastCursorToggle = 0;
static uint32_t lastLcdUpdateTime = 0;
static uint32_t lastUserAction = 0;

static const uint32_t WELCOME_MS      = 2500;
static const uint32_t CURSOR_BLINK_MS = 400;
static const uint32_t AUTO_BACK_MS    = 60000;

#define LONG_PRESS_MS         3000
#define CONTINUOUS_STEP_MS     300
#define COUNTDOWN_INC_MS      3000

/* Button timers */
static uint32_t sw_press_start[4] = {0,0,0,0};
static bool     sw_long_issued[4] = {false,false,false,false};

static uint32_t last_repeat_time = 0;

/* ================================================================
   EXTERNAL STATES
   ================================================================ */

extern ADC_Data adcData;
extern TimerSlot timerSlots[5];
extern TwistSettings twistSettings;

extern volatile bool manualActive;
extern volatile bool semiAutoActive;
extern volatile bool timerActive;
extern volatile bool countdownActive;
extern volatile bool twistActive;
extern volatile bool autoActive;
extern volatile uint32_t countdownDuration;

/* ================================================================
   TEMP EDIT VARIABLES
   ================================================================ */

static uint8_t  edit_timer_on_h  = 0,  edit_timer_on_m  = 0;
static uint8_t  edit_timer_off_h = 0, edit_timer_off_m = 0;
static uint8_t  edit_timer_dayMask   = 0x7F;
static uint8_t  edit_timer_dayIndex  = 0;
static bool     edit_timer_enabled   = true;

static uint16_t edit_auto_gap_s      = 60;
static uint16_t edit_auto_maxrun_min = 120;
static uint16_t edit_auto_retry      = 0;

static uint16_t edit_twist_on_s  = 5;
static uint16_t edit_twist_off_s = 5;

static uint8_t edit_twist_on_hh  = 6;
static uint8_t edit_twist_on_mm  = 0;
static uint8_t edit_twist_off_hh = 18;
static uint8_t edit_twist_off_mm = 0;

static uint16_t edit_countdown_min = 1;

static uint8_t currentSlot = 0;

/* ================================================================
   NEW SETTINGS TEMP VARIABLES (Option-B)
   ================================================================ */

static uint16_t edit_settings_gap_s = 10;
static uint8_t  edit_settings_retry = 3;

static uint16_t edit_settings_uv = 180;
static uint16_t edit_settings_ov = 260;

static float edit_settings_ol = 6.5f;
static float edit_settings_ul = 0.5f;

static uint16_t edit_settings_maxrun = 120;
static bool     edit_settings_factory_yes = false;

/* ================================================================
   MENU ITEMS
   ================================================================ */

static const char* const main_menu[] = {
    "Timer Mode",
    "Auto Mode",
    "Twist Mode",
    "Settings",
    "Back"
};

#define MAIN_MENU_COUNT (sizeof(main_menu)/sizeof(main_menu[0]))

static int menu_idx = 0;
static int menu_view_top = 0;

/* ================================================================
   LCD BASIC HELPERS
   ================================================================ */

static inline void refreshInactivityTimer(void) {
    lastUserAction = HAL_GetTick();
}

static inline void lcd_line(uint8_t row, const char* s) {
    char buf[17];
    snprintf(buf, sizeof(buf), "%-16.16s", s);
    lcd_put_cur(row, 0);
    lcd_send_string(buf);
}

static inline void lcd_line0(const char* s){ lcd_line(0, s); }
static inline void lcd_line1(const char* s){ lcd_line(1, s); }

/* ================================================================
   MENU CURSOR DRAW FUNCTION (SMOOTH BLINK)
   ================================================================ */

static void draw_menu_cursor(void)
{
    if (ui != UI_MENU) return;

    uint8_t row = 255;

    if (menu_idx == menu_view_top)
        row = 0;
    else if (menu_idx == menu_view_top + 1)
        row = 1;

    if (row > 1) return;

    lcd_put_cur(row, 0);
    lcd_send_data(cursorVisible ? '>' : ' ');
}

/* ================================================================
   WELCOME SCREEN
   ================================================================ */

static void show_welcome(void)
{
    lcd_clear();
    lcd_line0("   HELONIX");
    lcd_line1(" IntelligentSys");
}

/* ================================================================
   DASHBOARD DISPLAY
   ================================================================ */

static void show_dash(void)
{
    char l0[17], l1[17];

    const char* motor = Motor_GetStatus() ? "ON " : "OFF";

    const char* mode = "IDLE";
    if (manualActive)          mode = "MANUAL";
    else if (semiAutoActive)   mode = "SEMI";
    else if (timerActive)      mode = "TIMER";
    else if (countdownActive)  mode = "CD";
    else if (twistActive)      mode = "TWIST";
    else if (autoActive)       mode = "AUTO";

    snprintf(l0, sizeof(l0), "M:%s %s", motor, mode);

    int submerged = 0;
    for (int i = 1; i <= 5; i++){
        if (adcData.voltages[i] < 0.1f)
            submerged++;
    }

    const char* level =
        (submerged>=5)?"100%":
        (submerged==4)?"80%":
        (submerged==3)?"60%":
        (submerged==2)?"40%":
        (submerged==1)?"20%":"0%";

    snprintf(l1, sizeof(l1), "Water:%s", level);

    lcd_line0(l0);
    lcd_line1(l1);
}

/* ================================================================
   UPDATED MENU RENDERER
   ================================================================ */
void Screen_Init(void)
{
    lcd_init();
    lcd_clear();
    ui = UI_WELCOME;
    last_ui = UI_NONE;
    lastLcdUpdateTime = HAL_GetTick();
}

static void show_menu(void)
{
    char l0[17], l1[17];

    if (menu_idx < menu_view_top)
        menu_view_top = menu_idx;
    else if (menu_idx > (menu_view_top + 1))
        menu_view_top = menu_idx - 1;

    snprintf(l0, sizeof(l0), " %-15.15s", main_menu[menu_view_top]);
    snprintf(l1, sizeof(l1), " %-15.15s", main_menu[menu_view_top + 1]);

    lcd_line0(l0);
    lcd_line1(l1);

    draw_menu_cursor();
}
/***************************************************************
 *  SCREEN.C — PART B / 6
 *  TIMER / AUTO / TWIST / COUNTDOWN / SETTINGS UI
 ***************************************************************/

/* ================================================================
   LCD VALUE FORMATTER
   ================================================================ */
static inline void lcd_val_next_int(uint16_t v){
    char buf[17];
    snprintf(buf, sizeof(buf), "val:%03u   Next>", v);
    lcd_line1(buf);
}

static inline void lcd_val_next_float(float v){
    char buf[17];
    snprintf(buf, sizeof(buf), "val:%0.1f   Next>", v);
    lcd_line1(buf);
}

/* ================================================================
   MANUAL MODE UI
   ================================================================ */

static void show_manual(void){
    lcd_line0("Manual Mode");
    lcd_line1(Motor_GetStatus() ? "val:STOP   Next>"
                                : "val:START  Next>");
}

/* ================================================================
   TIMER MODE — SLOT SELECT (5 slots)
   ================================================================ */

static int timer_page = 0;  // 0,1,2 → scrolling pages

static void show_timer_slot_select(void)
{
    lcd_clear();
    char l0[17] = {0};
    char l1[17] = {0};

    if (timer_page == 0)      // S1, S2, S3, S4
    {
        snprintf(l0, sizeof(l0),
                 "%cS1   %cS2",
                 (currentSlot == 0 ? '>' : ' '),
                 (currentSlot == 1 ? '>' : ' '));

        snprintf(l1, sizeof(l1),
                 "%cS3   %cS4",
                 (currentSlot == 2 ? '>' : ' '),
                 (currentSlot == 3 ? '>' : ' '));
    }
    else if (timer_page == 1) // S3, S4, S5, Back
    {
        snprintf(l0, sizeof(l0),
                 "%cS3   %cS4",
                 (currentSlot == 2 ? '>' : ' '),
                 (currentSlot == 3 ? '>' : ' '));

        snprintf(l1, sizeof(l1),
                 "%cS5   %cBack",
                 (currentSlot == 4 ? '>' :
                  currentSlot == 5 ? '>' : ' '),
                 (currentSlot == 5 ? '>' : ' '));
    }
    else                      // timer_page == 2
    {
        snprintf(l0, sizeof(l0),
                 "%cS5",
                 (currentSlot == 4 ? '>' : ' '));

        snprintf(l1, sizeof(l1),
                 "%cBack",
                 (currentSlot == 5 ? '>' : ' '));
    }

    lcd_line0(l0);
    lcd_line1(l1);
}

static void show_timer(void){
    TimerSlot *t = &timerSlots[currentSlot];

    char l0[17];
    snprintf(l0,sizeof(l0),
             "Slot %d %02d:%02d-%02d:%02d",
             currentSlot+1,
             t->onHour, t->onMinute,
             t->offHour, t->offMinute);

    lcd_line0(l0);
    lcd_line1("val:Edit   Next>");
}

static void show_timer_on_h(void){
    lcd_line0("TIMER ON HOUR");
    lcd_val_next_int(edit_timer_on_h);
}

static void show_timer_on_m(void){
    lcd_line0("TIMER ON MIN");
    lcd_val_next_int(edit_timer_on_m);
}

static void show_timer_off_h(void){
    lcd_line0("TIMER OFF HR");
    lcd_val_next_int(edit_timer_off_h);
}

static void show_timer_off_m(void){
    lcd_line0("TIMER OFF MIN");
    lcd_val_next_int(edit_timer_off_m);
}

/* Timer days: M T W T F S S */
static void show_timer_days(void){
    char pattern[8];
    const char names[7] = {'M','T','W','T','F','S','S'};
    for (int i = 0; i < 7; i++){
        pattern[i] = (edit_timer_dayMask & (1u << i)) ? names[i] : '-';
    }
    pattern[7] = '\0';

    char l0[17];
    snprintf(l0, sizeof(l0), "Days:%-7s", pattern);
    lcd_line0(l0);
    lcd_line1("U:Next D:Toggle");
}

static void show_timer_enable(void){
    lcd_line0("SLOT ENABLE?");
    lcd_line1(edit_timer_enabled ? "val:YES   Next>" : "val:NO    Next>");
}

/* ================================================================
   AUTO MODE SETTINGS
   ================================================================ */

static void show_auto_menu(void){
    lcd_line0("Auto Settings");
    lcd_line1(">Gap/Max/Retry");
}

static void show_auto_gap(void){
    lcd_line0("DRY GAP (s)");
    lcd_val_next_int(edit_auto_gap_s);
}

static void show_auto_maxrun(void){
    lcd_line0("MAX RUN (min)");
    lcd_val_next_int(edit_auto_maxrun_min);
}

static void show_auto_retry(void){
    lcd_line0("RETRY COUNT");
    lcd_val_next_int(edit_auto_retry);
}

/* ================================================================
   SEMI-AUTO UI
   ================================================================ */

static void show_semi_auto(void){
    lcd_line0("Semi-Auto");
    lcd_line1(semiAutoActive ? "val:Disable Next>"
                             : "val:Enable  Next>");
}

/* ================================================================
   TWIST MODE UI
   ================================================================ */

static void show_twist(void){
    char l0[17];
    snprintf(l0,sizeof(l0),
             "Tw %02ds/%02ds",
             twistSettings.onDurationSeconds,
             twistSettings.offDurationSeconds);

    lcd_line0(l0);
    lcd_line1(twistActive ?  "val:STOP   Next>"
                          : "val:START  Next>");
}

static void show_twist_on_sec(void){
    lcd_line0("TWIST ON SEC");
    lcd_val_next_int(edit_twist_on_s);
}

static void show_twist_off_sec(void){
    lcd_line0("TWIST OFF SEC");
    lcd_val_next_int(edit_twist_off_s);
}

static void show_twist_on_h(void){
    lcd_line0("TWIST ON HH");
    lcd_val_next_int(edit_twist_on_hh);
}

static void show_twist_on_m(void){
    lcd_line0("TWIST ON MM");
    lcd_val_next_int(edit_twist_on_mm);
}

static void show_twist_off_h(void){
    lcd_line0("TWIST OFF HH");
    lcd_val_next_int(edit_twist_off_hh);
}

static void show_twist_off_m(void){
    lcd_line0("TWIST OFF MM");
    lcd_val_next_int(edit_twist_off_mm);
}

/* ================================================================
   COUNTDOWN UI
   ================================================================ */

static void show_countdown(void){
    char l0[17], l1[17];

    if (countdownActive){
        uint32_t sec = countdownDuration;
        uint32_t min = sec / 60;
        uint32_t s   = sec % 60;

        snprintf(l0,sizeof(l0),"CD %02u:%02u RUN",min,s);
        snprintf(l1,sizeof(l1),"Press to STOP");
    } else {
        snprintf(l0,sizeof(l0),"CD Set:%3u min",edit_countdown_min);
        snprintf(l1,sizeof(l1),"Press to START");
    }

    lcd_line0(l0);
    lcd_line1(l1);
}

static void show_countdown_edit_min(void){
    lcd_line0("SET MINUTES");
    lcd_val_next_int(edit_countdown_min);
}

/* ================================================================
   NEW SETTINGS UI (OPTION-B)
   ================================================================ */

static void show_settings_gap(void){
    lcd_line0("DRY GAP (s)");
    lcd_val_next_int(edit_settings_gap_s);
}

static void show_settings_retry(void){
    lcd_line0("RETRY COUNT");
    lcd_val_next_int(edit_settings_retry);
}

static void show_settings_uv(void){
    lcd_line0("UV CUT (V)");
    lcd_val_next_int(edit_settings_uv);
}

static void show_settings_ov(void){
    lcd_line0("OV CUT (V)");
    lcd_val_next_int(edit_settings_ov);
}

static void show_settings_ol(void){
    lcd_line0("OVERLOAD (A)");
    lcd_val_next_float(edit_settings_ol);
}

static void show_settings_ul(void){
    lcd_line0("UNDERLOAD (A)");
    lcd_val_next_float(edit_settings_ul);
}

static void show_settings_maxrun(void){
    lcd_line0("MAX RUN (min)");
    lcd_val_next_int(edit_settings_maxrun);
}

static void show_settings_factory(void){
    lcd_line0("FACTORY RESET?");
    lcd_line1(edit_settings_factory_yes ? "val:YES   Next>" : "val:NO    Next>");
}
/***************************************************************
 *  SCREEN.C — PART C / 6
 *  APPLY FUNCTIONS + MENU SELECT ENGINE + SETTINGS FLOW
 ***************************************************************/

/* ================================================================
   APPLY FUNCTIONS
   ================================================================ */

static void apply_timer_settings(void){
    TimerSlot *t = &timerSlots[currentSlot];

    t->onHour    = edit_timer_on_h;
    t->onMinute  = edit_timer_on_m;
    t->offHour   = edit_timer_off_h;
    t->offMinute = edit_timer_off_m;

    t->dayMask   = edit_timer_dayMask;
    t->enabled   = edit_timer_enabled;

    extern void ModelHandle_TimerRecalculateNow(void);
    ModelHandle_TimerRecalculateNow();
}

static void apply_auto_settings(void){
    ModelHandle_SetAutoSettings(
        edit_auto_gap_s,
        edit_auto_maxrun_min,
        edit_auto_retry
    );
}

static void apply_twist_settings(void){
    twistSettings.onDurationSeconds  = edit_twist_on_s;
    twistSettings.offDurationSeconds = edit_twist_off_s;

    twistSettings.onHour   = edit_twist_on_hh;
    twistSettings.onMinute = edit_twist_on_mm;

    twistSettings.offHour  = edit_twist_off_hh;
    twistSettings.offMinute= edit_twist_off_mm;
}

static void apply_countdown_settings(void){
    if (edit_countdown_min == 0)
        edit_countdown_min = 1;

    countdownDuration = (uint32_t)edit_countdown_min * 60u;
}

static void apply_settings_all(void){
    ModelHandle_SetUserSettings(
        edit_settings_gap_s,
        edit_settings_retry,
        edit_settings_uv,
        edit_settings_ov,
        edit_settings_ol,
        edit_settings_ul,
        edit_settings_maxrun
    );

    if (edit_settings_factory_yes)
        ModelHandle_FactoryReset();
}

/* ================================================================
   SETTINGS FLOW (OPTION-B)
   ================================================================ */

static void start_settings_edit_flow(void){
    /* Load existing values */
    edit_settings_gap_s   = ModelHandle_GetGapTime();
    edit_settings_retry   = ModelHandle_GetRetryCount();
    edit_settings_uv      = ModelHandle_GetUnderVolt();
    edit_settings_ov      = ModelHandle_GetOverVolt();
    edit_settings_ol      = ModelHandle_GetOverloadLimit();
    edit_settings_ul      = ModelHandle_GetUnderloadLimit();
    edit_settings_maxrun  = ModelHandle_GetMaxRunTime();
    edit_settings_factory_yes = 0;

    /* Start flow */
    ui = UI_SETTINGS_GAP;
    screenNeedsRefresh = true;
}

static void advance_settings_flow(void){
    switch(ui)
    {
        case UI_SETTINGS_GAP:
            ui = UI_SETTINGS_RETRY;
            break;

        case UI_SETTINGS_RETRY:
            ui = UI_SETTINGS_UV;
            break;

        case UI_SETTINGS_UV:
            ui = UI_SETTINGS_OV;
            break;

        case UI_SETTINGS_OV:
            ui = UI_SETTINGS_OL;
            break;

        case UI_SETTINGS_OL:
            ui = UI_SETTINGS_UL;
            break;

        case UI_SETTINGS_UL:
            ui = UI_SETTINGS_MAXRUN;
            break;

        case UI_SETTINGS_MAXRUN:
            ui = UI_SETTINGS_FACTORY;
            break;

        case UI_SETTINGS_FACTORY:
            apply_settings_all();
            ui = UI_DASH;
            break;

        default:
            ui = UI_DASH;
            break;
    }

    screenNeedsRefresh = true;
}

/* ================================================================
   MENU SELECT HANDLER — Updated For Settings
   ================================================================ */

static void goto_menu_top(void){
    menu_idx = 0;
    menu_view_top = 0;
}

static void menu_select(void){
    refreshInactivityTimer();

    /* WELCOME → DASH */
    if (ui == UI_WELCOME){
        ui = UI_DASH;
        screenNeedsRefresh = true;
        return;
    }

    /* DASH → MENU */
    if (ui == UI_DASH){
        ui = UI_MENU;
        goto_menu_top();
        screenNeedsRefresh = true;
        return;
    }

    /* MAIN MENU SELECT */
    if (ui == UI_MENU){
        switch(menu_idx)
        {
            case 0: // TIMER
                ui = UI_TIMER_SLOT_SELECT;
                timer_page = 0;
                currentSlot = 0;
                break;

            case 1: // AUTO
                ui = UI_AUTO_MENU;
                break;

            case 2: // TWIST
                ui = UI_TWIST;
                break;

            case 3: // SETTINGS (Option-B linear)
                start_settings_edit_flow();
                return;

            case 4: // BACK
                ui = UI_DASH;
                break;
        }

        screenNeedsRefresh = true;
        return;
    }

    /* ---------------- TIMER FLOW ---------------- */

    if (ui == UI_TIMER_SLOT_SELECT){
        if (currentSlot == 5){     // BACK
            ui = UI_DASH;
            screenNeedsRefresh = true;
            return;
        }

        /* Load slot into edit buffer */
        edit_timer_on_h   = timerSlots[currentSlot].onHour;
        edit_timer_on_m   = timerSlots[currentSlot].onMinute;
        edit_timer_off_h  = timerSlots[currentSlot].offHour;
        edit_timer_off_m  = timerSlots[currentSlot].offMinute;
        edit_timer_dayMask= timerSlots[currentSlot].dayMask;
        edit_timer_enabled= timerSlots[currentSlot].enabled;
        edit_timer_dayIndex = 0;

        ui = UI_TIMER_EDIT_SLOT_ON_H;
        screenNeedsRefresh = true;
        return;
    }

    /* TIMER EDIT CHAIN */
    switch(ui)
    {
        case UI_TIMER_EDIT_SLOT_ON_H:
            ui = UI_TIMER_EDIT_SLOT_ON_M;
            break;

        case UI_TIMER_EDIT_SLOT_ON_M:
            ui = UI_TIMER_EDIT_SLOT_OFF_H;
            break;

        case UI_TIMER_EDIT_SLOT_OFF_H:
            ui = UI_TIMER_EDIT_SLOT_OFF_M;
            break;

        case UI_TIMER_EDIT_SLOT_OFF_M:
            ui = UI_TIMER_EDIT_SLOT_DAYS;
            break;

        case UI_TIMER_EDIT_SLOT_DAYS:
            ui = UI_TIMER_EDIT_SLOT_ENABLE;
            break;

        case UI_TIMER_EDIT_SLOT_ENABLE:
            apply_timer_settings();
            ui = UI_TIMER_SLOT_SELECT;
            break;

        default:
            break;
    }

    /* ---------------- AUTO MODE FLOW ---------------- */

    if (ui == UI_AUTO_MENU) {
        autoActive = false;      // 🔥 IMPORTANT FIX
        ModelHandle_StopAuto();  // Ensure motor also stops
        ui = UI_AUTO_EDIT_GAP;
        screenNeedsRefresh = true;
        return;
    }

    if (ui == UI_AUTO_EDIT_GAP){
        ui = UI_AUTO_EDIT_MAXRUN;
        screenNeedsRefresh = true;
        return;
    }
    if (ui == UI_AUTO_EDIT_MAXRUN){
        ui = UI_AUTO_EDIT_RETRY;
        screenNeedsRefresh = true;
        return;
    }
    if (ui == UI_AUTO_EDIT_RETRY){
        apply_auto_settings();
        ui = UI_AUTO_MENU;
        screenNeedsRefresh = true;
        return;
    }

    /* ---------------- TWIST FLOW ---------------- */

    if (ui == UI_TWIST){
        ui = UI_TWIST_EDIT_ON;
        screenNeedsRefresh = true;
        return;
    }
    if (ui == UI_TWIST_EDIT_ON){
        ui = UI_TWIST_EDIT_OFF;
        screenNeedsRefresh = true;
        return;
    }
    if (ui == UI_TWIST_EDIT_OFF){
        ui = UI_TWIST_EDIT_ON_H;
        screenNeedsRefresh = true;
        return;
    }
    if (ui == UI_TWIST_EDIT_ON_H){
        ui = UI_TWIST_EDIT_ON_M;
        screenNeedsRefresh = true;
        return;
    }
    if (ui == UI_TWIST_EDIT_ON_M){
        ui = UI_TWIST_EDIT_OFF_H;
        screenNeedsRefresh = true;
        return;
    }
    if (ui == UI_TWIST_EDIT_OFF_H){
        ui = UI_TWIST_EDIT_OFF_M;
        screenNeedsRefresh = true;
        return;
    }
    if (ui == UI_TWIST_EDIT_OFF_M){
        apply_twist_settings();
        ui = UI_TWIST;
        screenNeedsRefresh = true;
        return;
    }

    /* ---------------- COUNTDOWN FLOW ---------------- */

    if (ui == UI_COUNTDOWN){
        if(countdownActive){
            ModelHandle_StopCountdown();
            ui = UI_DASH;
        } else {
            ui = UI_COUNTDOWN_EDIT_MIN;
        }
        screenNeedsRefresh = true;
        return;
    }

    if (ui == UI_COUNTDOWN_EDIT_MIN){
        apply_countdown_settings();
        ui = UI_COUNTDOWN;
        screenNeedsRefresh = true;
        return;
    }

    /* ---------------- SETTINGS FLOW (OPTION-B) ---------------- */

    if (ui >= UI_SETTINGS_GAP && ui <= UI_SETTINGS_FACTORY){
        advance_settings_flow();
        return;
    }
}
/***************************************************************
 *  SCREEN.C — PART D / 6
 *  BUTTON DECODER + EDIT ENGINE + MAIN SWITCH HANDLER
 ***************************************************************/

/* ================================================================
   BUTTON DECODER — Short/Long press detection
   ================================================================ */

static UiButton decode_button_press(void){
    bool sw1 = Switch_IsPressed(0);
    bool sw2 = Switch_IsPressed(1);
    bool sw3 = Switch_IsPressed(2);
    bool sw4 = Switch_IsPressed(3);

    bool sw[4] = {sw1, sw2, sw3, sw4};

    uint32_t now = HAL_GetTick();
    UiButton out = BTN_NONE;

    for (int i = 0; i < 4; i++)
    {
        if (sw[i] && sw_press_start[i] == 0) {
            sw_press_start[i] = now;
            sw_long_issued[i] = false;
        }
        else if (!sw[i] && sw_press_start[i] != 0){
            if (!sw_long_issued[i]){
                switch(i){
                    case 0: out = BTN_RESET; break;
                    case 1: out = BTN_SELECT; break;
                    case 2: out = BTN_UP; break;
                    case 3: out = BTN_DOWN; break;
                }
            }
            sw_press_start[i] = 0;
            sw_long_issued[i] = false;
        }
        else if (sw[i] && !sw_long_issued[i]){
            if (now - sw_press_start[i] >= LONG_PRESS_MS){
                sw_long_issued[i] = true;
                switch(i){
                    case 0: out = BTN_RESET_LONG; break;
                    case 1: out = BTN_SELECT_LONG; break;
                    case 2: out = BTN_UP_LONG; break;
                    case 3: out = BTN_DOWN_LONG; break;
                }
            }
        }
    }
    return out;
}

/* ================================================================
   EDIT ENGINE (Increase / Decrease)
   ================================================================ */

void increase_edit_value(void){
    switch(ui)
    {
        /* ---------------- TIMER ---------------- */
        case UI_TIMER_EDIT_SLOT_ON_H:  if(edit_timer_on_h < 23) edit_timer_on_h++; break;
        case UI_TIMER_EDIT_SLOT_ON_M:  if(edit_timer_on_m < 59) edit_timer_on_m++; break;
        case UI_TIMER_EDIT_SLOT_OFF_H: if(edit_timer_off_h < 23) edit_timer_off_h++; break;
        case UI_TIMER_EDIT_SLOT_OFF_M: if(edit_timer_off_m < 59) edit_timer_off_m++; break;

        /* ---------------- AUTO ---------------- */
        case UI_AUTO_EDIT_GAP:         edit_auto_gap_s++; break;
        case UI_AUTO_EDIT_MAXRUN:      edit_auto_maxrun_min++; break;
        case UI_AUTO_EDIT_RETRY:       edit_auto_retry++; break;

        /* ---------------- TWIST ---------------- */
        case UI_TWIST_EDIT_ON:         edit_twist_on_s++; break;
        case UI_TWIST_EDIT_OFF:        edit_twist_off_s++; break;

        case UI_TWIST_EDIT_ON_H:       if(edit_twist_on_hh < 23) edit_twist_on_hh++; break;
        case UI_TWIST_EDIT_ON_M:       if(edit_twist_on_mm < 59) edit_twist_on_mm++; break;
        case UI_TWIST_EDIT_OFF_H:      if(edit_twist_off_hh < 23) edit_twist_off_hh++; break;
        case UI_TWIST_EDIT_OFF_M:      if(edit_twist_off_mm < 59) edit_twist_off_mm++; break;

        /* ---------------- COUNTDOWN ---------------- */
        case UI_COUNTDOWN_EDIT_MIN:    if(edit_countdown_min < 999) edit_countdown_min++; break;

        /* ---------------- SETTINGS (Option-B) ---------------- */
        case UI_SETTINGS_GAP:
            edit_settings_gap_s++;
            break;

        case UI_SETTINGS_RETRY:
            edit_settings_retry++;
            break;

        case UI_SETTINGS_UV:
            if (edit_settings_uv < 500) edit_settings_uv++;
            break;

        case UI_SETTINGS_OV:
            if (edit_settings_ov < 500) edit_settings_ov++;
            break;

        case UI_SETTINGS_OL:
            edit_settings_ol += 0.1f;
            if (edit_settings_ol > 50.0f) edit_settings_ol = 50.0f;
            break;

        case UI_SETTINGS_UL:
            edit_settings_ul += 0.1f;
            if (edit_settings_ul > 50.0f) edit_settings_ul = 50.0f;
            break;

        case UI_SETTINGS_MAXRUN:
            edit_settings_maxrun++;
            break;

        case UI_SETTINGS_FACTORY:
            edit_settings_factory_yes ^= 1;
            break;

        default:
            break;
    }
}

void decrease_edit_value(void){
    switch(ui)
    {
        /* ---------------- TIMER ---------------- */
        case UI_TIMER_EDIT_SLOT_ON_H:  if(edit_timer_on_h > 0) edit_timer_on_h--; break;
        case UI_TIMER_EDIT_SLOT_ON_M:  if(edit_timer_on_m > 0) edit_timer_on_m--; break;
        case UI_TIMER_EDIT_SLOT_OFF_H: if(edit_timer_off_h > 0) edit_timer_off_h--; break;
        case UI_TIMER_EDIT_SLOT_OFF_M: if(edit_timer_off_m > 0) edit_timer_off_m--; break;

        /* ---------------- AUTO ---------------- */
        case UI_AUTO_EDIT_GAP:         if(edit_auto_gap_s > 0) edit_auto_gap_s--; break;
        case UI_AUTO_EDIT_MAXRUN:      if(edit_auto_maxrun_min > 0) edit_auto_maxrun_min--; break;
        case UI_AUTO_EDIT_RETRY:       if(edit_auto_retry > 0) edit_auto_retry--; break;

        /* ---------------- TWIST ---------------- */
        case UI_TWIST_EDIT_ON:         if(edit_twist_on_s > 0) edit_twist_on_s--; break;
        case UI_TWIST_EDIT_OFF:        if(edit_twist_off_s > 0) edit_twist_off_s--; break;

        case UI_TWIST_EDIT_ON_H:       if(edit_twist_on_hh > 0) edit_twist_on_hh--; break;
        case UI_TWIST_EDIT_ON_M:       if(edit_twist_on_mm > 0) edit_twist_on_mm--; break;
        case UI_TWIST_EDIT_OFF_H:      if(edit_twist_off_hh > 0) edit_twist_off_hh--; break;
        case UI_TWIST_EDIT_OFF_M:      if(edit_twist_off_mm > 0) edit_twist_off_mm--; break;

        /* ---------------- COUNTDOWN ---------------- */
        case UI_COUNTDOWN_EDIT_MIN:    if(edit_countdown_min > 1) edit_countdown_min--; break;

        /* ---------------- SETTINGS (Option-B) ---------------- */
        case UI_SETTINGS_GAP:
            if (edit_settings_gap_s > 0) edit_settings_gap_s--;
            break;

        case UI_SETTINGS_RETRY:
            if (edit_settings_retry > 0) edit_settings_retry--;
            break;

        case UI_SETTINGS_UV:
            if (edit_settings_uv > 50) edit_settings_uv--;
            break;

        case UI_SETTINGS_OV:
            if (edit_settings_ov > 50) edit_settings_ov--;
            break;

        case UI_SETTINGS_OL:
            edit_settings_ol -= 0.1f;
            if (edit_settings_ol < 0.1f) edit_settings_ol = 0.1f;
            break;

        case UI_SETTINGS_UL:
            edit_settings_ul -= 0.1f;
            if (edit_settings_ul < 0.1f) edit_settings_ul = 0.1f;
            break;

        case UI_SETTINGS_MAXRUN:
            if (edit_settings_maxrun > 1) edit_settings_maxrun--;
            break;

        case UI_SETTINGS_FACTORY:
            edit_settings_factory_yes ^= 1;
            break;

        default:
            break;
    }
}

/* ================================================================
   MAIN SWITCH HANDLER — with updated SETTINGS logic
   ================================================================ */

void Screen_HandleSwitches(void)
{
    UiButton b = decode_button_press();
    uint32_t now = HAL_GetTick();

    bool sw3 = Switch_IsPressed(2);
    bool sw4 = Switch_IsPressed(3);

    /* =====================================================================
       LONG PRESS SPECIALS
       ===================================================================== */

    if (b == BTN_RESET_LONG)
    {
        ModelHandle_ToggleManual();
        ui = UI_DASH;
        screenNeedsRefresh = true;
        return;
    }

    /* Continuous UP (SW3) */
    if (sw3 && sw_long_issued[2]) {
        if (now - last_repeat_time >= CONTINUOUS_STEP_MS) {
            last_repeat_time = now;

            if (ui == UI_MENU){
                if (menu_idx > 0) menu_idx--;
            }
            else if (ui == UI_TIMER_SLOT_SELECT){
                if (currentSlot > 0) currentSlot--;
                timer_page = (currentSlot < 2 ? 0 : (currentSlot < 5 ? 1 : 2));
            }
            else {
                increase_edit_value();
            }

            screenNeedsRefresh = true;
        }
    }

    /* Continuous DOWN (SW4) */
    if (sw4 && sw_long_issued[3]) {

        /* Countdown shortcut from DASH */
        if (ui == UI_DASH || ui == UI_COUNTDOWN){
            static uint32_t cd_last_inc = 0;

            if (ui == UI_DASH) {
                ui = UI_COUNTDOWN;
                screenNeedsRefresh = true;
            }

            show_countdown();

            if (now - cd_last_inc >= COUNTDOWN_INC_MS) {
                cd_last_inc = now;
                edit_countdown_min++;
                countdownDuration = edit_countdown_min * 60;
                show_countdown();
            }
            return;
        }

        if (now - last_repeat_time >= CONTINUOUS_STEP_MS) {
            last_repeat_time = now;

            if (ui == UI_MENU){
                if (menu_idx < MAIN_MENU_COUNT - 1) menu_idx++;
            }
            else if (ui == UI_TIMER_SLOT_SELECT){
                if (currentSlot < 5) currentSlot++;
                timer_page = (currentSlot < 2 ? 0 : (currentSlot < 5 ? 1 : 2));
            }
            else {
                decrease_edit_value();
            }

            screenNeedsRefresh = true;
        }
    }

    if (b == BTN_NONE)
        return;

    refreshInactivityTimer();

    /* =====================================================================
       DETECT IF MENU IS OPEN OR EDITING IS ACTIVE
       ===================================================================== */

    bool menu_open =
        (ui == UI_MENU ||
         ui == UI_TIMER_SLOT_SELECT ||
         ui == UI_TIMER_EDIT_SLOT_ON_H ||
         ui == UI_TIMER_EDIT_SLOT_ON_M ||
         ui == UI_TIMER_EDIT_SLOT_OFF_H ||
         ui == UI_TIMER_EDIT_SLOT_OFF_M ||
         ui == UI_TIMER_EDIT_SLOT_DAYS ||
         ui == UI_TIMER_EDIT_SLOT_ENABLE ||
         ui == UI_AUTO_MENU ||
         ui == UI_AUTO_EDIT_GAP ||
         ui == UI_AUTO_EDIT_MAXRUN ||
         ui == UI_AUTO_EDIT_RETRY ||
         ui == UI_TWIST ||
         ui == UI_TWIST_EDIT_ON ||
         ui == UI_TWIST_EDIT_OFF ||
         ui == UI_TWIST_EDIT_ON_H ||
         ui == UI_TWIST_EDIT_ON_M ||
         ui == UI_TWIST_EDIT_OFF_H ||
         ui == UI_TWIST_EDIT_OFF_M ||
         ui == UI_COUNTDOWN_EDIT_MIN ||
         (ui >= UI_SETTINGS_GAP && ui <= UI_SETTINGS_FACTORY));

    bool editing =
        (ui == UI_TIMER_EDIT_SLOT_ON_H ||
         ui == UI_TIMER_EDIT_SLOT_ON_M ||
         ui == UI_TIMER_EDIT_SLOT_OFF_H ||
         ui == UI_TIMER_EDIT_SLOT_OFF_M ||
         ui == UI_AUTO_EDIT_GAP ||
         ui == UI_AUTO_EDIT_MAXRUN ||
         ui == UI_AUTO_EDIT_RETRY ||
         ui == UI_TWIST_EDIT_ON ||
         ui == UI_TWIST_EDIT_OFF ||
         ui == UI_TWIST_EDIT_ON_H ||
         ui == UI_TWIST_EDIT_ON_M ||
         ui == UI_TWIST_EDIT_OFF_H ||
         ui == UI_TWIST_EDIT_OFF_M ||
         ui == UI_COUNTDOWN_EDIT_MIN ||
         (ui >= UI_SETTINGS_GAP && ui <= UI_SETTINGS_FACTORY));

    /* =====================================================================
       EDIT MODE BEHAVIOR
       ===================================================================== */

    if (menu_open && editing)
    {
        switch(b){
            case BTN_UP:        increase_edit_value(); break;
            case BTN_UP_LONG:   last_repeat_time = now; increase_edit_value(); break;

            case BTN_DOWN:      decrease_edit_value(); break;
            case BTN_DOWN_LONG: last_repeat_time = now; decrease_edit_value(); break;

            case BTN_SELECT:    menu_select(); break;
            case BTN_RESET:     ui = UI_MENU; break;

            default: break;
        }

        screenNeedsRefresh = true;
        return;
    }

    /* =====================================================================
       MENU NAVIGATION
       ===================================================================== */

    if (menu_open && ui == UI_MENU){
        switch(b){
            case BTN_UP:
                if(menu_idx > 0) menu_idx--;
                break;

            case BTN_DOWN:
                if(menu_idx < MAIN_MENU_COUNT-1) menu_idx++;
                break;

            case BTN_SELECT:
                menu_select();
                break;

            case BTN_RESET:
                ui = UI_DASH;
                break;
        }

        screenNeedsRefresh = true;
        return;
    }

    /* =====================================================================
       TIMER SLOT SELECT NAVIGATION
       ===================================================================== */

    if (menu_open && ui == UI_TIMER_SLOT_SELECT){
        switch(b){
            case BTN_UP:
                if (currentSlot > 0) currentSlot--;
                timer_page = (currentSlot < 2 ? 0 : (currentSlot < 5 ? 1 : 2));
                break;

            case BTN_DOWN:
                if (currentSlot < 5) currentSlot++;
                timer_page = (currentSlot < 2 ? 0 : (currentSlot < 5 ? 1 : 2));
                break;

            case BTN_SELECT:
                menu_select();
                break;

            case BTN_RESET:
                ui = UI_MENU;
                break;
        }

        screenNeedsRefresh = true;
        return;
    }

    /* =====================================================================
       DAYS EDIT MODE
       ===================================================================== */

    if (menu_open && ui == UI_TIMER_EDIT_SLOT_DAYS){
        switch(b){
            case BTN_UP:
            case BTN_UP_LONG:
                edit_timer_dayIndex = (edit_timer_dayIndex + 1) % 7;
                break;

            case BTN_DOWN:
            case BTN_DOWN_LONG:
                edit_timer_dayMask ^= (1u << edit_timer_dayIndex);
                break;

            case BTN_SELECT:
                menu_select();
                break;

            case BTN_RESET:
                ui = UI_TIMER_SLOT_SELECT;
                break;
        }

        screenNeedsRefresh = true;
        return;
    }

    /* =====================================================================
       SLOT ENABLE / DISABLE
       ===================================================================== */

    if (menu_open && ui == UI_TIMER_EDIT_SLOT_ENABLE){
        switch(b){
            case BTN_UP:
            case BTN_DOWN:
            case BTN_UP_LONG:
            case BTN_DOWN_LONG:
                edit_timer_enabled = !edit_timer_enabled;
                break;

            case BTN_SELECT:
                apply_timer_settings();
                ui = UI_TIMER_SLOT_SELECT;
                break;

            case BTN_RESET:
                ui = UI_TIMER_SLOT_SELECT;
                break;
        }

        screenNeedsRefresh = true;
        return;
    }

    /* =====================================================================
       SETTINGS FLOW (OPTION B)
       ===================================================================== */

    if (menu_open && (ui >= UI_SETTINGS_GAP && ui <= UI_SETTINGS_FACTORY))
    {
        switch(b){
            case BTN_UP:        increase_edit_value(); break;
            case BTN_DOWN:      decrease_edit_value(); break;
            case BTN_UP_LONG:   last_repeat_time = now; increase_edit_value(); break;
            case BTN_DOWN_LONG: last_repeat_time = now; decrease_edit_value(); break;

            case BTN_SELECT:    advance_settings_flow(); break;

            case BTN_RESET:     ui = UI_MENU; break;
        }

        screenNeedsRefresh = true;
        return;
    }

    /* =====================================================================
       AUTO MODE — BLOCK STARTING AUTO WHILE INSIDE AUTO MENU
       ===================================================================== */

    if (ui == UI_AUTO_MENU ||
        ui == UI_AUTO_EDIT_GAP ||
        ui == UI_AUTO_EDIT_MAXRUN ||
        ui == UI_AUTO_EDIT_RETRY)
    {
        if (b == BTN_SELECT){
            menu_select();     // continue editing
            screenNeedsRefresh = true;
        }
        else if (b == BTN_RESET){
            ui = UI_MENU;
            screenNeedsRefresh = true;
        }
        return;
    }

    /* =====================================================================
       BELOW: NORMAL MODE — DASHBOARD ONLY
       ===================================================================== */

    switch(b)
    {
        case BTN_RESET:
            reset();
            ui = UI_DASH;
            screenNeedsRefresh = true;
            return;

        case BTN_RESET_LONG:
            ModelHandle_ToggleManual();
            ui = UI_DASH;
            screenNeedsRefresh = true;
            return;

        /* SELECT → Start / Stop AUTO (ONLY ON DASHBOARD) */
        case BTN_SELECT:
            if (ui != UI_DASH)
                return;

            if(!autoActive)
                ModelHandle_StartAuto(edit_auto_gap_s, edit_auto_maxrun_min, edit_auto_retry);
            else
                ModelHandle_StopAuto();

            ui = UI_DASH;
            screenNeedsRefresh = true;
            return;

        case BTN_SELECT_LONG:
            ui = UI_MENU;
            goto_menu_top();
            screenNeedsRefresh = true;
            return;

        /* UP → Timer toggle */
        case BTN_UP:
            if(!timerActive)
                ModelHandle_StartTimerNearestSlot();
            else
                ModelHandle_StopTimer();

            ui = UI_DASH;
            screenNeedsRefresh = true;
            return;

        case BTN_UP_LONG:
            if(!semiAutoActive)
                ModelHandle_StartSemiAuto();
            else
                ModelHandle_StopSemiAuto();

            ui = UI_DASH;
            screenNeedsRefresh = true;
            return;

        /* DOWN → Countdown toggle */
        case BTN_DOWN:
            if(!countdownActive){
                ModelHandle_StartCountdown(edit_countdown_min * 60);
                ui = UI_COUNTDOWN;
            } else {
                ModelHandle_StopCountdown();
                ui = UI_DASH;
            }
            screenNeedsRefresh = true;
            return;

        case BTN_DOWN_LONG:
            last_repeat_time = now;
            return;

        default:
            return;
    }
}


/***************************************************************
 *  SCREEN.C — PART E / 6
 *  LCD Update Engine + UI Dispatcher + Cursor Blink
 ***************************************************************/

void Screen_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* ============================================================
       CURSOR BLINK (ONLY FOR MAIN MENU)
       ============================================================ */
    bool cursorBlinkActive = (ui == UI_MENU);

    if (cursorBlinkActive && (now - lastCursorToggle >= CURSOR_BLINK_MS))
    {
        cursorVisible = !cursorVisible;
        lastCursorToggle = now;
        draw_menu_cursor();   // Only draw the '>' cursor, no flicker
    }

    /* ============================================================
       AUTO TRANSITION FROM WELCOME → DASH
       ============================================================ */
    if (ui == UI_WELCOME && now - lastLcdUpdateTime >= WELCOME_MS)
    {
        ui = UI_DASH;
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }

    /* ============================================================
       AUTO-RETURN TO DASH AFTER 60 SECONDS INACTIVITY
       ============================================================ */
    if (ui != UI_WELCOME &&
        ui != UI_DASH &&
        now - lastUserAction >= AUTO_BACK_MS)
    {
        ui = UI_DASH;
        screenNeedsRefresh = true;
    }

    /* ============================================================
       DASHBOARD AUTO-REFRESH EVERY 1 SECOND
       ============================================================ */
    if ((ui == UI_DASH || ui == UI_COUNTDOWN) &&
        now - lastLcdUpdateTime >= 1000)
    {
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }

    /* ============================================================
       IF UI CHANGED OR REFRESH FLAG SET → REDRAW LCD
       ============================================================ */
    if (screenNeedsRefresh || ui != last_ui)
    {
        bool fullRefresh = (ui != last_ui);
        last_ui = ui;
        screenNeedsRefresh = false;

        if (fullRefresh)
            lcd_clear();

        /* ------------------------
           UI DISPATCHER
           ------------------------ */
        switch(ui)
        {
            /* ======== WELCOME / DASH ======== */
            case UI_WELCOME:
                show_welcome();
                break;

            case UI_DASH:
                show_dash();
                break;

            /* ======== MAIN MENU ======== */
            case UI_MENU:
                show_menu();
                break;

            /* ======== TIMER SLOT UI ======== */
            case UI_TIMER_SLOT_SELECT:
                show_timer_slot_select();
                break;

            case UI_TIMER:
                show_timer();
                break;

            case UI_TIMER_EDIT_SLOT_ON_H:
                show_timer_on_h();
                break;

            case UI_TIMER_EDIT_SLOT_ON_M:
                show_timer_on_m();
                break;

            case UI_TIMER_EDIT_SLOT_OFF_H:
                show_timer_off_h();
                break;

            case UI_TIMER_EDIT_SLOT_OFF_M:
                show_timer_off_m();
                break;

            case UI_TIMER_EDIT_SLOT_DAYS:
                show_timer_days();
                break;

            case UI_TIMER_EDIT_SLOT_ENABLE:
                show_timer_enable();
                break;

            /* ======== AUTO MODE UI ======== */
            case UI_AUTO_MENU:
                show_auto_menu();
                break;

            case UI_AUTO_EDIT_GAP:
                show_auto_gap();
                break;

            case UI_AUTO_EDIT_MAXRUN:
                show_auto_maxrun();
                break;

            case UI_AUTO_EDIT_RETRY:
                show_auto_retry();
                break;

            /* ======== TWIST MODE UI ======== */
            case UI_TWIST:
                show_twist();
                break;

            case UI_TWIST_EDIT_ON:
                show_twist_on_sec();
                break;

            case UI_TWIST_EDIT_OFF:
                show_twist_off_sec();
                break;

            case UI_TWIST_EDIT_ON_H:
                show_twist_on_h();
                break;

            case UI_TWIST_EDIT_ON_M:
                show_twist_on_m();
                break;

            case UI_TWIST_EDIT_OFF_H:
                show_twist_off_h();
                break;

            case UI_TWIST_EDIT_OFF_M:
                show_twist_off_m();
                break;

            /* ======== COUNTDOWN UI ======== */
            case UI_COUNTDOWN:
                show_countdown();
                break;

            case UI_COUNTDOWN_EDIT_MIN:
                show_countdown_edit_min();
                break;

            /* ======== SETTINGS UI (OPTION-B LINEAR FLOW) ======== */
            case UI_SETTINGS_GAP:
                show_settings_gap();
                break;

            case UI_SETTINGS_RETRY:
                show_settings_retry();
                break;

            case UI_SETTINGS_UV:
                show_settings_uv();
                break;

            case UI_SETTINGS_OV:
                show_settings_ov();
                break;

            case UI_SETTINGS_OL:
                show_settings_ol();
                break;

            case UI_SETTINGS_UL:
                show_settings_ul();
                break;

            case UI_SETTINGS_MAXRUN:
                show_settings_maxrun();
                break;

            case UI_SETTINGS_FACTORY:
                show_settings_factory();
                break;

            /* ======== DEFAULT (FALLBACK) ======== */
            default:
                break;
        }
    }
}
/***************************************************************
 *  SCREEN.C — PART F / 6 (CLEAN VERSION)
 *  Only missing externs needed for linking
 ***************************************************************/

/* ================================================================
   ONLY DECLARE FUNCTIONS THAT DO NOT ALREADY EXIST IN HEADER
   ================================================================ */

extern void ModelHandle_StartAuto(uint16_t gap_s, uint16_t maxrun_min, uint16_t retry);
extern void ModelHandle_StartTimerNearestSlot(void);
extern void ModelHandle_StopTimer(void);
extern void ModelHandle_StopSemiAuto(void);

/* Factory Reset (if available) */
extern void ModelHandle_FactoryReset(void);

/* ================================================================
   DO NOT DEFINE VARIABLES HERE — THEY WERE ALREADY DEFINED ABOVE
   (static uint16_t edit_settings_gap_s ... etc)
   ================================================================ */

