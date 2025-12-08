/***************************************************************
 *  HELONIX Water Pump Controller
 *  SCREEN.C — FINAL SMOOTH VERSION (OPTION-2)
 *  WITH ENABLE/NO (UP=YES, DOWN=NO, SELECT=NEXT)
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
   UI ENUMS — ALL MODES + NEW TIMER FLOW (CLEAN)
   ================================================================ */
typedef enum {
    UI_WELCOME = 0,
    UI_DASH,
    UI_MENU,

    /* Timer Editing Flow */
    UI_TIMER_SLOT_SELECT,
    UI_TIMER_EDIT_ON_TIME,
    UI_TIMER_EDIT_OFF_TIME,
    UI_TIMER_EDIT_DAYS,
    UI_TIMER_EDIT_GAP,
    UI_TIMER_EDIT_ENABLE,
    UI_TIMER_EDIT_SUMMARY,

    /* Auto Mode */
    UI_AUTO_MENU,
    UI_AUTO_EDIT_GAP,
    UI_AUTO_EDIT_MAXRUN,
    UI_AUTO_EDIT_RETRY,

    /* Semi-Auto */
    UI_SEMI_AUTO,

    /* Twist Mode */
    UI_TWIST,
    UI_TWIST_EDIT_ON,
    UI_TWIST_EDIT_OFF,
    UI_TWIST_EDIT_ON_H,
    UI_TWIST_EDIT_ON_M,
    UI_TWIST_EDIT_OFF_H,
    UI_TWIST_EDIT_OFF_M,

    /* Countdown */
    UI_COUNTDOWN,
    UI_COUNTDOWN_EDIT_MIN,

    /* Settings */
    UI_SETTINGS_GAP,
    UI_SETTINGS_RETRY,
    UI_SETTINGS_UV,
    UI_SETTINGS_OV,
    UI_SETTINGS_OL,
    UI_SETTINGS_UL,
    UI_SETTINGS_MAXRUN,
    UI_SETTINGS_FACTORY,

    UI_NONE,
    UI_MAX_
} UiState;

/* ================================================================
   BUTTON ENUMS — Clean, stable
   ================================================================ */
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
static UiState ui      = UI_WELCOME;
static UiState last_ui = UI_NONE;

static bool screenNeedsRefresh = false;
static bool cursorVisible = true;

static uint32_t lastCursorToggle   = 0;
static uint32_t lastLcdUpdateTime  = 0;
static uint32_t lastUserAction     = 0;

#define WELCOME_MS        2500
#define CURSOR_BLINK_MS    400
#define AUTO_BACK_MS     60000

#define LONG_PRESS_MS     3000
#define CONTINUOUS_STEP_MS 250   // smoother experience
#define COUNTDOWN_INC_MS  2000

/* Button press tracking */
static uint32_t sw_press_start[4] = {0,0,0,0};
static bool     sw_long_issued[4] = {false,false,false,false};

static uint32_t last_repeat_time = 0;

/* ================================================================
   EXTERNAL STATES (from model_handle)
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
   NEW TIMER EDIT VARIABLES — CLEAN + FIXED
   ================================================================ */
static uint8_t edit_on_h = 0;
static uint8_t edit_on_m = 0;

static uint8_t edit_off_h = 0;
static uint8_t edit_off_m = 0;

static uint8_t time_edit_field = 0; // 0 = HH, 1 = MM

/* Days mask & selection index */
static uint8_t edit_day_mask = 0x7F; // default 7 days ON
static uint8_t edit_day_index = 0;    // 0..9

/* Slot gap time (min) */
static uint8_t edit_gap_min = 0;

/* Enable/Disable slot */
static bool edit_slot_enabled = true;

/* Which slot is being edited? */
static uint8_t currentSlot = 0;

/* Slot page: 0 → S1/S2, 1 → S3/S4, 2 → S5/BACK */
static uint8_t timer_page = 0;

/* ================================================================
   AUTO / TWIST / COUNTDOWN / SETTINGS TEMP VARS (unchanged)
   ================================================================ */
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

/* SETTINGS */
static uint16_t edit_settings_gap_s = 10;
static uint8_t  edit_settings_retry = 3;
static uint16_t edit_settings_uv    = 180;
static uint16_t edit_settings_ov    = 260;

static float edit_settings_ol = 6.5f;
static float edit_settings_ul = 0.5f;

static uint16_t edit_settings_maxrun = 120;
static bool     edit_settings_factory_yes = false;

/* ================================================================
   MAIN MENU ITEMS
   ================================================================ */
static const char* const main_menu[] = {
    "Timer Mode",
    "Settings",
    "Back"
};
#define MAIN_MENU_COUNT 3

static uint8_t menu_idx      = 0;
static uint8_t menu_view_top = 0;
void Screen_Init(void)
{
    lcd_init();
    lcd_clear();

    ui = UI_WELCOME;
    last_ui = UI_NONE;

    screenNeedsRefresh = true;
    lastUserAction = HAL_GetTick();
}

/* ================================================================
   LCD Helpers — Safe, clean
   ================================================================ */
static inline void refreshInactivityTimer(void){
    lastUserAction = HAL_GetTick();
}

static inline void lcd_line(uint8_t row, const char* s){
    char buf[17];
    snprintf(buf, sizeof(buf), "%-16.16s", s);
    lcd_put_cur(row,0);
    lcd_send_string(buf);
}

static inline void lcd_line0(const char* s){ lcd_line(0,s); }
static inline void lcd_line1(const char* s){ lcd_line(1,s); }

/* ================================================================
   UI DRAWING FUNCTIONS — CLEAN, MODULAR & FINAL
   ================================================================ */

/***************************************************************
 *  WELCOME SCREEN
 ***************************************************************/
static void show_welcome(void)
{
    lcd_clear();
    lcd_line0("   HELONIX");
    lcd_line1(" IntelligentSys");
}

/***************************************************************
 *  DASHBOARD SCREEN
 ***************************************************************/
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

    /* Compute water level from 5 probes (ADC <0.1 = submerged) */
    int submerged = 0;
    for (int i = 1; i <= 5; i++)
        if (adcData.voltages[i] < 0.1f) submerged++;

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

/***************************************************************
 *  MAIN MENU DRAW
 ***************************************************************/
static void draw_menu_cursor(void)
{
    if (ui != UI_MENU) return;

    uint8_t row = 255;
    if      (menu_idx == menu_view_top)     row = 0;
    else if (menu_idx == menu_view_top + 1) row = 1;

    if (row <= 1)
    {
        lcd_put_cur(row,0);
        lcd_send_data(cursorVisible ? '>' : ' ');
    }
}

static void show_menu(void)
{
    char l0[17], l1[17];

    /* Adjust top-of-view */
    if (menu_idx < menu_view_top)
        menu_view_top = menu_idx;
    else if (menu_idx > menu_view_top + 1)
        menu_view_top = menu_idx - 1;

    snprintf(l0, sizeof(l0), " %-15.15s", main_menu[menu_view_top]);
    snprintf(l1, sizeof(l1), " %-15.15s", main_menu[menu_view_top + 1]);

    lcd_line0(l0);
    lcd_line1(l1);

    draw_menu_cursor();
}

/***************************************************************
 *  TIMER MODE — SLOT SELECT
 *  (Smooth, clean 3-page layout)
 ***************************************************************/
static void show_timer_slot_select(void)
{
    lcd_clear();

    /* Map page → slot indexes */
    int item1 = timer_page * 2;
    int item2 = item1 + 1;

    char l0[17], l1[17];

    /* Line0 */
    if (item1 == 5) {
        snprintf(l0, sizeof(l0), "%c Back",
                 (currentSlot == 5 ? '>' : ' '));
    }
    else {
        snprintf(l0, sizeof(l0), "%c Slot %d",
                 (currentSlot == item1 ? '>' : ' '),
                 item1 + 1);
    }

    /* Line1 */
    if (item2 <= 5)
    {
        if (item2 == 5) {
            snprintf(l1, sizeof(l1), "%c Back",
                     (currentSlot == 5 ? '>' : ' '));
        } else {
            snprintf(l1, sizeof(l1), "%c Slot %d",
                     (currentSlot == item2 ? '>' : ' '),
                     item2 + 1);
        }
    }
    else {
        snprintf(l1, sizeof(l1), "                ");
    }

    lcd_line0(l0);
    lcd_line1(l1);
}

/***************************************************************
 *  TIMER — EDIT ON TIME
 ***************************************************************/
static void show_edit_on_time(void)
{
    lcd_line0("Set ON Time");

    char buf[17];
    if (time_edit_field == 0)
        snprintf(buf, sizeof(buf), "[%02d]:%02d   Next>", edit_on_h, edit_on_m);
    else
        snprintf(buf, sizeof(buf), "%02d:[%02d]   Next>", edit_on_h, edit_on_m);

    lcd_line1(buf);
}

/***************************************************************
 *  TIMER — EDIT OFF TIME
 ***************************************************************/
static void show_edit_off_time(void)
{
    lcd_line0("Set OFF Time");

    char buf[17];
    if (time_edit_field == 0)
        snprintf(buf, sizeof(buf), "[%02d]:%02d   Next>", edit_off_h, edit_off_m);
    else
        snprintf(buf, sizeof(buf), "%02d:[%02d]   Next>", edit_off_h, edit_off_m);

    lcd_line1(buf);
}

/***************************************************************
 *  TIMER — DAYS MENU (0–6 days + EnableAll + DisableAll + Next>)
 ***************************************************************/
static const char* dayNames[] = {
    "Monday", "Tuesday", "Wed", "Thu", "Friday", "Sat", "Sun",
    "Enable All", "Disable All", "Next>"
};

static void show_timer_days(void)
{
    lcd_line0("Select Days");

    char buf[17];

    if (edit_day_index < 7)
    {
        uint8_t isOn = ((edit_day_mask >> edit_day_index) & 1);
        snprintf(buf, sizeof(buf), "> %s (%s)",
                 dayNames[edit_day_index],
                 isOn ? "ON" : "OFF");
    }
    else {
        snprintf(buf, sizeof(buf), "> %s",
                 dayNames[edit_day_index]);
    }

    lcd_line1(buf);
}

/***************************************************************
 *  TIMER — GAP (min)
 ***************************************************************/
static void show_timer_gap(void)
{
    lcd_line0("Set GAP (min)");

    char buf[17];
    snprintf(buf, sizeof(buf), "> %02u min   Next>", edit_gap_min);

    lcd_line1(buf);
}

/***************************************************************
 *  TIMER — ENABLE / DISABLE SLOT
 *  (UP = YES, DOWN = NO, SELECT = NEXT)
 ***************************************************************/
static void show_timer_enable(void)
{
    lcd_line0("Slot Enable?");
    lcd_line1(edit_slot_enabled ? "YES       Next>" :
                                  "NO        Next>");
}

/***************************************************************
 *  TIMER — SUMMARY SCREEN (NO TIMES SHOWN)
 ***************************************************************/
static void show_timer_summary(void)
{
    lcd_line0("Summary");

    if (edit_slot_enabled)
        lcd_line1("Enabled     Next>");
    else
        lcd_line1("Disabled    Next>");
}

/***************************************************************
 *  AUTO MODE
 ***************************************************************/
static void show_auto_menu(void)
{
    lcd_line0("Auto Settings");
    lcd_line1(">Gap/Max/Retry");
}

static void show_auto_gap(void){
    lcd_line0("DRY GAP (s)");
    char buf[17]; snprintf(buf,sizeof(buf),"val:%03u Next>",edit_auto_gap_s);
    lcd_line1(buf);
}
static void show_auto_maxrun(void){
    lcd_line0("MAX RUN (min)");
    char buf[17]; snprintf(buf,sizeof(buf),"val:%03u Next>",edit_auto_maxrun_min);
    lcd_line1(buf);
}
static void show_auto_retry(void){
    lcd_line0("RETRY COUNT");
    char buf[17]; snprintf(buf,sizeof(buf),"val:%03u Next>",edit_auto_retry);
    lcd_line1(buf);
}

/***************************************************************
 *  SEMI-AUTO UI
 ***************************************************************/
static void show_semi_auto(void)
{
    lcd_line0("Semi-Auto");
    lcd_line1(semiAutoActive ? "val:Disable Next>"
                             : "val:Enable  Next>");
}

/***************************************************************
 *  TWIST MODE UI
 ***************************************************************/
static void show_twist(void)
{
    char l0[17];
    snprintf(l0,sizeof(l0),"Tw %02us/%02us",
             (unsigned)twistSettings.onDurationSeconds,
             (unsigned)twistSettings.offDurationSeconds);

    lcd_line0(l0);
    lcd_line1(twistActive ?  "val:STOP   Next>" :
                            "val:START  Next>");
}

static void show_twist_on_sec(void){
    lcd_line0("TWIST ON SEC");
    char buf[17]; snprintf(buf,sizeof(buf),"val:%03u Next>",edit_twist_on_s);
    lcd_line1(buf);
}
static void show_twist_off_sec(void){
    lcd_line0("TWIST OFF SEC");
    char buf[17]; snprintf(buf,sizeof(buf),"val:%03u Next>",edit_twist_off_s);
    lcd_line1(buf);
}
static void show_twist_on_h(void){
    lcd_line0("TWIST ON HH");
    char buf[17]; snprintf(buf,sizeof(buf),"val:%03u Next>",edit_twist_on_hh);
    lcd_line1(buf);
}
static void show_twist_on_m(void){
    lcd_line0("TWIST ON MM");
    char buf[17]; snprintf(buf,sizeof(buf),"val:%03u Next>",edit_twist_on_mm);
    lcd_line1(buf);
}
static void show_twist_off_h(void){
    lcd_line0("TWIST OFF HH");
    char buf[17]; snprintf(buf,sizeof(buf),"val:%03u Next>",edit_twist_off_hh);
    lcd_line1(buf);
}
static void show_twist_off_m(void){
    lcd_line0("TWIST OFF MM");
    char buf[17]; snprintf(buf,sizeof(buf),"val:%03u Next>",edit_twist_off_mm);
    lcd_line1(buf);
}

/***************************************************************
 *  COUNTDOWN UI
 ***************************************************************/
static void show_countdown(void)
{
    char l0[17], l1[17];

    if (countdownActive)
    {
        uint32_t sec = countdownDuration;
        uint32_t min = sec / 60;
        uint32_t s   = sec % 60;

        snprintf(l0,sizeof(l0),"CD %02lu:%02lu RUN",
                 (unsigned long)min, (unsigned long)s);
        snprintf(l1,sizeof(l1),"Press to STOP");
    }
    else
    {
        snprintf(l0,sizeof(l0),"CD Set:%3u min",edit_countdown_min);
        snprintf(l1,sizeof(l1),"Press to START");
    }

    lcd_line0(l0);
    lcd_line1(l1);
}

static void show_countdown_edit_min(void){
    lcd_line0("SET MINUTES");
    char buf[17]; snprintf(buf,sizeof(buf),"val:%03u Next>",edit_countdown_min);
    lcd_line1(buf);
}

/***************************************************************
 *  SETTINGS UI
 ***************************************************************/
static void show_settings_gap(void){
    lcd_line0("DRY GAP (s)");
    char buf[17]; snprintf(buf,sizeof(buf),"val:%03u Next>",edit_settings_gap_s);
    lcd_line1(buf);
}
static void show_settings_retry(void){
    lcd_line0("RETRY COUNT");
    char buf[17]; snprintf(buf,sizeof(buf),"val:%03u Next>",edit_settings_retry);
    lcd_line1(buf);
}
static void show_settings_uv(void){
    lcd_line0("UV CUT (V)");
    char buf[17]; snprintf(buf,sizeof(buf),"val:%03u Next>",edit_settings_uv);
    lcd_line1(buf);
}
static void show_settings_ov(void){
    lcd_line0("OV CUT (V)");
    char buf[17]; snprintf(buf,sizeof(buf),"val:%03u Next>",edit_settings_ov);
    lcd_line1(buf);
}
static void show_settings_ol(void){
    lcd_line0("OVERLOAD (A)");
    char buf[17]; snprintf(buf,sizeof(buf),"val:%0.1f Next>",edit_settings_ol);
    lcd_line1(buf);
}
static void show_settings_ul(void){
    lcd_line0("UNDERLOAD (A)");
    char buf[17]; snprintf(buf,sizeof(buf),"val:%0.1f Next>",edit_settings_ul);
    lcd_line1(buf);
}
static void show_settings_maxrun(void){
    lcd_line0("MAX RUN (min)");
    char buf[17]; snprintf(buf,sizeof(buf),"val:%03u Next>",edit_settings_maxrun);
    lcd_line1(buf);
}
static void show_settings_factory(void){
    lcd_line0("FACTORY RESET?");
    lcd_line1(edit_settings_factory_yes ? "YES       Next>" :
                                         "NO        Next>");
}

/* ================================================================
   PART-3
   APPLY FUNCTIONS + MENU SELECT LOGIC
   ================================================================ */

/***************************************************************
 *  APPLY — SAVE TIMER SLOT (Final clean version)
 ***************************************************************/
static void apply_timer_slot(void)
{
    TimerSlot *t = &timerSlots[currentSlot];

    /* WRITE ON-TIME */
    t->onHour    = edit_on_h;
    t->onMinute  = edit_on_m;

    /* WRITE OFF-TIME */
    t->offHour   = edit_off_h;
    t->offMinute = edit_off_m;

    /* WRITE DAYS MASK */
    t->dayMask   = edit_day_mask;

    /* WRITE GAP */
    t->gapMinutes = edit_gap_min;

    /* ENABLE / DISABLE SLOT */
    t->enabled = edit_slot_enabled;

    /* Notify backend */
    extern void ModelHandle_TimerRecalculateNow(void);
    ModelHandle_TimerRecalculateNow();
}

/***************************************************************
 *  APPLY — AUTO SETTINGS
 ***************************************************************/
static void apply_auto_settings(void)
{
    ModelHandle_SetAutoSettings(
        edit_auto_gap_s,
        edit_auto_maxrun_min,
        edit_auto_retry
    );
}

/***************************************************************
 *  APPLY — TWIST SETTINGS
 ***************************************************************/
static void apply_twist_settings(void)
{
    twistSettings.onDurationSeconds  = edit_twist_on_s;
    twistSettings.offDurationSeconds = edit_twist_off_s;

    twistSettings.onHour   = edit_twist_on_hh;
    twistSettings.onMinute = edit_twist_on_mm;

    twistSettings.offHour  = edit_twist_off_hh;
    twistSettings.offMinute= edit_twist_off_mm;
}

/***************************************************************
 *  APPLY — COUNTDOWN SETTINGS
 ***************************************************************/
static void apply_countdown_settings(void)
{
    if (edit_countdown_min == 0)
        edit_countdown_min = 1;

    countdownDuration = (uint32_t)edit_countdown_min * 60u;
}

/***************************************************************
 *  APPLY — ALL USER SETTINGS
 ***************************************************************/
static void apply_settings_all(void)
{
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

/***************************************************************
 *  SETTINGS — START EDIT FLOW
 ***************************************************************/
static void start_settings_edit_flow(void)
{
    /* Load fresh settings */
    edit_settings_gap_s   = ModelHandle_GetGapTime();
    edit_settings_retry   = ModelHandle_GetRetryCount();
    edit_settings_uv      = ModelHandle_GetUnderVolt();
    edit_settings_ov      = ModelHandle_GetOverVolt();
    edit_settings_ol      = ModelHandle_GetOverloadLimit();
    edit_settings_ul      = ModelHandle_GetUnderloadLimit();
    edit_settings_maxrun  = ModelHandle_GetMaxRunTime();

    edit_settings_factory_yes = 0;

    ui = UI_SETTINGS_GAP;
    screenNeedsRefresh = true;
}

/***************************************************************
 *  SETTINGS — LINEAR FLOW SEQUENCER
 ***************************************************************/
static void advance_settings_flow(void)
{
    switch(ui)
    {
        case UI_SETTINGS_GAP:       ui = UI_SETTINGS_RETRY; break;
        case UI_SETTINGS_RETRY:     ui = UI_SETTINGS_UV;    break;
        case UI_SETTINGS_UV:        ui = UI_SETTINGS_OV;    break;
        case UI_SETTINGS_OV:        ui = UI_SETTINGS_OL;    break;
        case UI_SETTINGS_OL:        ui = UI_SETTINGS_UL;    break;
        case UI_SETTINGS_UL:        ui = UI_SETTINGS_MAXRUN; break;
        case UI_SETTINGS_MAXRUN:    ui = UI_SETTINGS_FACTORY; break;

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

/***************************************************************
 *  MENU SELECT — MAIN NAVIGATION + TIMER FLOW
 *  (FULLY REWRITTEN, smooth working)
 ***************************************************************/
static void goto_menu_top(void)
{
    menu_idx = 0;
    menu_view_top = 0;
}

static void menu_select(void)
{
    refreshInactivityTimer();

    /* ----------------------------------------------------------
       WELCOME → DASH
       ---------------------------------------------------------- */
    if (ui == UI_WELCOME)
    {
        ui = UI_DASH;
        screenNeedsRefresh = true;
        return;
    }

    /* ----------------------------------------------------------
       DASH → MENU
       ---------------------------------------------------------- */
    if (ui == UI_DASH)
    {
        goto_menu_top();
        ui = UI_MENU;
        screenNeedsRefresh = true;
        return;
    }

    /* ----------------------------------------------------------
       MAIN MENU (Timer / Settings / Back)
       ---------------------------------------------------------- */
    if (ui == UI_MENU)
    {
        switch(menu_idx)
        {
            case 0:   /* TIMER MODE */
                currentSlot = 0;
                timer_page = 0;
                ui = UI_TIMER_SLOT_SELECT;
                screenNeedsRefresh = true;
                return;

            case 1:   /* SETTINGS */
                start_settings_edit_flow();
                return;

            case 2:   /* BACK → DASH */
                ui = UI_DASH;
                screenNeedsRefresh = true;
                return;
        }
    }

    /* ----------------------------------------------------------
       SLOT SELECT → LOAD SLOT → BEGIN EDITING
       ---------------------------------------------------------- */
    if (ui == UI_TIMER_SLOT_SELECT)
    {
        if (currentSlot == 5)
        {
            ui = UI_MENU;  // BACK
            screenNeedsRefresh = true;
            return;
        }

        /* Load selected slot values */
        TimerSlot *t = &timerSlots[currentSlot];

        edit_on_h  = t->onHour;
        edit_on_m  = t->onMinute;

        edit_off_h = t->offHour;
        edit_off_m = t->offMinute;

        edit_day_mask = t->dayMask;
        edit_gap_min  = t->gapMinutes;

        edit_slot_enabled = t->enabled;

        /* Reset cursors */
        time_edit_field = 0;
        edit_day_index  = 0;

        ui = UI_TIMER_EDIT_ON_TIME;
        screenNeedsRefresh = true;
        return;
    }

    /* ----------------------------------------------------------
       TIMER EDIT FLOW SEQUENCER (Option-A)
       ---------------------------------------------------------- */
    switch(ui)
    {
        /* -------- ON TIME -------- */
        case UI_TIMER_EDIT_ON_TIME:
            if (time_edit_field == 0)
                time_edit_field = 1;     // HH → MM
            else {
                time_edit_field = 0;
                ui = UI_TIMER_EDIT_OFF_TIME;
            }
            break;

        /* -------- OFF TIME -------- */
        case UI_TIMER_EDIT_OFF_TIME:
            if (time_edit_field == 0)
                time_edit_field = 1;
            else {
                time_edit_field = 0;
                ui = UI_TIMER_EDIT_DAYS;
            }
            break;

        /* -------- DAYS -------- */
        case UI_TIMER_EDIT_DAYS:
            if (edit_day_index == 9)
                ui = UI_TIMER_EDIT_GAP;   // Next>
            else
                edit_day_index++;
            break;

        /* -------- GAP -------- */
        case UI_TIMER_EDIT_GAP:
            ui = UI_TIMER_EDIT_ENABLE;
            break;

        /* -------- ENABLE / DISABLE -------- */
        case UI_TIMER_EDIT_ENABLE:
            ui = UI_TIMER_EDIT_SUMMARY;
            break;

        /* -------- SUMMARY + SAVE -------- */
        case UI_TIMER_EDIT_SUMMARY:
            apply_timer_slot();
            ui = UI_TIMER_SLOT_SELECT;
            break;

        default:
            break;
    }

    screenNeedsRefresh = true;
}
/* ================================================================
   PART-4
   VALUE EDIT ENGINE + FULL SWITCH HANDLER
   ================================================================ */

/***************************************************************
 *  EDIT ENGINE — INCREASE VALUES (smooth, safe)
 ***************************************************************/
void increase_edit_value(void)
{
    switch(ui)
    {
        /* TIMER ON TIME */
        case UI_TIMER_EDIT_ON_TIME:
            if (time_edit_field == 0) { if (edit_on_h < 23) edit_on_h++; }
            else                      { if (edit_on_m < 59) edit_on_m++; }
            break;

        /* TIMER OFF TIME */
        case UI_TIMER_EDIT_OFF_TIME:
            if (time_edit_field == 0) { if (edit_off_h < 23) edit_off_h++; }
            else                      { if (edit_off_m < 59) edit_off_m++; }
            break;

        /* TIMER GAP */
        case UI_TIMER_EDIT_GAP:
            if (edit_gap_min < 240) edit_gap_min++;  // limit 4 hours
            break;

        /* AUTO SETTINGS */
        case UI_AUTO_EDIT_GAP:      edit_auto_gap_s++; break;
        case UI_AUTO_EDIT_MAXRUN:   edit_auto_maxrun_min++; break;
        case UI_AUTO_EDIT_RETRY:    edit_auto_retry++; break;

        /* TWIST MODE */
        case UI_TWIST_EDIT_ON:    edit_twist_on_s++; break;
        case UI_TWIST_EDIT_OFF:   edit_twist_off_s++; break;

        case UI_TWIST_EDIT_ON_H:  if (edit_twist_on_hh < 23) edit_twist_on_hh++; break;
        case UI_TWIST_EDIT_ON_M:  if (edit_twist_on_mm < 59) edit_twist_on_mm++; break;

        case UI_TWIST_EDIT_OFF_H: if (edit_twist_off_hh < 23) edit_twist_off_hh++; break;
        case UI_TWIST_EDIT_OFF_M: if (edit_twist_off_mm < 59) edit_twist_off_mm++; break;

        /* COUNTDOWN */
        case UI_COUNTDOWN_EDIT_MIN:
            if (edit_countdown_min < 999) edit_countdown_min++;
            break;

        /* SETTINGS */
        case UI_SETTINGS_GAP:     edit_settings_gap_s++; break;
        case UI_SETTINGS_RETRY:   edit_settings_retry++; break;

        case UI_SETTINGS_UV:
            if (edit_settings_uv < 500) edit_settings_uv++;
            break;

        case UI_SETTINGS_OV:
            if (edit_settings_ov < 500) edit_settings_ov++;
            break;

        case UI_SETTINGS_OL:
            edit_settings_ol += 0.1f;
            if (edit_settings_ol > 50) edit_settings_ol = 50;
            break;

        case UI_SETTINGS_UL:
            edit_settings_ul += 0.1f;
            if (edit_settings_ul > 50) edit_settings_ul = 50;
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

/***************************************************************
 *  EDIT ENGINE — DECREASE VALUES
 ***************************************************************/
void decrease_edit_value(void)
{
    switch(ui)
    {
        /* TIMER ON TIME */
        case UI_TIMER_EDIT_ON_TIME:
            if (time_edit_field == 0) { if (edit_on_h > 0) edit_on_h--; }
            else                      { if (edit_on_m > 0) edit_on_m--; }
            break;

        /* TIMER OFF TIME */
        case UI_TIMER_EDIT_OFF_TIME:
            if (time_edit_field == 0) { if (edit_off_h > 0) edit_off_h--; }
            else                      { if (edit_off_m > 0) edit_off_m--; }
            break;

        /* TIMER GAP */
        case UI_TIMER_EDIT_GAP:
            if (edit_gap_min > 0) edit_gap_min--;
            break;

        /* AUTO SETTINGS */
        case UI_AUTO_EDIT_GAP:      if(edit_auto_gap_s > 0) edit_auto_gap_s--; break;
        case UI_AUTO_EDIT_MAXRUN:   if(edit_auto_maxrun_min > 0) edit_auto_maxrun_min--; break;
        case UI_AUTO_EDIT_RETRY:    if(edit_auto_retry > 0) edit_auto_retry--; break;

        /* TWIST */
        case UI_TWIST_EDIT_ON:      if(edit_twist_on_s > 0) edit_twist_on_s--; break;
        case UI_TWIST_EDIT_OFF:     if(edit_twist_off_s > 0) edit_twist_off_s--; break;

        case UI_TWIST_EDIT_ON_H:    if(edit_twist_on_hh > 0) edit_twist_on_hh--; break;
        case UI_TWIST_EDIT_ON_M:    if(edit_twist_on_mm > 0) edit_twist_on_mm--; break;

        case UI_TWIST_EDIT_OFF_H:   if(edit_twist_off_hh > 0) edit_twist_off_hh--; break;
        case UI_TWIST_EDIT_OFF_M:   if(edit_twist_off_mm > 0) edit_twist_off_mm--; break;

        /* COUNTDOWN */
        case UI_COUNTDOWN_EDIT_MIN:
            if (edit_countdown_min > 1) edit_countdown_min--;
            break;

        /* SETTINGS */
        case UI_SETTINGS_GAP:     if(edit_settings_gap_s > 0) edit_settings_gap_s--; break;
        case UI_SETTINGS_RETRY:   if(edit_settings_retry > 0) edit_settings_retry--; break;

        case UI_SETTINGS_UV:
            if(edit_settings_uv > 50) edit_settings_uv--;
            break;

        case UI_SETTINGS_OV:
            if(edit_settings_ov > 50) edit_settings_ov--;
            break;

        case UI_SETTINGS_OL:
            edit_settings_ol -= 0.1f;
            if(edit_settings_ol < 0.1f) edit_settings_ol = 0.1f;
            break;

        case UI_SETTINGS_UL:
            edit_settings_ul -= 0.1f;
            if(edit_settings_ul < 0.1f) edit_settings_ul = 0.1f;
            break;

        case UI_SETTINGS_MAXRUN:
            if(edit_settings_maxrun > 1) edit_settings_maxrun--;
            break;

        case UI_SETTINGS_FACTORY:
            edit_settings_factory_yes ^= 1;
            break;

        default:
            break;
    }
}

/***************************************************************
 *  BUTTON DECODER (SHORT/LONG DETECTION)
 ***************************************************************/
static UiButton decode_button_press(void)
{
    bool sw[4] = {
        Switch_IsPressed(0),
        Switch_IsPressed(1),
        Switch_IsPressed(2),
        Switch_IsPressed(3)
    };

    uint32_t now = HAL_GetTick();
    UiButton out = BTN_NONE;

    for (int i = 0; i < 4; i++)
    {
        if (sw[i] && sw_press_start[i] == 0)
        {
            sw_press_start[i] = now;
            sw_long_issued[i] = false;
        }
        else if (!sw[i] && sw_press_start[i] != 0)
        {
            if (!sw_long_issued[i])
            {
                switch(i){
                    case 0: out = BTN_RESET;  break;
                    case 1: out = BTN_SELECT; break;
                    case 2: out = BTN_UP;     break;
                    case 3: out = BTN_DOWN;   break;
                }
            }
            sw_press_start[i] = 0;
            sw_long_issued[i] = false;
        }
        else if (sw[i] && !sw_long_issued[i])
        {
            if (now - sw_press_start[i] >= LONG_PRESS_MS)
            {
                sw_long_issued[i] = true;
                switch(i){
                    case 0: out = BTN_RESET_LONG;  break;
                    case 1: out = BTN_SELECT_LONG; break;
                    case 2: out = BTN_UP_LONG;     break;
                    case 3: out = BTN_DOWN_LONG;   break;
                }
            }
        }
    }
    return out;
}

/***************************************************************
 *  MAIN SWITCH HANDLER — FINAL SMOOTH VERSION
 ***************************************************************/
void Screen_HandleSwitches(void)
{
    UiButton b = decode_button_press();
    uint32_t now = HAL_GetTick();

    bool sw_up   = Switch_IsPressed(2);
    bool sw_down = Switch_IsPressed(3);

    /* ============================================================
       LONG PRESS RESET → manual toggle
       ============================================================ */
    if (b == BTN_RESET_LONG)
    {
        ModelHandle_ToggleManual();
        ui = UI_DASH;
        screenNeedsRefresh = true;
        return;
    }

    /* ============================================================
       CONTINUOUS UP (HOLD)
       ============================================================ */
    if (sw_up && sw_long_issued[2])
    {
        if (now - last_repeat_time >= CONTINUOUS_STEP_MS)
        {
            last_repeat_time = now;

            if (ui == UI_MENU)
            {
                if (menu_idx > 0) menu_idx--;
            }
            else if (ui == UI_TIMER_SLOT_SELECT)
            {
                if (currentSlot > 0) currentSlot--;
                timer_page = (currentSlot < 2 ? 0 :
                             (currentSlot < 5 ? 1 : 2));
            }
            else
                increase_edit_value();

            screenNeedsRefresh = true;
        }
    }

    /* ============================================================
       CONTINUOUS DOWN (HOLD)
       ============================================================ */
    if (sw_down && sw_long_issued[3])
    {
        if (now - last_repeat_time >= CONTINUOUS_STEP_MS)
        {
            last_repeat_time = now;

            if (ui == UI_MENU)
            {
                if (menu_idx < MAIN_MENU_COUNT-1) menu_idx++;
            }
            else if (ui == UI_TIMER_SLOT_SELECT)
            {
                if (currentSlot < 5) currentSlot++;
                timer_page = (currentSlot < 2 ? 0 :
                             (currentSlot < 5 ? 1 : 2));
            }
            else
                decrease_edit_value();

            screenNeedsRefresh = true;
        }
    }

    if (b == BTN_NONE) return;
    refreshInactivityTimer();

    /* ============================================================
       EDITING MODES (generic)
       ============================================================ */
    bool editing =
        ( ui == UI_TIMER_EDIT_ON_TIME  ||
          ui == UI_TIMER_EDIT_OFF_TIME ||
          ui == UI_TIMER_EDIT_GAP      ||
          ui == UI_AUTO_EDIT_GAP       ||
          ui == UI_AUTO_EDIT_MAXRUN    ||
          ui == UI_AUTO_EDIT_RETRY     ||
          ui == UI_TWIST_EDIT_ON       ||
          ui == UI_TWIST_EDIT_OFF      ||
          ui == UI_TWIST_EDIT_ON_H     ||
          ui == UI_TWIST_EDIT_ON_M     ||
          ui == UI_TWIST_EDIT_OFF_H    ||
          ui == UI_TWIST_EDIT_OFF_M    ||
          ui == UI_COUNTDOWN_EDIT_MIN  ||
          (ui >= UI_SETTINGS_GAP && ui <= UI_SETTINGS_FACTORY) );

    if (editing)
    {
        switch(b)
        {
            case BTN_UP:        increase_edit_value(); break;
            case BTN_DOWN:      decrease_edit_value(); break;
            case BTN_UP_LONG:   last_repeat_time = now; increase_edit_value(); break;
            case BTN_DOWN_LONG: last_repeat_time = now; decrease_edit_value(); break;
            case BTN_SELECT:    menu_select(); break;
            case BTN_RESET:     ui = UI_MENU; break;
        }

        screenNeedsRefresh = true;
        return;
    }

    /* ============================================================
       DAY SELECTION (0…6 days + Enable All + Disable All + Next)
       ============================================================ */
    if (ui == UI_TIMER_EDIT_DAYS)
    {
        switch(b)
        {
            case BTN_UP:
            case BTN_UP_LONG:
                if (edit_day_index == 0) edit_day_index = 9;
                else edit_day_index--;
                break;

            case BTN_DOWN:
            case BTN_DOWN_LONG:
                edit_day_index = (edit_day_index + 1) % 10;
                break;

            case BTN_SELECT:
                if (edit_day_index < 7)
                    edit_day_mask ^= (1u << edit_day_index);
                else if (edit_day_index == 7)
                    edit_day_mask = 0x7F;
                else if (edit_day_index == 8)
                    edit_day_mask = 0x00;
                else if (edit_day_index == 9)
                    ui = UI_TIMER_EDIT_GAP;
                break;

            case BTN_RESET:
                ui = UI_TIMER_SLOT_SELECT;
                break;
        }

        screenNeedsRefresh = true;
        return;
    }

    /* ============================================================
       ENABLE / DISABLE SLOT (Option-A: UP=YES, DOWN=NO, SELECT=NEXT)
       ============================================================ */
    if (ui == UI_TIMER_EDIT_ENABLE)
    {
        switch(b)
        {
            case BTN_UP:
            case BTN_UP_LONG:
                edit_slot_enabled = true;  // YES
                break;

            case BTN_DOWN:
            case BTN_DOWN_LONG:
                edit_slot_enabled = false; // NO
                break;

            case BTN_SELECT:
                ui = UI_TIMER_EDIT_SUMMARY;
                break;

            case BTN_RESET:
                ui = UI_TIMER_SLOT_SELECT;
                break;
        }

        screenNeedsRefresh = true;
        return;
    }

    /* ============================================================
       MAIN MENU
       ============================================================ */
    if (ui == UI_MENU)
    {
        switch(b)
        {
            case BTN_UP:     if(menu_idx > 0) menu_idx--; break;
            case BTN_DOWN:   if(menu_idx < MAIN_MENU_COUNT-1) menu_idx++; break;
            case BTN_SELECT: menu_select(); break;
            case BTN_RESET:  ui = UI_DASH; break;
        }
        screenNeedsRefresh = true;
        return;
    }

    /* ============================================================
       SLOT SELECT SCREEN
       ============================================================ */
    if (ui == UI_TIMER_SLOT_SELECT)
    {
        switch(b)
        {
            case BTN_UP:
                if (currentSlot > 0) currentSlot--;
                break;

            case BTN_DOWN:
                if (currentSlot < 5) currentSlot++;
                break;

            case BTN_SELECT:
                menu_select();
                break;

            case BTN_RESET:
                ui = UI_MENU;
                break;
        }

        timer_page = (currentSlot < 2 ? 0 :
                     (currentSlot < 5 ? 1 : 2));

        screenNeedsRefresh = true;
        return;
    }

    /* ============================================================
       DASHBOARD ACTIONS
       ============================================================ */
    switch(b)
    {
        case BTN_RESET:
            reset();
            ui = UI_DASH;
            screenNeedsRefresh = true;
            return;

        case BTN_SELECT:
            if (ui != UI_DASH) return;
            if(!autoActive) ModelHandle_StartAuto(edit_auto_gap_s, edit_auto_maxrun_min, edit_auto_retry);
            else            ModelHandle_StopAuto();
            screenNeedsRefresh = true;
            return;

        case BTN_SELECT_LONG:
            ui = UI_MENU;
            menu_idx = 0;
            screenNeedsRefresh = true;
            return;

        case BTN_UP:
            if(!timerActive) ModelHandle_StartTimerNearestSlot();
            else             ModelHandle_StopTimer();
            ui = UI_DASH;
            screenNeedsRefresh = true;
            return;

        case BTN_UP_LONG:
            if(!semiAutoActive) ModelHandle_StartSemiAuto();
            else                ModelHandle_StopSemiAuto();
            ui = UI_DASH;
            screenNeedsRefresh = true;
            return;

        case BTN_DOWN:
            if(!countdownActive){
                ModelHandle_StartCountdown(edit_countdown_min * 60);
                ui = UI_COUNTDOWN;
            }
            else {
                ModelHandle_StopCountdown();
                ui = UI_DASH;
            }
            screenNeedsRefresh = true;
            return;

        default:
            return;
    }
}
/***************************************************************
 *  PART-5 — LCD UPDATE ENGINE + UI DISPATCHER
 ***************************************************************/
void Screen_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* ============================================================
       CURSOR BLINK ONLY IN MENU
       ============================================================ */
    bool cursorBlinkActive = (ui == UI_MENU);

    if (cursorBlinkActive && (now - lastCursorToggle >= CURSOR_BLINK_MS))
    {
        cursorVisible = !cursorVisible;
        lastCursorToggle = now;
        draw_menu_cursor();  // Efficient — updates only cursor
    }

    /* ============================================================
       WELCOME SCREEN → AUTO JUMP TO DASH
       ============================================================ */
    if (ui == UI_WELCOME && now - lastLcdUpdateTime >= WELCOME_MS)
    {
        ui = UI_DASH;
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }

    /* ============================================================
       AUTO RETURN TO DASH AFTER INACTIVITY
       ============================================================ */
    if (ui != UI_WELCOME &&
        ui != UI_DASH &&
        (now - lastUserAction >= AUTO_BACK_MS))
    {
        ui = UI_DASH;
        screenNeedsRefresh = true;
    }

    /* ============================================================
       DASH & COUNTDOWN REFRESH EVERY 1 SEC
       ============================================================ */
    if ((ui == UI_DASH || ui == UI_COUNTDOWN) &&
        now - lastLcdUpdateTime >= 1000)
    {
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }

    /* ============================================================
       REDRAW SCREEN IF NEEDED
       ============================================================ */
    if (screenNeedsRefresh || ui != last_ui)
    {
        bool fullRefresh = (ui != last_ui);
        last_ui = ui;
        screenNeedsRefresh = false;

        if (fullRefresh)
            lcd_clear();

        /* ============================================================
           UI DISPATCHER — CALL PROPER DRAW FUNCTION
           ============================================================ */
        switch(ui)
        {
            /* ---------------- WELCOME + DASH ---------------- */
            case UI_WELCOME: show_welcome(); break;
            case UI_DASH:    show_dash();    break;

            /* ----------------- MAIN MENU -------------------- */
            case UI_MENU: show_menu(); break;

            /* ---------------- TIMER MODE -------------------- */
            case UI_TIMER_SLOT_SELECT:  show_timer_slot_select(); break;

            case UI_TIMER_EDIT_ON_TIME:   show_edit_on_time();   break;
            case UI_TIMER_EDIT_OFF_TIME:  show_edit_off_time();  break;
            case UI_TIMER_EDIT_DAYS:      show_timer_days();      break;
            case UI_TIMER_EDIT_GAP:       show_timer_gap();       break;
            case UI_TIMER_EDIT_ENABLE:    show_timer_enable();   break;
            case UI_TIMER_EDIT_SUMMARY:   show_timer_summary();  break;

            /* ---------------- AUTO MODE --------------------- */
            case UI_AUTO_MENU:        show_auto_menu();        break;
            case UI_AUTO_EDIT_GAP:    show_auto_gap();         break;
            case UI_AUTO_EDIT_MAXRUN: show_auto_maxrun();      break;
            case UI_AUTO_EDIT_RETRY:  show_auto_retry();       break;

            /* ---------------- SEMI-AUTO --------------------- */
            case UI_SEMI_AUTO: show_semi_auto(); break;

            /* ---------------- TWIST MODE -------------------- */
            case UI_TWIST:             show_twist();          break;
            case UI_TWIST_EDIT_ON:     show_twist_on_sec();   break;
            case UI_TWIST_EDIT_OFF:    show_twist_off_sec();  break;
            case UI_TWIST_EDIT_ON_H:   show_twist_on_h();     break;
            case UI_TWIST_EDIT_ON_M:   show_twist_on_m();     break;
            case UI_TWIST_EDIT_OFF_H:  show_twist_off_h();    break;
            case UI_TWIST_EDIT_OFF_M:  show_twist_off_m();    break;

            /* ---------------- COUNTDOWN --------------------- */
            case UI_COUNTDOWN:          show_countdown();        break;
            case UI_COUNTDOWN_EDIT_MIN: show_countdown_edit_min(); break;

            /* ---------------- SETTINGS ---------------------- */
            case UI_SETTINGS_GAP:      show_settings_gap();      break;
            case UI_SETTINGS_RETRY:    show_settings_retry();    break;
            case UI_SETTINGS_UV:       show_settings_uv();       break;
            case UI_SETTINGS_OV:       show_settings_ov();       break;
            case UI_SETTINGS_OL:       show_settings_ol();       break;
            case UI_SETTINGS_UL:       show_settings_ul();       break;
            case UI_SETTINGS_MAXRUN:   show_settings_maxrun();   break;
            case UI_SETTINGS_FACTORY:  show_settings_factory();  break;

            default:
                break;
        }
    }
}

/***************************************************************
 *  FINAL EXTERNS — CONNECTED TO MODEL ENGINE
 ***************************************************************/
extern void ModelHandle_StartAuto(uint16_t gap_s, uint16_t maxrun_min, uint16_t retry);
extern void ModelHandle_StartTimerNearestSlot(void);
extern void ModelHandle_StopTimer(void);
extern void ModelHandle_StopSemiAuto(void);
extern void ModelHandle_FactoryReset(void);

/***************************************************************
 *  END OF COMPLETE SCREEN.C
 ***************************************************************/
