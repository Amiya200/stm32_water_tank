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
   UI ENUMS — ALL MODES + NEW TIMER / DEVICE MENUS
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

    /* Auto Mode (kept for future use) */
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

    /* Device Setup (Settings) – scroll menu + per-item editors */
    UI_DEVSET_MENU,          // Scrollable list inside Device Setup

    UI_SETTINGS_GAP,         // Set Dry Run
    UI_SETTINGS_RETRY,       // Set Testing Gap
    UI_SETTINGS_UV,          // Set Low Volt
    UI_SETTINGS_OV,          // Set High Volt
    UI_SETTINGS_OL,          // Over Load
    UI_SETTINGS_UL,          // Under Load
    UI_SETTINGS_MAXRUN,      // Set Max Run
    UI_SETTINGS_PWRREST,     // Power Restore
    UI_SETTINGS_FACTORY,     // Factory Reset (Yes/No)

    /* Date / Time / Day editors */
    UI_DEVSET_EDIT_DATE,
    UI_DEVSET_EDIT_TIME,
    UI_DEVSET_EDIT_DAY,

    /* Add New Device Submenus */
    UI_ADD_DEVICE_MENU,        // Pair / Remove
    UI_ADD_DEVICE_PAIR,        // Choose Wi-Fi/Receiver/Tx to pair
    UI_ADD_DEVICE_REMOVE,      // Choose Wi-Fi/Receiver/Tx to remove
    UI_ADD_DEVICE_PAIR_DONE,   // Paired OK
    UI_ADD_DEVICE_REMOVE_DONE, // Removed OK

    /* Reset confirm from main menu */
    UI_RESET_CONFIRM,

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

#define WELCOME_MS         2500
#define CURSOR_BLINK_MS     400
#define AUTO_BACK_MS      60000

#define LONG_PRESS_MS      3000
#define CONTINUOUS_STEP_MS  250   // smoother experience
#define COUNTDOWN_INC_MS   2000

/* Button press tracking */
static uint32_t sw_press_start[4] = {0,0,0,0};
static bool     sw_long_issued[4] = {false,false,false,false};

static uint32_t last_repeat_time = 0;

/* For reset confirm screen */
static bool reset_confirm_yes = false;

/* ================================================================
   EXTERNAL STATES (from model_handle / rtc_i2c)
   ================================================================ */
extern ADC_Data adcData;
extern TimerSlot timerSlots[5];
extern TwistSettings twistSettings;
extern RTC_Time_t time;   /* global RTC time struct from rtc_i2c.c */

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
static uint8_t edit_day_index = 0;   // 0..9

/* Slot gap time (min) */
static uint8_t edit_gap_min = 0;

/* Enable/Disable slot */
static bool edit_slot_enabled = true;

/* Which slot is being edited? 0..4, 5=Back */
static uint8_t currentSlot = 0;

/* Slot page: 0 → T1/T2, 1 → T3/T4, 2 → T5/BACK */
static uint8_t timer_page = 0;

/* ================================================================
   AUTO / TWIST / COUNTDOWN / SETTINGS TEMP VARS
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

/* SETTINGS (Device Setup) – core values that actually map to model_handle */
/* NOTE: edit_settings_gap_s now holds MINUTES (0=Disable, 1..15)  */
static uint16_t edit_settings_gap_s = 10;   // Dry run gap (min)
/* Testing gap / retry count now 0=Disable, 1..180min (UI side)    */
static uint8_t  edit_settings_retry = 3;
/* Low / High Volt: 0=Disable, else 150–200 / 250–300V             */
static uint16_t edit_settings_uv    = 180;  // Low volt
static uint16_t edit_settings_ov    = 260;  // High volt

/* Over/Under load: 0=Disable, else A (0.1 step) with clamping     */
static int edit_settings_ol = 6;       // Overload (A) change to int
static int edit_settings_ul = 0;       // Underload (A) change to int

/* Max run in minutes: 0=Disable, 10..300                          */
static uint16_t edit_settings_maxrun = 120;
static uint8_t  edit_settings_pwrrest = 0;   // 0=YES,1=NO,2=LAST
static bool     edit_settings_factory_yes = false;

/* ================================================================
   DATE / TIME / DAY EDIT (Device Setup)
   ================================================================ */
static uint8_t  edit_date_dd    = 1;
static uint8_t  edit_date_mm    = 1;
static uint16_t edit_date_yyyy  = 2025;
static uint8_t  edit_date_field = 0;  // 0=DD,1=MM,2=YY

static uint8_t  edit_time_hh    = 0;
static uint8_t  edit_time_min   = 0;
static uint8_t  edit_time_field = 0;  // 0=HH,1=MM

/* Day index 0..6 -> Sun..Sat */
static uint8_t  edit_day_idx2   = 0;
static const char* const dowNames[7] = {
    "Sun","Mon","Tue","Wed","Thu","Fri","Sat"
};

/* ================================================================
   ADD NEW DEVICE STATE
   ================================================================ */
static uint8_t addDevMenuIndex  = 0; // 0=Pair Device, 1=Remove Device
static uint8_t addDevTypeIndex  = 0; // 0=Wi-Fi, 1=Receiver, 2=Transmitter
static uint8_t lastAddDevType   = 0; // for "Paired/Removed <type>"
static bool    lastAddActionPair = true;

static const char* const addDevTypeNames[] = {
    "Wi-Fi",
    "Receiver",
    "Transmitter"
};

/* ================================================================
   MAIN MENU ITEMS (as per document)
   ================================================================ */
static const char* const main_menu[] = {
    "Timer Setting",     // 0
    "Add New Device",    // 1
    "Device Setup",      // 2
    "Reset To Default"   // 3
};
#define MAIN_MENU_COUNT 4

static uint8_t menu_idx      = 0;
static uint8_t menu_view_top = 0;

/* ================================================================
   DEVICE SETUP SCROLL MENU
   (Mapped to individual edit screens)
   ================================================================ */
static const char* const devset_menu_items[] = {
    "Set Dry Run",     // -> UI_SETTINGS_GAP
    "Set Testing Gap", // -> UI_SETTINGS_RETRY
    "Set Low Volt",    // -> UI_SETTINGS_UV
    "Set High Volt",   // -> UI_SETTINGS_OV
    "Set Over Load",   // -> UI_SETTINGS_OL
    "Set Under Load",  // -> UI_SETTINGS_UL
    "Set Max Run",     // -> UI_SETTINGS_MAXRUN
    "Set Date",        // -> UI_DEVSET_EDIT_DATE
    "Set Time",        // -> UI_DEVSET_EDIT_TIME
    "Set Day",         // -> UI_DEVSET_EDIT_DAY
    "Power Restore",   // -> UI_SETTINGS_PWRREST
    "Factory Reset",   // -> UI_SETTINGS_FACTORY
    "Back"             // -> Back to Main Menu
};
#define DEVSET_MENU_COUNT  (sizeof(devset_menu_items)/sizeof(devset_menu_items[0]))

static uint8_t devset_idx      = 0;
static uint8_t devset_view_top = 0;

/* ================================================================ */

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
   UI DRAWING FUNCTIONS
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

    if (menu_view_top + 1 < MAIN_MENU_COUNT)
        snprintf(l1, sizeof(l1), " %-15.15s", main_menu[menu_view_top + 1]);
    else
        snprintf(l1, sizeof(l1), "                ");

    lcd_line0(l0);
    lcd_line1(l1);

    draw_menu_cursor();
}

/***************************************************************
 *  DEVICE SETUP MENU DRAW (scrolling cursor)
 ***************************************************************/
static void show_devset_menu(void)
{
    char l0[17], l1[17];

    /* Adjust top-of-view */
    if (devset_idx < devset_view_top)
        devset_view_top = devset_idx;
    else if (devset_idx > devset_view_top + 1)
        devset_view_top = devset_idx - 1;

    /* Line 0 */
    uint8_t idx0 = devset_view_top;
    uint8_t idx1 = devset_view_top + 1;

    if (idx0 < DEVSET_MENU_COUNT)
        snprintf(l0, sizeof(l0), "%c%-15.15s",
                 (devset_idx == idx0 ? '>' : ' '),
                 devset_menu_items[idx0]);
    else
        snprintf(l0, sizeof(l0), "                ");

    if (idx1 < DEVSET_MENU_COUNT)
        snprintf(l1, sizeof(l1), "%c%-15.15s",
                 (devset_idx == idx1 ? '>' : ' '),
                 devset_menu_items[idx1]);
    else
        snprintf(l1, sizeof(l1), "                ");

    lcd_line0(l0);
    lcd_line1(l1);
}

/***************************************************************
 *  TIMER MODE — TIMER SELECT
 ***************************************************************/
static void show_timer_slot_select(void)
{
    lcd_clear();

    /* Map page → timer indexes */
    int item1 = timer_page * 2;
    int item2 = item1 + 1;

    char l0[17], l1[17];

    /* Line0 */
    if (item1 == 5) {
        snprintf(l0, sizeof(l0), "%c Back",
                 (currentSlot == 5 ? '>' : ' '));
    }
    else {
        snprintf(l0, sizeof(l0), "%c Timer %d",
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
            snprintf(l1, sizeof(l1), "%c Timer %d",
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
    char title[17];
    snprintf(title, sizeof(title), "T%u On Time", (unsigned)(currentSlot + 1));
    lcd_line0(title);

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
    char title[17];
    snprintf(title, sizeof(title), "T%u Off Time", (unsigned)(currentSlot + 1));
    lcd_line0(title);

    char buf[17];
    if (time_edit_field == 0)
        snprintf(buf, sizeof(buf), "[%02d]:%02d   Next>", edit_off_h, edit_off_m);
    else
        snprintf(buf, sizeof(buf), "%02d:[%02d]   Next>", edit_off_h, edit_off_m);

    lcd_line1(buf);
}

/***************************************************************
 *  TIMER — DAYS MENU
 ***************************************************************/
static const char* dayNames[] = {
    "Monday", "Tuesday", "Wed", "Thu", "Friday", "Sat", "Sun",
    "Enable All", "Disable All", "Next>"
};

static void show_timer_days(void)
{
    lcd_line0("Timer Days");

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
    lcd_line0("Timer Gap (min)");

    char buf[17];
    snprintf(buf, sizeof(buf), ">T%u %3u min Next>",
             (unsigned)(currentSlot + 1),
             (unsigned)edit_gap_min);

    lcd_line1(buf);
}

/***************************************************************
 *  TIMER — ENABLE / DISABLE
 ***************************************************************/
static void show_timer_enable(void)
{
    char title[17];
    snprintf(title, sizeof(title), "T%u Enable?",
             (unsigned)(currentSlot + 1));
    lcd_line0(title);
    lcd_line1(edit_slot_enabled ? "YES       Next>" :
                                  "NO        Next>");
}

/***************************************************************
 *  TIMER — SUMMARY
 ***************************************************************/
static void show_timer_summary(void)
{
    char title[17];
    snprintf(title, sizeof(title), "T%u Summary",
             (unsigned)(currentSlot + 1));
    lcd_line0(title);

    if (edit_slot_enabled)
        lcd_line1("Enabled     Next>");
    else
        lcd_line1("Disabled    Next>");
}

/***************************************************************
 *  AUTO MODE (kept)
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
 *  DEVICE SETUP (SETTINGS) UI
 ***************************************************************/
/* Dry Run: 0=Disable, 1..15 min */
static void show_settings_gap(void){
    lcd_line0("Set Dry Run");
    char buf[17];
    if (edit_settings_gap_s == 0)
        snprintf(buf,sizeof(buf),"Disable    Next>");
    else
        snprintf(buf,sizeof(buf),"val:%2umin Next>", edit_settings_gap_s);
    lcd_line1(buf);
}

/* Testing Gap: 0=Disable, 1..180min */
static void show_settings_retry(void){
    lcd_line0("Testing Gap");
    char buf[17];
    if (edit_settings_retry == 0)
        snprintf(buf,sizeof(buf),"Disable    Next>");
    else
        snprintf(buf,sizeof(buf),"val:%3umin Next>", edit_settings_retry);
    lcd_line1(buf);
}

/* Low Volt: 0=Disable, else 150–200V */
static void show_settings_uv(void){
    lcd_line0("Set Low Volt");
    char buf[17];
    if (edit_settings_uv == 0)
        snprintf(buf,sizeof(buf),"Disable    Next>");
    else
        snprintf(buf,sizeof(buf),"val:%3uV Next>", edit_settings_uv);
    lcd_line1(buf);
}

/* High Volt: 0=Disable, else 250–300V */
static void show_settings_ov(void){
    lcd_line0("Set High Volt");
    char buf[17];
    if (edit_settings_ov == 0)
        snprintf(buf,sizeof(buf),"Disable    Next>");
    else
        snprintf(buf,sizeof(buf),"val:%3uV Next>", edit_settings_ov);
    lcd_line1(buf);
}

/* Over Load: 0=Disable, else up to 25A */
/* Over Load: 0=Disable, else up to 25A */
static void show_settings_ol(void){
    lcd_line0("Over Load (A)");
    char buf[17];
    if (edit_settings_ol < 1)
        snprintf(buf, sizeof(buf), "Disable    Next>");
    else
        snprintf(buf, sizeof(buf), "val:%3d Next>", edit_settings_ol);
    lcd_line1(buf);
}
/* Under Load: 0=Disable, else up to 10A */
/* Under Load: 0=Disable, else up to 10A */
static void show_settings_ul(void){
    lcd_line0("Under Load (A)");
    char buf[17];
    if (edit_settings_ul < 1)
        snprintf(buf, sizeof(buf), "Disable    Next>");
    else
        snprintf(buf, sizeof(buf), "val:%3d Next>", edit_settings_ul);
    lcd_line1(buf);
}

/* Max Run: 0=Disable, else 10–300min */
static void show_settings_maxrun(void){
    lcd_line0("Set Max Run");
    char buf[17];
    if (edit_settings_maxrun == 0)
        snprintf(buf,sizeof(buf),"Disable    Next>");
    else
        snprintf(buf,sizeof(buf),"val:%3umin Next>",edit_settings_maxrun);
    lcd_line1(buf);
}

/* Power Restore: YES / NO / LAST */
static void show_settings_pwrrest(void){
    lcd_line0("Power Restore");
    const char* text =
        (edit_settings_pwrrest == 0) ? "YES       Next>" :
        (edit_settings_pwrrest == 1) ? "NO        Next>" :
                                       "LAST      Next>";
    lcd_line1(text);
}

static void show_settings_factory(void){
    lcd_line0("Factory Reset?");
    lcd_line1(edit_settings_factory_yes ? "YES       Next>" :
                                         "NO        Next>");
}

/***************************************************************
 *  DEVICE SETUP — DATE/TIME/DAY SCREENS
 ***************************************************************/
static void show_devset_edit_date(void)
{
    lcd_line0("Set Date");

    uint8_t yy2 = (uint8_t)(edit_date_yyyy % 100);
    char buf[17];

    if (edit_date_field == 0)
        snprintf(buf,sizeof(buf),"[%02u]-%02u-%02u", edit_date_dd, edit_date_mm, yy2);
    else if (edit_date_field == 1)
        snprintf(buf,sizeof(buf),"%02u-[%02u]-%02u", edit_date_dd, edit_date_mm, yy2);
    else
        snprintf(buf,sizeof(buf),"%02u-%02u-[%02u]", edit_date_dd, edit_date_mm, yy2);

    lcd_line1(buf);
}

static void show_devset_edit_time(void)
{
    lcd_line0("Set Time");
    char buf[17];

    if (edit_time_field == 0)
        snprintf(buf,sizeof(buf),"[%02u]:%02u", edit_time_hh, edit_time_min);
    else
        snprintf(buf,sizeof(buf),"%02u:[%02u]", edit_time_hh, edit_time_min);

    lcd_line1(buf);
}

static void show_devset_edit_day(void)
{
    lcd_line0("Set Day");
    char buf[17];
    snprintf(buf,sizeof(buf),"> %s", dowNames[edit_day_idx2 % 7]);
    lcd_line1(buf);
}

/***************************************************************
 *  ADD NEW DEVICE — UI
 ***************************************************************/
static void show_add_device_menu(void)
{
    lcd_line0(addDevMenuIndex == 0 ? ">Pair Device" : " Pair Device");
    lcd_line1(addDevMenuIndex == 1 ? ">Remove Device" : " Remove Device");
}

static void show_add_device_pair(void)
{
    lcd_line0("Pair Device");
    char buf[17];
    snprintf(buf,sizeof(buf),">%s", addDevTypeNames[addDevTypeIndex]);
    lcd_line1(buf);
}

static void show_add_device_remove(void)
{
    lcd_line0("Remove Device");
    char buf[17];
    snprintf(buf,sizeof(buf),">%s", addDevTypeNames[addDevTypeIndex]);
    lcd_line1(buf);
}

static void show_add_device_pair_done(void)
{
    lcd_line0("Paired Device");
    char buf[17];
    snprintf(buf,sizeof(buf),"%s   OK>", addDevTypeNames[lastAddDevType]);
    lcd_line1(buf);
}

static void show_add_device_remove_done(void)
{
    lcd_line0("Removed Device");
    char buf[17];
    snprintf(buf,sizeof(buf),"%s   OK>", addDevTypeNames[lastAddDevType]);
    lcd_line1(buf);
}

/***************************************************************
 *  RESET TO DEFAULT — CONFIRM
 ***************************************************************/
static void show_reset_confirm(void)
{
    lcd_line0("Reset To Default?");
    lcd_line1(reset_confirm_yes ? "YES       Apply>" :
                                  "NO        Back>");
}

/* ================================================================
   APPLY FUNCTIONS
   ================================================================ */
static void apply_timer_slot(void)
{
    TimerSlot *t = &timerSlots[currentSlot];

    t->onHour    = edit_on_h;
    t->onMinute  = edit_on_m;
    t->offHour   = edit_off_h;
    t->offMinute = edit_off_m;
    t->dayMask   = edit_day_mask;
    t->gapMinutes = edit_gap_min;
    t->enabled = edit_slot_enabled;

    extern void ModelHandle_TimerRecalculateNow(void);
    ModelHandle_TimerRecalculateNow();
}

static void apply_auto_settings(void)
{
    ModelHandle_SetAutoSettings(
        edit_auto_gap_s,
        edit_auto_maxrun_min,
        edit_auto_retry
    );
}

/* Apply current Device Setup core settings into model */
static void apply_settings_core(void)
{
    /* Convert UI dry-run minutes to seconds; 0 = disabled */
    uint16_t gap_s = 0;
    if (edit_settings_gap_s > 0)
        gap_s = (uint16_t)(edit_settings_gap_s * 60U);

    ModelHandle_SetUserSettings(
        gap_s,
        edit_settings_retry,
        edit_settings_uv,
        edit_settings_ov,
        edit_settings_ol,
        edit_settings_ul,
        edit_settings_maxrun
    );
}

/***************************************************************
 *  SETTINGS FLOW — START (Device Setup)
 *  Load all values + RTC into local edit variables and go to
 *  Device Setup scroll menu.
 ***************************************************************/
static void start_settings_edit_flow(void)
{
    /* Load fresh settings from model */
    uint16_t gap_s = ModelHandle_GetGapTime();
    if (gap_s == 0) {
        edit_settings_gap_s = 0;
    } else {
        edit_settings_gap_s = gap_s / 60;
        if (edit_settings_gap_s < 1)  edit_settings_gap_s = 1;
        if (edit_settings_gap_s > 15) edit_settings_gap_s = 15;
    }

    edit_settings_retry   = ModelHandle_GetRetryCount();
    if (edit_settings_retry > 180)
        edit_settings_retry = 180;

    edit_settings_uv      = ModelHandle_GetUnderVolt();
    if (edit_settings_uv != 0) {    // 0=Disable
        if (edit_settings_uv < 150) edit_settings_uv = 150;
        if (edit_settings_uv > 200) edit_settings_uv = 200;
    }

    edit_settings_ov      = ModelHandle_GetOverVolt();
    if (edit_settings_ov != 0) {    // 0=Disable
        if (edit_settings_ov < 250) edit_settings_ov = 250;
        if (edit_settings_ov > 300) edit_settings_ov = 300;
    }

    edit_settings_ol      = ModelHandle_GetOverloadLimit();
    if (edit_settings_ol > 25.0f) edit_settings_ol = 25.0f;

    edit_settings_ul      = ModelHandle_GetUnderloadLimit();
    if (edit_settings_ul > 10.0f) edit_settings_ul = 10.0f;

    edit_settings_maxrun  = ModelHandle_GetMaxRunTime();
    if (edit_settings_maxrun > 300) edit_settings_maxrun = 300;

    /* Power Restore mode from model 0=YES,1=NO,2=LAST */
    edit_settings_pwrrest = ModelHandle_GetPowerRestoreMode();

    edit_settings_factory_yes = 0;

    /* Load current RTC date/time/day */
    RTC_GetTimeDate();

    edit_date_dd    = time.dom;
    edit_date_mm    = time.month;
    edit_date_yyyy  = time.year;
    edit_date_field = 0;

    edit_time_hh    = time.hour;
    edit_time_min   = time.min;
    edit_time_field = 0;

    /* DS1307 dow: 1..7; map to 0..6 index (Sun..Sat) */
    if (time.dow >= 1 && time.dow <= 7)
        edit_day_idx2 = (uint8_t)((time.dow - 1) % 7);
    else
        edit_day_idx2 = 0;

    devset_idx      = 0;
    devset_view_top = 0;

    ui = UI_DEVSET_MENU;
    screenNeedsRefresh = true;
}

/***************************************************************
 *  MENU SELECT — MAIN NAVIGATION + TIMER / SETTINGS
 ***************************************************************/
static void goto_menu_top(void)
{
    menu_idx = 0;
    menu_view_top = 0;
}

static void menu_select(void)
{
    refreshInactivityTimer();

    /* WELCOME → DASH */
    if (ui == UI_WELCOME)
    {
        ui = UI_DASH;
        screenNeedsRefresh = true;
        return;
    }

    /* DASH → MENU (also handled via long SELECT) */
    if (ui == UI_DASH)
    {
        goto_menu_top();
        ui = UI_MENU;
        screenNeedsRefresh = true;
        return;
    }

    /* MAIN MENU */
    if (ui == UI_MENU)
    {
        switch(menu_idx)
        {
            case 0:   /* TIMER SETTING */
                currentSlot = 0;
                timer_page  = 0;
                ui = UI_TIMER_SLOT_SELECT;
                screenNeedsRefresh = true;
                return;

            case 1:   /* ADD NEW DEVICE */
                addDevMenuIndex = 0;
                addDevTypeIndex = 0;
                ui = UI_ADD_DEVICE_MENU;
                screenNeedsRefresh = true;
                return;

            case 2:   /* DEVICE SETUP */
                start_settings_edit_flow();  /* sets ui = UI_DEVSET_MENU */
                return;

            case 3:   /* RESET TO DEFAULT CONFIRM */
                reset_confirm_yes = false;
                ui = UI_RESET_CONFIRM;
                screenNeedsRefresh = true;
                return;
        }
    }

    /* TIMER SLOT SELECT → LOAD TIMER */
    if (ui == UI_TIMER_SLOT_SELECT)
    {
        if (currentSlot == 5)
        {
            ui = UI_MENU;  // BACK
            screenNeedsRefresh = true;
            return;
        }

        TimerSlot *t = &timerSlots[currentSlot];

        edit_on_h  = t->onHour;
        edit_on_m  = t->onMinute;
        edit_off_h = t->offHour;
        edit_off_m = t->offMinute;

        edit_day_mask = t->dayMask;
        edit_gap_min  = t->gapMinutes;
        edit_slot_enabled = t->enabled;

        time_edit_field = 0;
        edit_day_index  = 0;

        ui = UI_TIMER_EDIT_ON_TIME;
        screenNeedsRefresh = true;
        return;
    }

    /* TIMER EDIT FLOW SEQUENCE */
    switch(ui)
    {
        case UI_TIMER_EDIT_ON_TIME:
            if (time_edit_field == 0)
                time_edit_field = 1;
            else {
                time_edit_field = 0;
                ui = UI_TIMER_EDIT_OFF_TIME;
            }
            break;

        case UI_TIMER_EDIT_OFF_TIME:
            if (time_edit_field == 0)
                time_edit_field = 1;
            else {
                time_edit_field = 0;
                ui = UI_TIMER_EDIT_DAYS;
            }
            break;

        case UI_TIMER_EDIT_DAYS:
            if (edit_day_index == 9)
                ui = UI_TIMER_EDIT_GAP;
            else
                edit_day_index++;
            break;

        case UI_TIMER_EDIT_GAP:
            ui = UI_TIMER_EDIT_ENABLE;
            break;

        case UI_TIMER_EDIT_ENABLE:
            ui = UI_TIMER_EDIT_SUMMARY;
            break;

        case UI_TIMER_EDIT_SUMMARY:
            apply_timer_slot();
            ui = UI_TIMER_SLOT_SELECT;   /* after summary → back to slot select */
            break;

        default:
            break;
    }

    screenNeedsRefresh = true;
}

/* ================================================================
   VALUE EDIT ENGINE
   ================================================================ */
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

        /* SETTINGS / DEVICE SETUP */
        /* Dry run gap: 0..15 min */
        case UI_SETTINGS_GAP:
            if (edit_settings_gap_s < 15) edit_settings_gap_s++;
            break;

        /* Testing Gap: 0..180min */
        case UI_SETTINGS_RETRY:
            if (edit_settings_retry < 180) edit_settings_retry++;
            break;

        /* Low Volt: 0=Disable, else 150..200V */
        case UI_SETTINGS_UV:
            if (edit_settings_uv == 0) edit_settings_uv = 150;
            else if (edit_settings_uv < 200) edit_settings_uv++;
            break;

        /* High Volt: 0=Disable, else 250..300V */
        case UI_SETTINGS_OV:
            if (edit_settings_ov == 0) edit_settings_ov = 250;
            else if (edit_settings_ov < 300) edit_settings_ov++;
            break;

        /* Over Load: 0..25A (0=Disable) */
        case UI_SETTINGS_OL:
        	if (edit_settings_ol < 25) edit_settings_ol++;  // Increment by 1
        	break;

        case UI_SETTINGS_UL:
        	if (edit_settings_ul < 10) edit_settings_ul++;  // Increment by 1
        	break;

        /* Max Run: 0=Disable, else 10..300min */
        case UI_SETTINGS_MAXRUN:
            if (edit_settings_maxrun == 0) edit_settings_maxrun = 10;
            else if (edit_settings_maxrun < 300) edit_settings_maxrun++;
            break;

        /* Power Restore: 0=YES,1=NO,2=LAST */
        case UI_SETTINGS_PWRREST:
            if (edit_settings_pwrrest < 2) edit_settings_pwrrest++;
            else edit_settings_pwrrest = 0;
            break;

        case UI_SETTINGS_FACTORY:
            edit_settings_factory_yes ^= 1;
            break;

        /* DEVSET DATE/TIME/DAY */
        case UI_DEVSET_EDIT_DATE:
            if (edit_date_field == 0) {
                if (edit_date_dd < 31) edit_date_dd++;
            } else if (edit_date_field == 1) {
                if (edit_date_mm < 12) edit_date_mm++;
            } else {
                if (edit_date_yyyy < 2099) edit_date_yyyy++;
            }
            break;

        case UI_DEVSET_EDIT_TIME:
            if (edit_time_field == 0) {
                if (edit_time_hh < 23) edit_time_hh++;
            } else {
                if (edit_time_min < 59) edit_time_min++;
            }
            break;

        case UI_DEVSET_EDIT_DAY:
            edit_day_idx2 = (uint8_t)((edit_day_idx2 + 1) % 7);
            break;

        default:
            break;
    }
}

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

        /* SETTINGS / DEVICE SETUP */
        /* Dry run gap: 0..15, 0=Disable */
        case UI_SETTINGS_GAP:
            if(edit_settings_gap_s > 0) edit_settings_gap_s--;
            break;

        /* Testing Gap: 0..180 */
        case UI_SETTINGS_RETRY:
            if(edit_settings_retry > 0) edit_settings_retry--;
            break;

        /* Low Volt: 0=Disable, else down to 150 */
        case UI_SETTINGS_UV:
            if (edit_settings_uv > 150) edit_settings_uv--;
            else if (edit_settings_uv == 150) edit_settings_uv = 0;
            break;

        /* High Volt: 0=Disable, else down to 250 */
        case UI_SETTINGS_OV:
            if (edit_settings_ov > 250) edit_settings_ov--;
            else if (edit_settings_ov == 250) edit_settings_ov = 0;
            break;

        case UI_SETTINGS_OL:
        	if (edit_settings_ol > 0) edit_settings_ol--;  // Decrement by 1
        	break;
        case UI_SETTINGS_UL:
        	if (edit_settings_ul > 0) edit_settings_ul--;  // Decrement by 1
        	break;

        /* Max Run: 0=Disable, else 10..300 */
        case UI_SETTINGS_MAXRUN:
            if(edit_settings_maxrun > 10) edit_settings_maxrun--;
            else if (edit_settings_maxrun == 10) edit_settings_maxrun = 0;
            break;

        /* Power Restore: 0=YES,1=NO,2=LAST */
        case UI_SETTINGS_PWRREST:
            if (edit_settings_pwrrest == 0) edit_settings_pwrrest = 2;
            else edit_settings_pwrrest--;
            break;

        case UI_SETTINGS_FACTORY:
            edit_settings_factory_yes ^= 1;
            break;

        /* DEVSET DATE/TIME/DAY */
        case UI_DEVSET_EDIT_DATE:
            if (edit_date_field == 0) {
                if (edit_date_dd > 1) edit_date_dd--;
            } else if (edit_date_field == 1) {
                if (edit_date_mm > 1) edit_date_mm--;
            } else {
                if (edit_date_yyyy > 2020) edit_date_yyyy--;
            }
            break;

        case UI_DEVSET_EDIT_TIME:
            if (edit_time_field == 0) {
                if (edit_time_hh > 0) edit_time_hh--;
            } else {
                if (edit_time_min > 0) edit_time_min--;
            }
            break;

        case UI_DEVSET_EDIT_DAY:
            edit_day_idx2 = (uint8_t)((edit_day_idx2 + 6) % 7);
            break;

        default:
            break;
    }
}

/***************************************************************
 *  BUTTON DECODER
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
 *  MAIN SWITCH HANDLER
 ***************************************************************/
/***************************************************************
 *  MAIN SWITCH HANDLER
 ***************************************************************/
void Screen_HandleSwitches(void)
{
    static bool prev_sw_down_edit = false;   /* track DOWN state in countdown edit */

    UiButton b = decode_button_press();
    uint32_t now = HAL_GetTick();

    bool sw_up   = Switch_IsPressed(2);
    bool sw_down = Switch_IsPressed(3);

    /* LONG PRESS RESET → manual toggle */
    if (b == BTN_RESET_LONG)
    {
        ModelHandle_ToggleManual();
        ui = UI_DASH;
        screenNeedsRefresh = true;
        return;
    }

    /* CONTINUOUS UP (HOLD) */
    if (sw_up && sw_long_issued[2] && ui != UI_COUNTDOWN_EDIT_MIN)
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
            else if (ui == UI_DEVSET_MENU)
            {
                if (devset_idx > 0) devset_idx--;
            }
            else
            {
                /* Any edit screen (excluding countdown edit) */
                increase_edit_value();
            }

            screenNeedsRefresh = true;
        }
    }

    /* CONTINUOUS DOWN (HOLD) */
    if (sw_down && sw_long_issued[3] && ui != UI_COUNTDOWN_EDIT_MIN)
    {
        if (now - last_repeat_time >= CONTINUOUS_STEP_MS)
        {
            last_repeat_time = now;

            if (ui == UI_MENU)
            {
                if (menu_idx < MAIN_MENU_COUNT - 1) menu_idx++;
            }
            else if (ui == UI_TIMER_SLOT_SELECT)
            {
                if (currentSlot < 5) currentSlot++;
                timer_page = (currentSlot < 2 ? 0 :
                             (currentSlot < 5 ? 1 : 2));
            }
            else if (ui == UI_DEVSET_MENU)
            {
                if (devset_idx < DEVSET_MENU_COUNT - 1) devset_idx++;
            }
            else
            {
                /* Any edit screen (excluding countdown edit) */
                decrease_edit_value();
            }

            screenNeedsRefresh = true;
        }
    }

    /* ============================================
       COUNTDOWN EDIT: single DOWN button, press & hold
       - Entered via BTN_DOWN_LONG from DASH.
       - While DOWN is held: increase value periodically.
       - On release of DOWN: exit back to DASH.
       ============================================ */
    if (ui == UI_COUNTDOWN_EDIT_MIN)
    {
        if (sw_down)
        {
            /* first press → immediate step + start repeat timer */
            if (!prev_sw_down_edit)
            {
                last_repeat_time = now;
                increase_edit_value();
                screenNeedsRefresh = true;
            }
            else if (now - last_repeat_time >= CONTINUOUS_STEP_MS)
            {
                /* continuous increase while holding */
                last_repeat_time = now;
                increase_edit_value();
                screenNeedsRefresh = true;
            }
        }

        /* detect release → leave edit mode */
        if (!sw_down && prev_sw_down_edit)
        {
            ui = UI_DASH;
            screenNeedsRefresh = true;
        }

        prev_sw_down_edit = sw_down;
        return;
    }
    else
    {
        /* not in countdown edit → reset tracking flag */
        prev_sw_down_edit = false;
    }

    if (b == BTN_NONE) return;
    refreshInactivityTimer();

    /* ================================
       GENERIC EDITING MODES
       ================================ */
    bool editing =
        (ui == UI_TIMER_EDIT_ON_TIME ||
         ui == UI_TIMER_EDIT_OFF_TIME ||
         ui == UI_TIMER_EDIT_GAP ||
         ui == UI_AUTO_EDIT_GAP ||
         ui == UI_AUTO_EDIT_MAXRUN ||
         ui == UI_AUTO_EDIT_RETRY ||
         ui == UI_TWIST_EDIT_ON ||
         ui == UI_TWIST_EDIT_OFF ||
         ui == UI_TWIST_EDIT_ON_H ||
         ui == UI_TWIST_EDIT_ON_M ||
         ui == UI_TWIST_EDIT_OFF_H ||
         ui == UI_TWIST_EDIT_OFF_M ||
         /* NOTE: UI_COUNTDOWN_EDIT_MIN is handled separately above */
         (ui >= UI_SETTINGS_GAP && ui <= UI_SETTINGS_FACTORY) ||
         ui == UI_DEVSET_EDIT_DATE ||
         ui == UI_DEVSET_EDIT_TIME ||
         ui == UI_DEVSET_EDIT_DAY);

    if (editing)
    {
        switch (b)
        {
            case BTN_UP:
                increase_edit_value();
                break;

            case BTN_DOWN:
                decrease_edit_value();
                break;

            case BTN_UP_LONG:
                last_repeat_time = now;
                increase_edit_value();
                break;

            case BTN_DOWN_LONG:
                last_repeat_time = now;
                decrease_edit_value();
                break;

            case BTN_SELECT:

                /* TIMER editing uses linear flow via menu_select() */
                if (ui == UI_TIMER_EDIT_ON_TIME ||
                    ui == UI_TIMER_EDIT_OFF_TIME ||
                    ui == UI_TIMER_EDIT_GAP)
                {
                    menu_select();
                }
                /* SETTINGS from Device Setup: apply and go back to DEVSET menu */
                else if (ui >= UI_SETTINGS_GAP && ui <= UI_SETTINGS_MAXRUN)
                {
                    apply_settings_core();
                    ui = UI_DEVSET_MENU;
                }
                else if (ui == UI_SETTINGS_PWRREST)
                {
                    ModelHandle_SetPowerRestoreMode(edit_settings_pwrrest);
                    ui = UI_DEVSET_MENU;
                }
                else if (ui == UI_SETTINGS_FACTORY)
                {
                    if (edit_settings_factory_yes)
                        ModelHandle_FactoryReset();
                    ui = UI_DEVSET_MENU;
                }
                /* Device setup: Set Date */
                else if (ui == UI_DEVSET_EDIT_DATE)
                {
                    if (edit_date_field < 2)
                    {
                        edit_date_field++;  // DD -> MM -> YY
                    }
                    else
                    {
                        /* Finished editing → write to RTC */
                        RTC_GetTimeDate();

                        uint8_t sec   = time.sec;
                        uint8_t min   = time.min;
                        uint8_t hour  = time.hour;
                        uint8_t dow   = time.dow;
                        uint8_t dom   = edit_date_dd;
                        uint8_t month = edit_date_mm;
                        uint16_t year = edit_date_yyyy;

                        RTC_SetTimeDate(sec, min, hour, dow, dom, month, year);
                        RTC_GetTimeDate();

                        edit_date_field = 0;
                        ui = UI_DEVSET_MENU;
                    }
                }
                /* Device setup: Set Time */
                else if (ui == UI_DEVSET_EDIT_TIME)
                {
                    if (edit_time_field == 0)
                    {
                        edit_time_field = 1;   // HH -> MM
                    }
                    else
                    {
                        RTC_GetTimeDate();

                        uint8_t sec   = time.sec;
                        uint8_t dow   = time.dow;
                        uint8_t dom   = time.dom;
                        uint8_t month = time.month;
                        uint16_t year = time.year;

                        RTC_SetTimeDate(sec,
                                        edit_time_min,
                                        edit_time_hh,
                                        dow, dom, month, year);

                        RTC_GetTimeDate();

                        edit_time_field = 0;
                        ui = UI_DEVSET_MENU;
                    }
                }
                /* Device setup: Set Day */
                else if (ui == UI_DEVSET_EDIT_DAY)
                {
                    RTC_GetTimeDate();

                    uint8_t sec   = time.sec;
                    uint8_t min   = time.min;
                    uint8_t hour  = time.hour;
                    uint8_t dom   = time.dom;
                    uint8_t month = time.month;
                    uint16_t year = time.year;

                    /* DS1307 DOW 1..7 */
                    uint8_t dow   = (uint8_t)((edit_day_idx2 % 7) + 1);

                    RTC_SetTimeDate(sec, min, hour, dow, dom, month, year);
                    RTC_GetTimeDate();

                    ui = UI_DEVSET_MENU;
                }
                else
                {
                    /* Fallback: just go back to Device Setup menu */
                    ui = UI_DEVSET_MENU;
                }
                break;

            case BTN_RESET:
                if (ui >= UI_SETTINGS_GAP && ui <= UI_SETTINGS_FACTORY)
                {
                    ui = UI_DEVSET_MENU;
                }
                else if (ui == UI_DEVSET_EDIT_DATE ||
                         ui == UI_DEVSET_EDIT_TIME ||
                         ui == UI_DEVSET_EDIT_DAY)
                {
                    ui = UI_DEVSET_MENU;
                }
                else
                {
                    ui = UI_MENU;
                }
                break;

            default:
                break;
        }

        screenNeedsRefresh = true;
        return;
    }

    /* ===============================
       TIMER DAYS SELECTION
       =============================== */
    if (ui == UI_TIMER_EDIT_DAYS)
    {
        switch (b)
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

            default:
                break;
        }

        screenNeedsRefresh = true;
        return;
    }

    /* ===============================
       TIMER ENABLE / DISABLE
       =============================== */
    if (ui == UI_TIMER_EDIT_ENABLE)
    {
        switch (b)
        {
            case BTN_UP:
            case BTN_UP_LONG:
                edit_slot_enabled = true;
                break;

            case BTN_DOWN:
            case BTN_DOWN_LONG:
                edit_slot_enabled = false;
                break;

            case BTN_SELECT:
                ui = UI_TIMER_EDIT_SUMMARY;
                break;

            case BTN_RESET:
                ui = UI_TIMER_SLOT_SELECT;
                break;

            default:
                break;
        }

        screenNeedsRefresh = true;
        return;
    }

    /* ===============================
       ADD NEW DEVICE — MAIN MENU
       =============================== */
    if (ui == UI_ADD_DEVICE_MENU)
    {
        switch (b)
        {
            case BTN_UP:
            case BTN_UP_LONG:
                if (addDevMenuIndex > 0) addDevMenuIndex--;
                break;

            case BTN_DOWN:
            case BTN_DOWN_LONG:
                if (addDevMenuIndex < 1) addDevMenuIndex++;
                break;

            case BTN_SELECT:
            case BTN_SELECT_LONG:
                addDevTypeIndex = 0;
                if (addDevMenuIndex == 0)
                    ui = UI_ADD_DEVICE_PAIR;
                else
                    ui = UI_ADD_DEVICE_REMOVE;
                break;

            case BTN_RESET:
                ui = UI_MENU;
                break;

            default:
                break;
        }

        screenNeedsRefresh = true;
        return;
    }

    /* ADD NEW DEVICE — PAIR */
    if (ui == UI_ADD_DEVICE_PAIR)
    {
        switch (b)
        {
            case BTN_UP:
            case BTN_UP_LONG:
                if (addDevTypeIndex > 0) addDevTypeIndex--;
                break;

            case BTN_DOWN:
            case BTN_DOWN_LONG:
                if (addDevTypeIndex < 2) addDevTypeIndex++;
                break;

            case BTN_SELECT:
            case BTN_SELECT_LONG:
                lastAddActionPair = true;
                lastAddDevType = addDevTypeIndex;
                ui = UI_ADD_DEVICE_PAIR_DONE;
                break;

            case BTN_RESET:
                ui = UI_ADD_DEVICE_MENU;
                break;

            default:
                break;
        }

        screenNeedsRefresh = true;
        return;
    }

    /* ADD NEW DEVICE — REMOVE */
    if (ui == UI_ADD_DEVICE_REMOVE)
    {
        switch (b)
        {
            case BTN_UP:
            case BTN_UP_LONG:
                if (addDevTypeIndex > 0) addDevTypeIndex--;
                break;

            case BTN_DOWN:
            case BTN_DOWN_LONG:
                if (addDevTypeIndex < 2) addDevTypeIndex++;
                break;

            case BTN_SELECT:
            case BTN_SELECT_LONG:
                lastAddActionPair = false;
                lastAddDevType = addDevTypeIndex;
                ui = UI_ADD_DEVICE_REMOVE_DONE;
                break;

            case BTN_RESET:
                ui = UI_ADD_DEVICE_MENU;
                break;

            default:
                break;
        }

        screenNeedsRefresh = true;
        return;
    }

    /* ADD NEW DEVICE — DONE SCREENS */
    if (ui == UI_ADD_DEVICE_PAIR_DONE || ui == UI_ADD_DEVICE_REMOVE_DONE)
    {
        switch (b)
        {
            case BTN_SELECT:
            case BTN_SELECT_LONG:
            case BTN_RESET:
                ui = UI_ADD_DEVICE_MENU;
                break;
            default:
                break;
        }
        screenNeedsRefresh = true;
        return;
    }

    /* ===============================
       RESET TO DEFAULT CONFIRM
       =============================== */
    if (ui == UI_RESET_CONFIRM)
    {
        switch (b)
        {
            case BTN_UP:
            case BTN_UP_LONG:
                reset_confirm_yes = true;
                break;

            case BTN_DOWN:
            case BTN_DOWN_LONG:
                reset_confirm_yes = false;
                break;

            case BTN_SELECT:
            case BTN_SELECT_LONG:
                if (reset_confirm_yes)
                {
                    ModelHandle_FactoryReset();
                }
                ui = UI_DASH;
                break;

            case BTN_RESET:
                ui = UI_MENU;
                break;

            default:
                break;
        }

        screenNeedsRefresh = true;
        return;
    }

    /* ===============================
       DEVICE SETUP MENU (scroll)
       =============================== */
    if (ui == UI_DEVSET_MENU)
    {
        switch (b)
        {
            case BTN_UP:
            case BTN_UP_LONG:
                if (devset_idx > 0) devset_idx--;
                break;

            case BTN_DOWN:
            case BTN_DOWN_LONG:
                if (devset_idx < DEVSET_MENU_COUNT - 1) devset_idx++;
                break;

            case BTN_SELECT:
            case BTN_SELECT_LONG:
                switch (devset_idx)
                {
                    case 0:  ui = UI_SETTINGS_GAP;        break;
                    case 1:  ui = UI_SETTINGS_RETRY;      break;
                    case 2:  ui = UI_SETTINGS_UV;         break;
                    case 3:  ui = UI_SETTINGS_OV;         break;
                    case 4:  ui = UI_SETTINGS_OL;         break;
                    case 5:  ui = UI_SETTINGS_UL;         break;
                    case 6:  ui = UI_SETTINGS_MAXRUN;     break;
                    case 7:  ui = UI_DEVSET_EDIT_DATE;    break;
                    case 8:  ui = UI_DEVSET_EDIT_TIME;    break;
                    case 9:  ui = UI_DEVSET_EDIT_DAY;     break;
                    case 10: ui = UI_SETTINGS_PWRREST;    break;
                    case 11: ui = UI_SETTINGS_FACTORY;    break;
                    case 12: ui = UI_MENU;                break;
                    default: break;
                }
                break;

            case BTN_RESET:
                ui = UI_MENU;
                break;

            default:
                break;
        }

        screenNeedsRefresh = true;
        return;
    }

    /* ===============================
       MAIN MENU
       =============================== */
    if (ui == UI_MENU)
    {
        switch (b)
        {
            case BTN_UP:
                if (menu_idx > 0) menu_idx--;
                break;

            case BTN_DOWN:
                if (menu_idx < MAIN_MENU_COUNT - 1) menu_idx++;
                break;

            case BTN_SELECT:
            case BTN_SELECT_LONG:
                menu_select();
                break;

            case BTN_RESET:
                ui = UI_DASH;
                break;

            default:
                break;
        }
        screenNeedsRefresh = true;
        return;
    }

    /* ===============================
       TIMER SLOT SELECT
       =============================== */
    if (ui == UI_TIMER_SLOT_SELECT)
    {
        switch (b)
        {
            case BTN_UP:
                if (currentSlot > 0) currentSlot--;
                break;

            case BTN_DOWN:
                if (currentSlot < 5) currentSlot++;
                break;

            case BTN_SELECT:
            case BTN_SELECT_LONG:
                menu_select();
                break;

            case BTN_RESET:
                ui = UI_MENU;
                break;

            default:
                break;
        }

        timer_page = (currentSlot < 2 ? 0 :
                     (currentSlot < 5 ? 1 : 2));

        screenNeedsRefresh = true;
        return;
    }

    /* ===============================
       DASHBOARD BUTTON ACTIONS
       =============================== */
    switch (b)
    {
        case BTN_RESET:
            /* Restart pump / test-run */
            reset();
            ui = UI_DASH;
            screenNeedsRefresh = true;
            return;

        case BTN_SELECT:
            /* Auto mode toggle from DASH */
            if (ui != UI_DASH) return;
            if (!autoActive)
                ModelHandle_StartAuto(edit_auto_gap_s, edit_auto_maxrun_min, edit_auto_retry);
            else
                ModelHandle_StopAuto();
            screenNeedsRefresh = true;
            return;

        case BTN_SELECT_LONG:
            /* Open Main Menu */
            ui = UI_MENU;
            menu_idx = 0;
            menu_view_top = 0;
            screenNeedsRefresh = true;
            return;

        case BTN_UP:
            /* Timer mode toggle (nearest slot) */
            if (!timerActive)
                ModelHandle_StartTimerNearestSlot();
            else
                ModelHandle_StopTimer();
            ui = UI_DASH;
            screenNeedsRefresh = true;
            return;

        case BTN_UP_LONG:
            /* Semi-auto toggle */
            if (!semiAutoActive)
                ModelHandle_StartSemiAuto();
            else
                ModelHandle_StopSemiAuto();
            ui = UI_DASH;
            screenNeedsRefresh = true;
            return;

        case BTN_DOWN:
            /* Countdown RUN / STOP */
            if (!countdownActive)
            {
                ModelHandle_StartCountdown(edit_countdown_min * 60);
                ui = UI_COUNTDOWN;
            }
            else
            {
                ModelHandle_StopCountdown();
                ui = UI_DASH;
            }
            screenNeedsRefresh = true;
            return;

        case BTN_DOWN_LONG:
            /* Long press to EDIT countdown time (when not running) */
            if (!countdownActive)
            {
                ui = UI_COUNTDOWN_EDIT_MIN;
                screenNeedsRefresh = true;
            }
            return;

        default:
            return;
    }
}

/***************************************************************
 *  LCD UPDATE ENGINE + UI DISPATCHER
 ***************************************************************/
void Screen_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* CURSOR BLINK ONLY IN MAIN MENU */
    bool cursorBlinkActive = (ui == UI_MENU);

    if (cursorBlinkActive && (now - lastCursorToggle >= CURSOR_BLINK_MS))
    {
        cursorVisible = !cursorVisible;
        lastCursorToggle = now;
        draw_menu_cursor();
    }

    /* WELCOME → DASH AUTO */
    if (ui == UI_WELCOME && now - lastLcdUpdateTime >= WELCOME_MS)
    {
        ui = UI_DASH;
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }

    /* AUTO BACK TO DASH AFTER INACTIVITY */
    if (ui != UI_WELCOME &&
        ui != UI_DASH &&
        (now - lastUserAction >= AUTO_BACK_MS))
    {
        ui = UI_DASH;
        screenNeedsRefresh = true;
    }

    /* DASH & COUNTDOWN REFRESH EVERY 1s */
    if ((ui == UI_DASH || ui == UI_COUNTDOWN) &&
        now - lastLcdUpdateTime >= 1000)
    {
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }

    /* REDRAW IF NEEDED */
    if (screenNeedsRefresh || ui != last_ui)
    {
        bool fullRefresh = (ui != last_ui);
        last_ui = ui;
        screenNeedsRefresh = false;

        if (fullRefresh)
            lcd_clear();

        switch(ui)
        {
            /* WELCOME + DASH */
            case UI_WELCOME:              show_welcome();              break;
            case UI_DASH:                 show_dash();                 break;

            /* MAIN MENU */
            case UI_MENU:                 show_menu();                 break;

            /* TIMER */
            case UI_TIMER_SLOT_SELECT:    show_timer_slot_select();    break;
            case UI_TIMER_EDIT_ON_TIME:   show_edit_on_time();         break;
            case UI_TIMER_EDIT_OFF_TIME:  show_edit_off_time();        break;
            case UI_TIMER_EDIT_DAYS:      show_timer_days();           break;
            case UI_TIMER_EDIT_GAP:       show_timer_gap();            break;
            case UI_TIMER_EDIT_ENABLE:    show_timer_enable();         break;
            case UI_TIMER_EDIT_SUMMARY:   show_timer_summary();        break;

            /* AUTO */
            case UI_AUTO_MENU:            show_auto_menu();            break;
            case UI_AUTO_EDIT_GAP:        show_auto_gap();             break;
            case UI_AUTO_EDIT_MAXRUN:     show_auto_maxrun();          break;
            case UI_AUTO_EDIT_RETRY:      show_auto_retry();           break;

            /* SEMI AUTO */
            case UI_SEMI_AUTO:            show_semi_auto();            break;

            /* TWIST */
            case UI_TWIST:                show_twist();                break;
            case UI_TWIST_EDIT_ON:        show_twist_on_sec();         break;
            case UI_TWIST_EDIT_OFF:       show_twist_off_sec();        break;
            case UI_TWIST_EDIT_ON_H:      show_twist_on_h();           break;
            case UI_TWIST_EDIT_ON_M:      show_twist_on_m();           break;
            case UI_TWIST_EDIT_OFF_H:     show_twist_off_h();          break;
            case UI_TWIST_EDIT_OFF_M:     show_twist_off_m();          break;

            /* COUNTDOWN */
            case UI_COUNTDOWN:            show_countdown();            break;
            case UI_COUNTDOWN_EDIT_MIN:   show_countdown_edit_min();   break;

            /* DEVICE SETUP (SCROLL MENU + EDITORS) */
            case UI_DEVSET_MENU:          show_devset_menu();          break;

            case UI_SETTINGS_GAP:         show_settings_gap();         break;
            case UI_SETTINGS_RETRY:       show_settings_retry();       break;
            case UI_SETTINGS_UV:          show_settings_uv();          break;
            case UI_SETTINGS_OV:          show_settings_ov();          break;
            case UI_SETTINGS_OL:          show_settings_ol();          break;
            case UI_SETTINGS_UL:          show_settings_ul();          break;
            case UI_SETTINGS_MAXRUN:      show_settings_maxrun();      break;
            case UI_SETTINGS_PWRREST:     show_settings_pwrrest();     break;
            case UI_SETTINGS_FACTORY:     show_settings_factory();     break;

            case UI_DEVSET_EDIT_DATE:     show_devset_edit_date();     break;
            case UI_DEVSET_EDIT_TIME:     show_devset_edit_time();     break;
            case UI_DEVSET_EDIT_DAY:      show_devset_edit_day();      break;

            /* ADD NEW DEVICE FLOWS */
            case UI_ADD_DEVICE_MENU:        show_add_device_menu();        break;
            case UI_ADD_DEVICE_PAIR:        show_add_device_pair();        break;
            case UI_ADD_DEVICE_REMOVE:      show_add_device_remove();      break;
            case UI_ADD_DEVICE_PAIR_DONE:   show_add_device_pair_done();   break;
            case UI_ADD_DEVICE_REMOVE_DONE: show_add_device_remove_done(); break;

            /* RESET CONFIRM */
            case UI_RESET_CONFIRM:        show_reset_confirm();        break;

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
