#include "screen.h"
#include "lcd_i2c.h"
#include "switches.h"
#include "model_handle.h"
#include "adc.h"
#include "rtc_i2c.h"
#include "acs712.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
typedef enum {
    UI_WELCOME = 0,
    UI_DASH,
    UI_MENU,
    UI_TIMER_SLOT_SELECT,
    UI_TIMER_EDIT_ON_TIME,
    UI_TIMER_EDIT_OFF_TIME,
    UI_TIMER_EDIT_DAYS,
    UI_TIMER_EDIT_GAP,
    UI_TIMER_EDIT_ENABLE,
    UI_TIMER_EDIT_SUMMARY,
    UI_AUTO_MENU,
    UI_AUTO_EDIT_GAP,
    UI_AUTO_EDIT_MAXRUN,
    UI_AUTO_EDIT_RETRY,
    UI_SEMI_AUTO,
    UI_TWIST,
    UI_TWIST_EDIT_ON,
    UI_TWIST_EDIT_OFF,
    UI_TWIST_EDIT_ON_H,
    UI_TWIST_EDIT_ON_M,
    UI_TWIST_EDIT_OFF_H,
    UI_TWIST_EDIT_OFF_M,
    UI_COUNTDOWN,
    UI_COUNTDOWN_EDIT_MIN,
    UI_DEVSET_MENU,
    UI_SETTINGS_GAP,
    UI_SETTINGS_RETRY,
    UI_SETTINGS_UV,
    UI_SETTINGS_OV,
    UI_SETTINGS_OL,
    UI_SETTINGS_UL,
    UI_SETTINGS_MAXRUN,
    UI_SETTINGS_PWRREST,
    UI_SETTINGS_FACTORY,
    UI_DEVSET_EDIT_DATE,
    UI_DEVSET_EDIT_TIME,
    UI_DEVSET_EDIT_DAY,
    UI_ADD_DEVICE_MENU,
    UI_ADD_DEVICE_PAIR,
    UI_ADD_DEVICE_REMOVE,
    UI_ADD_DEVICE_PAIR_DONE,
    UI_ADD_DEVICE_REMOVE_DONE,
    UI_RESET_CONFIRM,
    UI_NONE,
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
#define CONTINUOUS_STEP_MS  250
#define COUNTDOWN_INC_MS   2000
static uint16_t bk_gap = 10;
static uint8_t  bk_retry = 3;
static uint16_t bk_uv = 180;
static uint16_t bk_ov = 260;
static int      bk_ol = 6;
static int      bk_ul = 2;
static uint16_t bk_maxrun = 120;

/* Button press tracking */
static uint32_t sw_press_start[4] = {0,0,0,0};
static bool     sw_long_issued[4] = {false,false,false,false};
static uint32_t last_repeat_time = 0;
static bool reset_confirm_yes = false;
extern ADC_Data adcData;
extern TimerSlot timerSlots[5];
extern TwistSettings twistSettings;
extern RTC_Time_t time;
extern volatile bool manualActive;
extern volatile bool semiAutoActive;
extern volatile bool timerActive;
extern volatile bool countdownActive;
extern volatile bool twistActive;
extern volatile bool autoActive;
extern volatile uint32_t countdownDuration;
static uint8_t edit_on_h = 0;
static uint8_t edit_on_m = 0;
static uint8_t edit_off_h = 0;
static uint8_t edit_off_m = 0;
static uint8_t time_edit_field = 0;
static uint8_t edit_day_mask = 0x7F;
static uint8_t edit_day_index = 0;
static uint8_t edit_gap_min = 0;
static bool edit_slot_enabled = true;
static uint8_t currentSlot = 0;
static uint8_t timer_page = 0;
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
static uint16_t edit_settings_gap_s = 10;
static uint8_t  edit_settings_retry = 3;
static uint16_t edit_settings_uv    = 180;
static uint16_t edit_settings_ov    = 260;
static int edit_settings_ol = 6;
static int edit_settings_ul = 0;
static uint16_t edit_settings_maxrun = 120;
static uint8_t  edit_settings_pwrrest = 0;
static bool     edit_settings_factory_yes = false;
static uint8_t  edit_date_dd    = 1;
static uint8_t  edit_date_mm    = 1;
static uint16_t edit_date_yyyy  = 2025;
static uint8_t  edit_date_field = 0;
static uint8_t  edit_time_hh    = 0;
static uint8_t  edit_time_min   = 0;
static uint8_t  edit_time_field = 0;
static uint8_t  edit_day_idx2   = 0;
static const char* const dowNames[7] = {
    "Sun","Mon","Tue","Wed","Thu","Fri","Sat"
};
static uint8_t addDevMenuIndex  = 0;
static uint8_t addDevTypeIndex  = 0;
static uint8_t lastAddDevType   = 0;
static bool    lastAddActionPair = true;
static const char* const addDevTypeNames[] = {
    "Wi-Fi",
    "Receiver",
    "Transmitter"
};
static const char* const main_menu[] = {
    "Add New Device",
    "Device Setup",
    "Reset To Default"
};
#define MAIN_MENU_COUNT 3
static uint8_t menu_idx      = 0;
static uint8_t menu_view_top = 0;
static const char* const devset_menu_items[] = {
    "Dry Run",
    "Testing Gap",
    "Low Volt",
    "High Volt",
    "Over Load",
    "Under Load",
    "Max Run",
    "Set Date",
    "Set Time",
    "Set Day",
    "Power Restore",
    "Factory Reset",
    "Back"
};
#define DEVSET_MENU_COUNT  (sizeof(devset_menu_items)/sizeof(devset_menu_items[0]))
static uint8_t devset_idx      = 0;
static uint8_t devset_view_top = 0;
void Screen_Init(void)
{
    lcd_init();
    lcd_clear();
    ui = UI_WELCOME;
    last_ui = UI_NONE;
    screenNeedsRefresh = true;
    lastUserAction = HAL_GetTick();
}
static inline void refreshInactivityTimer(void){
    lastUserAction = HAL_GetTick();
}
static inline void lcd_line(uint8_t row, const char* s){
    char buf[17];
    snprintf(buf, sizeof(buf), "%-16.16s", s);
    lcd_put_cur(row,0);
    lcd_send_string(buf);
}
static uint8_t dash_page = 0;
static uint32_t dash_cycle_start = 0;
#define DASH_PAGE1_TIME 20000
#define DASH_PAGE2_TIME 5000
#define DASH_PAGE3_TIME 5000
static inline void lcd_line0(const char* s){ lcd_line(0,s); }
static inline void lcd_line1(const char* s){ lcd_line(1,s); }
static void show_welcome(void)
{
    lcd_clear();
    lcd_line0("   HELONIX");
    lcd_line1(" IntelligentSys");
}
static void show_dash(void)
{
    char l0[17], l1[17];
    uint32_t now = HAL_GetTick();
    if (dash_cycle_start == 0)
        dash_cycle_start = now;
    uint32_t elapsed = now - dash_cycle_start;
    if (elapsed < DASH_PAGE1_TIME)
        dash_page = 0;
    else if (elapsed < (DASH_PAGE1_TIME + DASH_PAGE2_TIME))
        dash_page = 1;
    else if (elapsed < (DASH_PAGE1_TIME + DASH_PAGE2_TIME + DASH_PAGE3_TIME))
        dash_page = 2;
    else
    {
        dash_cycle_start = now;
        dash_page = 0;
    }
    int submerged = 0;
    for (int i = 0; i <= 3; i++)
    {
        if (adcData.voltages[i] < 0.1f)
            submerged++;
    }
    int tankPercent = (submerged * 100) / 4;
    const char* mode =
        manualActive    ? "MANUL" :
        semiAutoActive  ? "SEMI " :
        timerActive     ? "TIMER" :
        countdownActive ? "COUNT" :
        twistActive     ? "TWIST" :
        autoActive      ? "AUTO " : "IDLE ";
    bool motorOn = Motor_GetStatus();
    if (dash_page == 0)
    {
        snprintf(l0, sizeof(l0), "M:%s %-5s %3d%%",
                 motorOn ? "ON " : "OFF",
                 mode,
                 tankPercent);
        const char *gw =
            (adcData.voltages[4] <= 0.01f) ? "YES" : "NO ";

        const char *dry =
            (adcData.voltages[5] <= 0.01f) ? "YES" : "NO ";
        snprintf(l1, sizeof(l1), "G.W:%s DRY:%s", gw, dry);

    }
    else if (dash_page == 1)
    {
        snprintf(l0, sizeof(l0), "Date:%02u-%02u-%02u",
                 time.dom,
                 time.month,
                 (uint8_t)(time.year % 100));
        snprintf(l1, sizeof(l1), "Time:%02u:%02u:%02u",
                 time.hour,
                 time.min,
                 time.sec);
    }
    else
    {
        snprintf(l0, sizeof(l0), "V:%3.0fV  I:%3.1fA",
                 g_voltageV,
                 g_currentA);
        if (ModelHandle_IsOverload())
            snprintf(l1, sizeof(l1), "OVER LOAD!");
        else if (ModelHandle_IsUnderload())
            snprintf(l1, sizeof(l1), "UNDER LOAD!");
        else if (ModelHandle_IsVoltageFault())
            snprintf(l1, sizeof(l1), "VOLT Error!");
        else
            snprintf(l1, sizeof(l1), "Live Monitor");
    }
    lcd_line0(l0);
    lcd_line1(l1);
}
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
static void show_devset_menu(void)
{
    char l0[17], l1[17];
    if (devset_idx < devset_view_top)
        devset_view_top = devset_idx;
    else if (devset_idx > devset_view_top + 1)
        devset_view_top = devset_idx - 1;
    uint8_t idx0 = devset_view_top;
    uint8_t idx1 = devset_view_top + 1;
    char star0 = ' ';
    char star1 = ' ';
    if (idx0 == 0 && edit_settings_gap_s > 0)     star0 = '*';
    if (idx0 == 1 && edit_settings_retry > 0)     star0 = '*';
    if (idx0 == 2 && edit_settings_uv > 0)        star0 = '*';
    if (idx0 == 3 && edit_settings_ov > 0)        star0 = '*';
    if (idx0 == 4 && edit_settings_ol > 0)        star0 = '*';
    if (idx0 == 5 && edit_settings_ul > 0)        star0 = '*';
    if (idx0 == 6 && edit_settings_maxrun > 0)    star0 = '*';
    if (idx1 == 0 && edit_settings_gap_s > 0)     star1 = '*';
    if (idx1 == 1 && edit_settings_retry > 0)     star1 = '*';
    if (idx1 == 2 && edit_settings_uv > 0)        star1 = '*';
    if (idx1 == 3 && edit_settings_ov > 0)        star1 = '*';
    if (idx1 == 4 && edit_settings_ol > 0)        star1 = '*';
    if (idx1 == 5 && edit_settings_ul > 0)        star1 = '*';
    if (idx1 == 6 && edit_settings_maxrun > 0)    star1 = '*';
    if (idx0 < DEVSET_MENU_COUNT)
        snprintf(l0, sizeof(l0), "%c%c%-14.14s",
                 (devset_idx == idx0 ? '>' : ' '),
                 star0,
                 devset_menu_items[idx0]);
    else
        snprintf(l0, sizeof(l0), "                ");
    if (idx1 < DEVSET_MENU_COUNT)
        snprintf(l1, sizeof(l1), "%c%c%-14.14s",
                 (devset_idx == idx1 ? '>' : ' '),
                 star1,
                 devset_menu_items[idx1]);
    else
        snprintf(l1, sizeof(l1), "                ");
    lcd_line0(l0);
    lcd_line1(l1);
}
static void show_timer_slot_select(void)
{
    lcd_clear();
    int item1 = timer_page * 2;
    int item2 = item1 + 1;
    char l0[17], l1[17];
    if (item1 == 5) {
        snprintf(l0, sizeof(l0), "%c Back",
                 (currentSlot == 5 ? '>' : ' '));
    }
    else {
        snprintf(l0, sizeof(l0), "%c Timer %d",
                 (currentSlot == item1 ? '>' : ' '),
                 item1 + 1);
    }
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
static const char* dayNames[] = {
    "Enable All", "Disable All",
    "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun",
    "Next>"
};
static void show_timer_days(void)
{
    lcd_line0("Timer Days");
    char buf[17];
    if (edit_day_index >= 2 && edit_day_index <= 8)
        {
            uint8_t d = edit_day_index - 2;
            uint8_t isOn = (edit_day_mask >> d) & 1;
            snprintf(buf,sizeof(buf),"> %s (%s)", dayNames[edit_day_index], isOn?"ON":"OFF");
        }
        else
        {
            snprintf(buf,sizeof(buf),"> %s", dayNames[edit_day_index]);
        }
    lcd_line1(buf);
}
static void show_timer_gap(void)
{
    lcd_line0("Timer Gap (min)");

    char buf[17];
    snprintf(buf, sizeof(buf), ">T%u %3u min Next>",
             (unsigned)(currentSlot + 1),
             (unsigned)edit_gap_min);

    lcd_line1(buf);
}
static void show_timer_enable(void)
{
    char title[17];
    snprintf(title, sizeof(title), "T%u Enable?",
             (unsigned)(currentSlot + 1));
    lcd_line0(title);
    lcd_line1(edit_slot_enabled ? "YES       Next>" :
                                  "NO        Next>");
}
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
static void show_semi_auto(void)
{
    lcd_line0("Semi-Auto");
    lcd_line1(semiAutoActive ? "val:Disable Next>"
                             : "val:Enable  Next>");
}
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
static void show_settings_gap(void){
    lcd_line0("Set Dry Run");
    char buf[17];
    if (edit_settings_gap_s == 0)
        snprintf(buf,sizeof(buf),"Disable    Next>");
    else
        snprintf(buf,sizeof(buf),"val:%2umin Next>", edit_settings_gap_s);
    lcd_line1(buf);
}
static void show_settings_retry(void){
    lcd_line0("Testing Gap");
    char buf[17];
    if (edit_settings_retry == 0)
        snprintf(buf,sizeof(buf),"Disable    Next>");
    else
        snprintf(buf,sizeof(buf),"val:%3umin Next>", edit_settings_retry);
    lcd_line1(buf);
}
static void show_settings_uv(void){
    lcd_line0("Low Volt");
    char buf[17];
    if (edit_settings_uv == 0)
        snprintf(buf,sizeof(buf),"Disable    Next>");
    else
        snprintf(buf,sizeof(buf),"val:%3uV Next>", edit_settings_uv);
    lcd_line1(buf);
}
static void show_settings_ov(void){
    lcd_line0("High Volt");
    char buf[17];
    if (edit_settings_ov == 0)
        snprintf(buf,sizeof(buf),"Disable    Next>");
    else
        snprintf(buf,sizeof(buf),"val:%3uV Next>", edit_settings_ov);
    lcd_line1(buf);
}
static void show_settings_ol(void){
    lcd_line0("Over Load (A)");
    char buf[17];
    if (edit_settings_ol < 1)
        snprintf(buf, sizeof(buf), "Disable    Next>");
    else
        snprintf(buf, sizeof(buf), "val:%3d Next>", edit_settings_ol);
    lcd_line1(buf);
}
static void show_settings_ul(void){
    lcd_line0("Under Load (A)");
    char buf[17];
    if (edit_settings_ul < 1)
        snprintf(buf, sizeof(buf), "Disable    Next>");
    else
        snprintf(buf, sizeof(buf), "val:%3d Next>", edit_settings_ul);
    lcd_line1(buf);
}
static void show_settings_maxrun(void){
    lcd_line0("Max Run");
    char buf[17];
    if (edit_settings_maxrun == 0)
        snprintf(buf,sizeof(buf),"Disable    Next>");
    else
        snprintf(buf,sizeof(buf),"val:%3umin Next>",edit_settings_maxrun);
    lcd_line1(buf);
}
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
static void show_reset_confirm(void)
{
    lcd_line0("Reset To Default?");
    lcd_line1(reset_confirm_yes ? "YES       Apply>" :
                                  "NO        Back>");
}
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
static void apply_settings_core(void)
{
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
static void start_settings_edit_flow(void)
{
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
    if (edit_settings_uv != 0) {
        if (edit_settings_uv < 150) edit_settings_uv = 150;
        if (edit_settings_uv > 200) edit_settings_uv = 200;
    }
    edit_settings_ov      = ModelHandle_GetOverVolt();
    if (edit_settings_ov != 0) {
        if (edit_settings_ov < 250) edit_settings_ov = 250;
        if (edit_settings_ov > 300) edit_settings_ov = 300;
    }
    edit_settings_ol      = ModelHandle_GetOverloadLimit();
    if (edit_settings_ol > 25.0f) edit_settings_ol = 25.0f;
    edit_settings_ul      = ModelHandle_GetUnderloadLimit();
    if (edit_settings_ul > 10.0f) edit_settings_ul = 10.0f;
    edit_settings_maxrun  = ModelHandle_GetMaxRunTime();
    if (edit_settings_maxrun > 300) edit_settings_maxrun = 300;
    edit_settings_pwrrest = ModelHandle_GetPowerRestoreMode();
    edit_settings_factory_yes = 0;
    RTC_GetTimeDate();
    edit_date_dd    = time.dom;
    edit_date_mm    = time.month;
    edit_date_yyyy  = time.year;
    edit_date_field = 0;
    edit_time_hh    = time.hour;
    edit_time_min   = time.min;
    edit_time_field = 0;
    if (time.dow >= 1 && time.dow <= 7)
        edit_day_idx2 = (uint8_t)((time.dow - 1) % 7);
    else
        edit_day_idx2 = 0;
    devset_idx      = 0;
    devset_view_top = 0;
    ui = UI_DEVSET_MENU;
    screenNeedsRefresh = true;
}
static void goto_menu_top(void)
{
    menu_idx = 0;
    menu_view_top = 0;
}
static void menu_select(void)
{
    refreshInactivityTimer();
    if (ui == UI_WELCOME)
    {
        ui = UI_DASH;
        screenNeedsRefresh = true;
        return;
    }
    if (ui == UI_DASH)
    {
        goto_menu_top();
        ui = UI_MENU;
        screenNeedsRefresh = true;
        return;
    }
    if (ui == UI_MENU)
    {
        switch(menu_idx)
        {
            case 0:
                addDevMenuIndex = 0;
                addDevTypeIndex = 0;
                ui = UI_ADD_DEVICE_MENU;
                screenNeedsRefresh = true;
                return;
            case 1:
                start_settings_edit_flow();
                return;
            case 2:
                reset_confirm_yes = false;
                ui = UI_RESET_CONFIRM;
                screenNeedsRefresh = true;
                return;
        }
    }
    if (ui == UI_TIMER_SLOT_SELECT)
        {
            if (currentSlot == 5)
            {
                ui = UI_MENU;
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
            apply_timer_slot();
            ui = UI_TIMER_SLOT_SELECT;
            break;
        case UI_TIMER_EDIT_SUMMARY:
            apply_timer_slot();
            ui = UI_TIMER_SLOT_SELECT;
            break;
        default:
            break;
    }
    screenNeedsRefresh = true;
}
void increase_edit_value(void)
{
    switch(ui)
    {
        case UI_TIMER_EDIT_ON_TIME:
            if (time_edit_field == 0) {
                if (edit_on_h <= 18) edit_on_h += 5;
                else edit_on_h = 23;
            } else {
                if (edit_on_m <= 54) edit_on_m += 5;
                else edit_on_m = 59;
            }
            break;

        case UI_TIMER_EDIT_OFF_TIME:
            if (time_edit_field == 0) {
                if (edit_off_h <= 18) edit_off_h += 5;
                else edit_off_h = 23;
            } else {
                if (edit_off_m <= 54) edit_off_m += 5;
                else edit_off_m = 59;
            }
            break;

        case UI_TIMER_EDIT_GAP:
            if (edit_gap_min <= 235) edit_gap_min += 5;
            else edit_gap_min = 240;
            break;

        case UI_AUTO_EDIT_GAP:        edit_auto_gap_s += 5;      break;
        case UI_AUTO_EDIT_MAXRUN:     edit_auto_maxrun_min += 5; break;
        case UI_AUTO_EDIT_RETRY:      edit_auto_retry += 5;      break;

        case UI_TWIST_EDIT_ON:        edit_twist_on_s += 5;      break;
        case UI_TWIST_EDIT_OFF:       edit_twist_off_s += 5;     break;

        case UI_TWIST_EDIT_ON_H:
            if (edit_twist_on_hh <= 18) edit_twist_on_hh += 5;
            else edit_twist_on_hh = 23;
            break;

        case UI_TWIST_EDIT_ON_M:
            if (edit_twist_on_mm <= 54) edit_twist_on_mm += 5;
            else edit_twist_on_mm = 59;
            break;

        case UI_TWIST_EDIT_OFF_H:
            if (edit_twist_off_hh <= 18) edit_twist_off_hh += 5;
            else edit_twist_off_hh = 23;
            break;

        case UI_TWIST_EDIT_OFF_M:
            if (edit_twist_off_mm <= 54) edit_twist_off_mm += 5;
            else edit_twist_off_mm = 59;
            break;

        /* 🚫 REMOVED: UI_COUNTDOWN_EDIT_MIN
           Countdown edit is handled ONLY in Screen_HandleSwitches() */

        case UI_SETTINGS_GAP:
            if (edit_settings_gap_s <= 10) edit_settings_gap_s += 5;
            else edit_settings_gap_s = 15;
            break;

        case UI_SETTINGS_RETRY:
            if (edit_settings_retry <= 175) edit_settings_retry += 5;
            else edit_settings_retry = 180;
            break;

        case UI_SETTINGS_UV:
            if (edit_settings_uv == 0) edit_settings_uv = 150;
            else if (edit_settings_uv <= 195) edit_settings_uv += 5;
            else edit_settings_uv = 200;
            break;

        case UI_SETTINGS_OV:
            if (edit_settings_ov == 0) edit_settings_ov = 250;
            else if (edit_settings_ov <= 295) edit_settings_ov += 5;
            else edit_settings_ov = 300;
            break;

        case UI_SETTINGS_OL:
            if (edit_settings_ol <= 20) edit_settings_ol += 5;
            else edit_settings_ol = 25;
            break;

        case UI_SETTINGS_UL:
            if (edit_settings_ul <= 5) edit_settings_ul += 5;
            else edit_settings_ul = 10;
            break;

        case UI_SETTINGS_MAXRUN:
            if (edit_settings_maxrun == 0) edit_settings_maxrun = 10;
            else if (edit_settings_maxrun <= 295) edit_settings_maxrun += 5;
            else edit_settings_maxrun = 300;
            break;

        case UI_SETTINGS_PWRREST:
            edit_settings_pwrrest = (edit_settings_pwrrest + 1) % 3;
            break;

        case UI_SETTINGS_FACTORY:
            edit_settings_factory_yes ^= 1;
            break;

        case UI_DEVSET_EDIT_DATE:
            if (edit_date_field == 0) {
                if (edit_date_dd <= 26) edit_date_dd += 5;
                else edit_date_dd = 31;
            } else if (edit_date_field == 1) {
                if (edit_date_mm <= 7) edit_date_mm += 5;
                else edit_date_mm = 12;
            } else {
                if (edit_date_yyyy <= 2094) edit_date_yyyy += 5;
                else edit_date_yyyy = 2099;
            }
            break;

        case UI_DEVSET_EDIT_TIME:
            if (edit_time_field == 0) {
                if (edit_time_hh <= 18) edit_time_hh += 5;
                else edit_time_hh = 23;
            } else {
                if (edit_time_min <= 54) edit_time_min += 5;
                else edit_time_min = 59;
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
        case UI_TIMER_EDIT_ON_TIME:
            if (time_edit_field == 0) {
                if (edit_on_h >= 5) edit_on_h -= 5;
                else edit_on_h = 0;
            } else {
                if (edit_on_m >= 5) edit_on_m -= 5;
                else edit_on_m = 0;
            }
            break;
        case UI_TIMER_EDIT_OFF_TIME:
            if (time_edit_field == 0) {
                if (edit_off_h >= 5) edit_off_h -= 5;
                else edit_off_h = 0;
            } else {
                if (edit_off_m >= 5) edit_off_m -= 5;
                else edit_off_m = 0;
            }
            break;
        case UI_TIMER_EDIT_GAP:
            if (edit_gap_min >= 5) edit_gap_min -= 5;
            else edit_gap_min = 0;
            break;
        case UI_AUTO_EDIT_GAP:
            if (edit_auto_gap_s >= 5) edit_auto_gap_s -= 5;
            else edit_auto_gap_s = 0;
            break;
        case UI_AUTO_EDIT_MAXRUN:
            if (edit_auto_maxrun_min >= 5) edit_auto_maxrun_min -= 5;
            else edit_auto_maxrun_min = 0;
            break;
        case UI_AUTO_EDIT_RETRY:
            if (edit_auto_retry >= 5) edit_auto_retry -= 5;
            else edit_auto_retry = 0;
            break;
        case UI_SETTINGS_GAP:
            if (edit_settings_gap_s >= 5) edit_settings_gap_s -= 5;
            else edit_settings_gap_s = 0;
            break;
        case UI_SETTINGS_RETRY:
            if (edit_settings_retry >= 5) edit_settings_retry -= 5;
            else edit_settings_retry = 0;
            break;
        case UI_SETTINGS_OL:
            if (edit_settings_ol >= 5) edit_settings_ol -= 5;
            else edit_settings_ol = 0;
            break;
        case UI_SETTINGS_UL:
            if (edit_settings_ul >= 5) edit_settings_ul -= 5;
            else edit_settings_ul = 0;
            break;
        case UI_SETTINGS_MAXRUN:
            if (edit_settings_maxrun >= 15) edit_settings_maxrun -= 5;
            else edit_settings_maxrun = 0;
            break;
        case UI_DEVSET_EDIT_TIME:
            if (edit_time_field == 0) {
                if (edit_time_hh >= 5) edit_time_hh -= 5;
                else edit_time_hh = 0;
            } else {
                if (edit_time_min >= 5) edit_time_min -= 5;
                else edit_time_min = 0;
            }
            break;
        case UI_DEVSET_EDIT_DAY:
            edit_day_idx2 = (uint8_t)((edit_day_idx2 + 6) % 7);
            break;

        default:
            break;
    }
}
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
void Screen_HandleSwitches(void)
{
    static bool prev_sw_down_edit = false;
    UiButton b = decode_button_press();
    uint32_t now = HAL_GetTick();
    bool sw_up   = Switch_IsPressed(2);
    bool sw_down = Switch_IsPressed(3);
    if (sw_up && sw_long_issued[2] && ui != UI_COUNTDOWN_EDIT_MIN)
    {
        if (now - last_repeat_time >= CONTINUOUS_STEP_MS)
        {
            last_repeat_time = now;
            if (ui == UI_MENU && menu_idx > 0)
                menu_idx--;
            else if (ui == UI_TIMER_SLOT_SELECT && currentSlot > 0)
                currentSlot--;
            else if (ui == UI_DEVSET_MENU && devset_idx > 0)
                devset_idx--;
            screenNeedsRefresh = true;
        }
    }
    if (sw_down && sw_long_issued[3] && ui != UI_COUNTDOWN_EDIT_MIN)
    {
        if (now - last_repeat_time >= CONTINUOUS_STEP_MS)
        {
            last_repeat_time = now;
            if (ui == UI_MENU && menu_idx < MAIN_MENU_COUNT - 1)
                menu_idx++;
            else if (ui == UI_TIMER_SLOT_SELECT && currentSlot < 5)
                currentSlot++;
            else if (ui == UI_DEVSET_MENU && devset_idx < DEVSET_MENU_COUNT - 1)
                devset_idx++;
            screenNeedsRefresh = true;
        }
    }
    if (ui == UI_COUNTDOWN_EDIT_MIN)
    {
        if (sw_down)
        {
            if (!prev_sw_down_edit ||
                (now - last_repeat_time >= CONTINUOUS_STEP_MS))
            {
                last_repeat_time = now;
                if (edit_countdown_min < 999)
                    edit_countdown_min += 1;
                screenNeedsRefresh = true;
            }
        }
        else if (prev_sw_down_edit)
        {
            ui = UI_DASH;
            screenNeedsRefresh = true;
        }
        prev_sw_down_edit = sw_down;
        return;
    }

    prev_sw_down_edit = false;

    if (b == BTN_NONE)
        return;

    refreshInactivityTimer();

    /* ==============================
       NORMAL BUTTON HANDLING
    ===============================*/

    switch (b)
    {
        case BTN_RESET:
            ModelHandle_ToggleManual();
            ui = UI_DASH;
            break;

        case BTN_SELECT:
            if (!autoActive)
                ModelHandle_StartAuto(edit_auto_gap_s,
                                      edit_auto_maxrun_min,
                                      edit_auto_retry);
            else
                ModelHandle_StopAuto();
            break;

        case BTN_SELECT_LONG:
            ui = UI_MENU;
            break;

        case BTN_UP:
            if (!timerActive)
                ModelHandle_StartTimerNearestSlot();
            else
                ModelHandle_StopTimer();
            break;

        case BTN_UP_LONG:
            if (!semiAutoActive)
                ModelHandle_StartSemiAuto();
            else
                ModelHandle_StopSemiAuto();
            break;

        case BTN_DOWN:
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
            break;

        case BTN_DOWN_LONG:
            if (!countdownActive)
            {
                edit_countdown_min = 1;
                ui = UI_COUNTDOWN_EDIT_MIN;
                screenNeedsRefresh = true;
            }
            break;

        default:
            break;
    }

    screenNeedsRefresh = true;
}
void Screen_Update(void)
{
    uint32_t now = HAL_GetTick();
    bool cursorBlinkActive = (ui == UI_MENU);
    if (cursorBlinkActive && (now - lastCursorToggle >= CURSOR_BLINK_MS))
    {
        cursorVisible = !cursorVisible;
        lastCursorToggle = now;
        draw_menu_cursor();
    }
    if (ui == UI_WELCOME && now - lastLcdUpdateTime >= WELCOME_MS)
    {
        ui = UI_DASH;
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }
    if (ui != UI_WELCOME &&
        ui != UI_DASH &&
        (now - lastUserAction >= AUTO_BACK_MS))
    {
        ui = UI_DASH;
        screenNeedsRefresh = true;
    }
    if ((ui == UI_DASH || ui == UI_COUNTDOWN) &&
        now - lastLcdUpdateTime >= 1000)
    {
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }
    if (screenNeedsRefresh || ui != last_ui)
    {
        bool fullRefresh = (ui != last_ui);
        last_ui = ui;
        screenNeedsRefresh = false;
        if (fullRefresh)
            lcd_clear();
        switch(ui)
        {
            case UI_WELCOME:              show_welcome();              break;
            case UI_DASH:                 show_dash();                 break;
            case UI_MENU:                 show_menu();                 break;
            case UI_TIMER_SLOT_SELECT:    show_timer_slot_select();    break;
            case UI_TIMER_EDIT_ON_TIME:   show_edit_on_time();         break;
            case UI_TIMER_EDIT_OFF_TIME:  show_edit_off_time();        break;
            case UI_TIMER_EDIT_DAYS:      show_timer_days();           break;
            case UI_TIMER_EDIT_GAP:       show_timer_gap();            break;
            case UI_TIMER_EDIT_ENABLE:    show_timer_enable();         break;
            case UI_TIMER_EDIT_SUMMARY:   show_timer_summary();        break;
            case UI_AUTO_MENU:            show_auto_menu();            break;
            case UI_AUTO_EDIT_GAP:        show_auto_gap();             break;
            case UI_AUTO_EDIT_MAXRUN:     show_auto_maxrun();          break;
            case UI_AUTO_EDIT_RETRY:      show_auto_retry();           break;
            case UI_SEMI_AUTO:            show_semi_auto();            break;
            case UI_TWIST:                show_twist();                break;
            case UI_TWIST_EDIT_ON:        show_twist_on_sec();         break;
            case UI_TWIST_EDIT_OFF:       show_twist_off_sec();        break;
            case UI_TWIST_EDIT_ON_H:      show_twist_on_h();           break;
            case UI_TWIST_EDIT_ON_M:      show_twist_on_m();           break;
            case UI_TWIST_EDIT_OFF_H:     show_twist_off_h();          break;
            case UI_TWIST_EDIT_OFF_M:     show_twist_off_m();          break;
            case UI_COUNTDOWN:            show_countdown();            break;
            case UI_COUNTDOWN_EDIT_MIN:   show_countdown_edit_min();   break;
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
            case UI_ADD_DEVICE_MENU:        show_add_device_menu();        break;
            case UI_ADD_DEVICE_PAIR:        show_add_device_pair();        break;
            case UI_ADD_DEVICE_REMOVE:      show_add_device_remove();      break;
            case UI_ADD_DEVICE_PAIR_DONE:   show_add_device_pair_done();   break;
            case UI_ADD_DEVICE_REMOVE_DONE: show_add_device_remove_done(); break;
            case UI_RESET_CONFIRM:        show_reset_confirm();        break;

            default:
                break;
        }
    }
}
extern void ModelHandle_StartAuto(uint16_t gap_s, uint16_t maxrun_min, uint16_t retry);
extern void ModelHandle_StartTimerNearestSlot(void);
extern void ModelHandle_StopTimer(void);
extern void ModelHandle_StopSemiAuto(void);
extern void ModelHandle_FactoryReset(void);
