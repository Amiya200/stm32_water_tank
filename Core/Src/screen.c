/***************************************************************
 *  HELONIX Water Pump Controller
 *  FINAL SCREEN.C — FULL CLEAN VERSION (Option 1)
 *  Includes Countdown Fix + SW4 Press Logic + AUTO Display
 *  PART A OF 3
 ***************************************************************/

#include "screen.h"
#include "lcd_i2c.h"
#include "switches.h"
#include "model_handle.h"
#include "adc.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

/* ================================================================
   UI ENUMS (ALL PRESERVED)
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
    UI_TIMER_EDIT_SLOT_ENABLE,

    UI_AUTO_MENU,
    UI_AUTO_EDIT_GAP,
    UI_AUTO_EDIT_MAXRUN,
    UI_AUTO_EDIT_RETRY,

    UI_COUNTDOWN,
    UI_COUNTDOWN_EDIT_MIN,
    UI_COUNTDOWN_TOGGLE,

    UI_TWIST,
    UI_TWIST_EDIT_ON,
    UI_TWIST_EDIT_OFF,
    UI_TWIST_EDIT_ON_H,
    UI_TWIST_EDIT_ON_M,
    UI_TWIST_EDIT_OFF_H,
    UI_TWIST_EDIT_OFF_M,

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

static const uint32_t WELCOME_MS = 2500;
static const uint32_t CURSOR_BLINK_MS = 400;
static const uint32_t AUTO_BACK_MS = 60000;

#define LONG_PRESS_MS        3000
#define CONTINUOUS_STEP_MS    300
#define COUNTDOWN_INC_MS     1000   /* fast repeat for long press */

/* Button press trackers */
static uint32_t sw_press_start[4] = {0,0,0,0};
static bool     sw_long_issued[4] = {false,false,false,false};

static uint32_t last_repeat_time = 0;

/* ================================================================
   EXTERNALS
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

static uint8_t  edit_timer_on_h  = 6,  edit_timer_on_m  = 30;
static uint8_t  edit_timer_off_h = 18, edit_timer_off_m = 30;

static uint16_t edit_auto_gap_s = 60;
static uint16_t edit_auto_maxrun_min = 120;
static uint16_t edit_auto_retry = 0;

static uint16_t edit_twist_on_s  = 5;
static uint16_t edit_twist_off_s = 5;

static uint8_t  edit_twist_on_hh  = 6;
static uint8_t  edit_twist_on_mm  = 0;
static uint8_t  edit_twist_off_hh = 18;
static uint8_t  edit_twist_off_mm = 0;

static uint16_t edit_countdown_min = 0;

static uint8_t currentSlot = 0;

/* MENU ITEMS */
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
   HELPERS
   ================================================================ */

static inline void refreshInactivityTimer(void){
    lastUserAction = HAL_GetTick();
}

/* ================================================================
   SCREEN INIT
   ================================================================ */

void Screen_Init(void)
{
    lcd_init();
    ui = UI_WELCOME;
    lastLcdUpdateTime = HAL_GetTick();
    screenNeedsRefresh = true;
    cursorVisible = true;

    for (int i=0;i<4;i++){
        sw_press_start[i] = 0;
        sw_long_issued[i] = false;
    }

    lcd_clear();
    lcd_put_cur(0,0);
    lcd_send_string("   HELONIX");
    lcd_put_cur(1,0);
    lcd_send_string(" IntelligentSys");
}

/* ================================================================
   LCD Output Formatting
   ================================================================ */

static inline void lcd_line(uint8_t row, const char* s){
    char buf[17];
    snprintf(buf, sizeof(buf), "%-16.16s", s);
    lcd_put_cur(row, 0);
    lcd_send_string(buf);
}

static inline void lcd_line0(const char* s){ lcd_line(0,s); }
static inline void lcd_line1(const char* s){ lcd_line(1,s); }

static inline void lcd_val_next(uint16_t v){
    char buf[17];
    snprintf(buf,sizeof(buf),"val:%03u   Next>",v);
    lcd_line1(buf);
}

/* ================================================================
   SCREENS
   ================================================================ */

static void show_welcome(void){
    lcd_clear();
    lcd_line0("   HELONIX");
    lcd_line1(" IntelligentSys");
}

/* ---------------- DASHBOARD ---------------- */

static void show_dash(void){
    char l0[17], l1[17];

    /* Motor */
    const char* motor = Motor_GetStatus() ? "ON " : "OFF";

    /* Mode text with AUTO fix */
    const char* mode = "IDLE";
    if (manualActive)          mode = "MANUAL";
    else if (semiAutoActive)   mode = "SEMI";
    else if (timerActive)      mode = "TIMER";
    else if (countdownActive)  mode = "CD";
    else if (twistActive)      mode = "TWIST";
    else if (autoActive)       mode = "AUTO";   // FIXED

    snprintf(l0,sizeof(l0),"M:%s %s",motor,mode);

    /* Water level */
    int submerged = 0;
    for (int i=1;i<=5;i++){
        if (adcData.voltages[i] < 0.1f)
            submerged++;
    }

    const char* level =
        (submerged>=5)?"100%":
        (submerged==4)?"80%":
        (submerged==3)?"60%":
        (submerged==2)?"40%":
        (submerged==1)?"20%":"0%";

    snprintf(l1,sizeof(l1),"Water:%s",level);

    lcd_line0(l0);
    lcd_line1(l1);
}

/* ---------------- MENU ---------------- */

static void format_menu_line(char* buf,size_t size,int idx,bool sel){
    if (idx < 0 || idx >= MAIN_MENU_COUNT){
        snprintf(buf,size,"                ");
        return;
    }
    snprintf(buf,size,"%c%-15.15s",sel?'>':' ',main_menu[idx]);
}

static void show_menu(void){
    char l0[17], l1[17];

    if (menu_idx < menu_view_top)
        menu_view_top = menu_idx;
    else if (menu_idx > menu_view_top+1)
        menu_view_top = menu_idx - 1;

    format_menu_line(l0,sizeof(l0),menu_view_top,
                     cursorVisible && menu_idx==menu_view_top);
    format_menu_line(l1,sizeof(l1),menu_view_top+1,
                     cursorVisible && menu_idx==menu_view_top+1);

    lcd_line0(l0);
    lcd_line1(l1);
}

/* ---------------- MANUAL ---------------- */

static void show_manual(void){
    lcd_line0("Manual Mode");
    lcd_line1(Motor_GetStatus() ? "val:STOP    Next>" :
                                  "val:START   Next>");
}

/* ---------------- AUTO ---------------- */

static void show_auto_menu(void){
    lcd_line0("Auto Settings");
    lcd_line1(">Gap/Max/Retry");
}

static void show_auto_gap(void){
    lcd_line0("DRY GAP (s)");
    lcd_val_next(edit_auto_gap_s);
}

static void show_auto_maxrun(void){
    lcd_line0("MAX RUN (min)");
    lcd_val_next(edit_auto_maxrun_min);
}

static void show_auto_retry(void){
    lcd_line0("RETRY COUNT");
    lcd_val_next(edit_auto_retry);
}

/* ---------------- SEMI AUTO ---------------- */

static void show_semi_auto(void){
    lcd_line0("Semi-Auto");
    lcd_line1(semiAutoActive ? "val:Disable Next>" :
                               "val:Enable  Next>");
}

/* ---------------- TIMER ---------------- */

static void show_timer(void){
    char l0[17];
    TimerSlot* t = &timerSlots[currentSlot];
    snprintf(l0,sizeof(l0),"Slot %d %02d:%02d-%02d:%02d",
             currentSlot+1,
             t->onHour,t->onMinute,
             t->offHour,t->offMinute);
    lcd_line0(l0);
    lcd_line1("val:Edit   Next>");
}

/* ---------------- COUNTDOWN ---------------- */

static void show_countdown(void){
    char l0[17], l1[17];

    if (countdownActive){
        uint32_t sec = countdownDuration;
        uint32_t min = sec/60;
        uint32_t s   = sec%60;
        snprintf(l0,sizeof(l0),"CD %02u:%02u RUN",min,s);
        snprintf(l1,sizeof(l1),"Press to STOP");
    }
    else{
        snprintf(l0,sizeof(l0),"CD Set:%3u min",edit_countdown_min);
        snprintf(l1,sizeof(l1),"Press to START");
    }
    lcd_line0(l0);
    lcd_line1(l1);
}

/* ---------------- TWIST ---------------- */

static void show_twist(void){
    char l0[17];
    snprintf(l0,sizeof(l0),"Tw %02ds/%02ds",
             twistSettings.onDurationSeconds,
             twistSettings.offDurationSeconds);
    lcd_line0(l0);
    lcd_line1(twistActive ? "val:STOP   Next>" :
                            "val:START  Next>");
}

/* PART A END */
/***************************************************************
 *  SCREEN.C — PART B OF 3
 *  Editing Logic + Menu Transitions
 ***************************************************************/

/* ================================================================
   TIMER EDIT APPLY
   ================================================================ */

static void apply_timer_settings(void){
    TimerSlot *t = &timerSlots[currentSlot];
    t->enabled  = true;
    t->onHour   = edit_timer_on_h;
    t->onMinute = edit_timer_on_m;
    t->offHour  = edit_timer_off_h;
    t->offMinute= edit_timer_off_m;

    extern void ModelHandle_TimerRecalculateNow(void);
    ModelHandle_TimerRecalculateNow();
}

/* RENDER */

static void show_timer_on_h(void){
    lcd_line0("TIMER ON HOUR");
    lcd_val_next(edit_timer_on_h);
}

static void show_timer_on_m(void){
    lcd_line0("TIMER ON MIN");
    lcd_val_next(edit_timer_on_m);
}

static void show_timer_off_h(void){
    lcd_line0("TIMER OFF HOUR");
    lcd_val_next(edit_timer_off_h);
}

static void show_timer_off_m(void){
    lcd_line0("TIMER OFF MIN");
    lcd_val_next(edit_timer_off_m);
}

/* ================================================================
   AUTO SETTINGS APPLY
   ================================================================ */

static void apply_auto_settings(void){
    ModelHandle_SetAutoSettings(
        edit_auto_gap_s,
        edit_auto_maxrun_min,
        edit_auto_retry
    );
}

/* ================================================================
   TWIST SETTINGS APPLY
   ================================================================ */

static void apply_twist_settings(void){
    twistSettings.onDurationSeconds  = edit_twist_on_s;
    twistSettings.offDurationSeconds = edit_twist_off_s;

    twistSettings.onHour   = edit_twist_on_hh;
    twistSettings.onMinute = edit_twist_on_mm;
    twistSettings.offHour  = edit_twist_off_hh;
    twistSettings.offMinute= edit_twist_off_mm;
}

/* RENDER */

static void show_twist_on_sec(void){
    lcd_line0("TWIST ON SEC");
    lcd_val_next(edit_twist_on_s);
}

static void show_twist_off_sec(void){
    lcd_line0("TWIST OFF SEC");
    lcd_val_next(edit_twist_off_s);
}

static void show_twist_on_h(void){
    lcd_line0("TWIST ON HH");
    lcd_val_next(edit_twist_on_hh);
}

static void show_twist_on_m(void){
    lcd_line0("TWIST ON MM");
    lcd_val_next(edit_twist_on_mm);
}

static void show_twist_off_h(void){
    lcd_line0("TWIST OFF HH");
    lcd_val_next(edit_twist_off_hh);
}

static void show_twist_off_m(void){
    lcd_line0("TWIST OFF MM");
    lcd_val_next(edit_twist_off_mm);
}

/* ================================================================
   COUNTDOWN SETTINGS APPLY
   ================================================================ */

static void apply_countdown_settings(void){
    countdownDuration = (uint32_t)edit_countdown_min * 60u;
}

static void show_countdown_edit_min(void){
    lcd_line0("SET MINUTES");
    lcd_val_next(edit_countdown_min);
}

/* ================================================================
   EDIT VALUE INCREASE
   ================================================================ */

static void increase_edit_value(void){
    switch(ui)
    {
        case UI_TIMER_EDIT_SLOT_ON_H:
            edit_timer_on_h = (edit_timer_on_h + 1) % 24;
            break;

        case UI_TIMER_EDIT_SLOT_ON_M:
            edit_timer_on_m = (edit_timer_on_m + 1) % 60;
            break;

        case UI_TIMER_EDIT_SLOT_OFF_H:
            edit_timer_off_h = (edit_timer_off_h + 1) % 24;
            break;

        case UI_TIMER_EDIT_SLOT_OFF_M:
            edit_timer_off_m = (edit_timer_off_m + 1) % 60;
            break;

        case UI_AUTO_EDIT_GAP:
            edit_auto_gap_s++;
            break;

        case UI_AUTO_EDIT_MAXRUN:
            edit_auto_maxrun_min++;
            break;

        case UI_AUTO_EDIT_RETRY:
            edit_auto_retry++;
            break;

        case UI_TWIST_EDIT_ON:
            if (++edit_twist_on_s > 999) edit_twist_on_s = 0;
            break;

        case UI_TWIST_EDIT_OFF:
            if (++edit_twist_off_s > 999) edit_twist_off_s = 0;
            break;

        case UI_TWIST_EDIT_ON_H:
            edit_twist_on_hh = (edit_twist_on_hh + 1) % 24;
            break;

        case UI_TWIST_EDIT_ON_M:
            edit_twist_on_mm = (edit_twist_on_mm + 1) % 60;
            break;

        case UI_TWIST_EDIT_OFF_H:
            edit_twist_off_hh = (edit_twist_off_hh + 1) % 24;
            break;

        case UI_TWIST_EDIT_OFF_M:
            edit_twist_off_mm = (edit_twist_off_mm + 1) % 60;
            break;

        case UI_COUNTDOWN_EDIT_MIN:
            if (++edit_countdown_min > 999)
                edit_countdown_min = 1;
            break;

        default:
            break;
    }
}

/* ================================================================
   EDIT VALUE DECREASE
   ================================================================ */

static void decrease_edit_value(void){
    switch(ui)
    {
        case UI_TIMER_EDIT_SLOT_ON_H:
            edit_timer_on_h = (edit_timer_on_h==0)?23:edit_timer_on_h-1;
            break;

        case UI_TIMER_EDIT_SLOT_ON_M:
            edit_timer_on_m = (edit_timer_on_m==0)?59:edit_timer_on_m-1;
            break;

        case UI_TIMER_EDIT_SLOT_OFF_H:
            edit_timer_off_h = (edit_timer_off_h==0)?23:edit_timer_off_h-1;
            break;

        case UI_TIMER_EDIT_SLOT_OFF_M:
            edit_timer_off_m = (edit_timer_off_m==0)?59:edit_timer_off_m-1;
            break;

        case UI_AUTO_EDIT_GAP:
            if (edit_auto_gap_s>0) edit_auto_gap_s--;
            break;

        case UI_AUTO_EDIT_MAXRUN:
            if (edit_auto_maxrun_min>0) edit_auto_maxrun_min--;
            break;

        case UI_AUTO_EDIT_RETRY:
            if (edit_auto_retry>0) edit_auto_retry--;
            break;

        case UI_TWIST_EDIT_ON:
            edit_twist_on_s = (edit_twist_on_s==0)?999:edit_twist_on_s-1;
            break;

        case UI_TWIST_EDIT_OFF:
            edit_twist_off_s = (edit_twist_off_s==0)?999:edit_twist_off_s-1;
            break;

        case UI_TWIST_EDIT_ON_H:
            edit_twist_on_hh = (edit_twist_on_hh==0)?23:edit_twist_on_hh-1;
            break;

        case UI_TWIST_EDIT_ON_M:
            edit_twist_on_mm = (edit_twist_on_mm==0)?59:edit_twist_on_mm-1;
            break;

        case UI_TWIST_EDIT_OFF_H:
            edit_twist_off_hh = (edit_twist_off_hh==0)?23:edit_twist_off_hh-1;
            break;

        case UI_TWIST_EDIT_OFF_M:
            edit_twist_off_mm = (edit_twist_off_mm==0)?59:edit_twist_off_mm-1;
            break;

        case UI_COUNTDOWN_EDIT_MIN:
            edit_countdown_min = (edit_countdown_min==1)?999:edit_countdown_min-1;
            break;

        default:
            break;
    }
}

/* ================================================================
   MENU SELECT (SW2 Short)
   ================================================================ */

static void goto_menu_top(void){
    menu_idx = 0;
    menu_view_top = 0;
}

static void menu_select(void){
    refreshInactivityTimer();

    switch(ui)
    {
        case UI_WELCOME:
            ui = UI_DASH;
            screenNeedsRefresh = true;
            return;

        case UI_DASH:
            ui = UI_MENU;
            goto_menu_top();
            screenNeedsRefresh = true;
            return;

        case UI_MENU:
            switch(menu_idx)
            {
                case 0: ui = UI_TIMER; currentSlot = 0; break;
                case 1: ui = UI_AUTO_MENU; break;
                case 2: ui = UI_TWIST; break;
                case 3: ui = UI_DASH; break;
                case 4: ui = UI_DASH; break;
            }
            screenNeedsRefresh = true;
            return;

        /* TIMER EDIT FLOW */
        case UI_TIMER:
            ui = UI_TIMER_EDIT_SLOT_ON_H; screenNeedsRefresh = true; return;

        case UI_TIMER_EDIT_SLOT_ON_H:
            ui = UI_TIMER_EDIT_SLOT_ON_M; screenNeedsRefresh = true; return;

        case UI_TIMER_EDIT_SLOT_ON_M:
            ui = UI_TIMER_EDIT_SLOT_OFF_H; screenNeedsRefresh = true; return;

        case UI_TIMER_EDIT_SLOT_OFF_H:
            ui = UI_TIMER_EDIT_SLOT_OFF_M; screenNeedsRefresh = true; return;

        case UI_TIMER_EDIT_SLOT_OFF_M:
            apply_timer_settings();
            ui = UI_TIMER;
            screenNeedsRefresh = true;
            return;

        /* AUTO EDIT FLOW */
        case UI_AUTO_MENU:
            ui = UI_AUTO_EDIT_GAP; screenNeedsRefresh = true; return;

        case UI_AUTO_EDIT_GAP:
            ui = UI_AUTO_EDIT_MAXRUN; screenNeedsRefresh = true; return;

        case UI_AUTO_EDIT_MAXRUN:
            ui = UI_AUTO_EDIT_RETRY; screenNeedsRefresh = true; return;

        case UI_AUTO_EDIT_RETRY:
            apply_auto_settings();
            ui = UI_AUTO_MENU;
            screenNeedsRefresh = true;
            return;

        /* TWIST EDIT FLOW */
        case UI_TWIST:
            ui = UI_TWIST_EDIT_ON; screenNeedsRefresh = true; return;

        case UI_TWIST_EDIT_ON:
            ui = UI_TWIST_EDIT_OFF; screenNeedsRefresh = true; return;

        case UI_TWIST_EDIT_OFF:
            ui = UI_TWIST_EDIT_ON_H; screenNeedsRefresh = true; return;

        case UI_TWIST_EDIT_ON_H:
            ui = UI_TWIST_EDIT_ON_M; screenNeedsRefresh = true; return;

        case UI_TWIST_EDIT_ON_M:
            ui = UI_TWIST_EDIT_OFF_H; screenNeedsRefresh = true; return;

        case UI_TWIST_EDIT_OFF_H:
            ui = UI_TWIST_EDIT_OFF_M; screenNeedsRefresh = true; return;

        case UI_TWIST_EDIT_OFF_M:
            apply_twist_settings();
            ui = UI_TWIST;
            screenNeedsRefresh = true;
            return;

        /* COUNTDOWN EDIT FLOW */
        case UI_COUNTDOWN:
            if (countdownActive){
                ModelHandle_StopCountdown();
                screenNeedsRefresh = true;
            } else {
                ui = UI_COUNTDOWN_EDIT_MIN;
                screenNeedsRefresh = true;
            }
            return;

        case UI_COUNTDOWN_EDIT_MIN:
            apply_countdown_settings();
            ui = UI_COUNTDOWN;
            screenNeedsRefresh = true;
            return;

        default:
            return;
    }
}

/* ================================================================
   MENU BACK (SW1 Short)
   ================================================================ */

static void menu_reset(void){
    refreshInactivityTimer();

    switch(ui)
    {
        case UI_MENU: ui = UI_DASH; break;
        case UI_DASH: ui = UI_WELCOME; break;

        case UI_TIMER:
        case UI_AUTO_MENU:
        case UI_TWIST:
        case UI_COUNTDOWN:
        case UI_SEMI_AUTO:
        case UI_MANUAL:
            ui = UI_MENU;
            break;

        case UI_TIMER_EDIT_SLOT_ON_H:
        case UI_TIMER_EDIT_SLOT_ON_M:
        case UI_TIMER_EDIT_SLOT_OFF_H:
        case UI_TIMER_EDIT_SLOT_OFF_M:
        case UI_TIMER_EDIT_SLOT_ENABLE:
            ui = UI_TIMER;
            break;

        case UI_AUTO_EDIT_GAP:
        case UI_AUTO_EDIT_MAXRUN:
        case UI_AUTO_EDIT_RETRY:
            ui = UI_AUTO_MENU;
            break;

        case UI_TWIST_EDIT_ON:
        case UI_TWIST_EDIT_OFF:
        case UI_TWIST_EDIT_ON_H:
        case UI_TWIST_EDIT_ON_M:
        case UI_TWIST_EDIT_OFF_H:
        case UI_TWIST_EDIT_OFF_M:
            ui = UI_TWIST;
            break;

        case UI_COUNTDOWN_EDIT_MIN:
        case UI_COUNTDOWN_TOGGLE:
            ui = UI_COUNTDOWN;
            break;

        default:
            ui = UI_MENU;
            break;
    }

    screenNeedsRefresh = true;
}

/* END OF PART B */
/***************************************************************
 *  SCREEN.C — PART C OF 3
 *  Button Handling + LCD Update Engine
 ***************************************************************/

/* ================================================================
   BUTTON DECODER (WITH FIXED LONG-PRESS EDGE CASES)
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
        /* --------------------------------------------------------
           PRESS START
           -------------------------------------------------------- */
        if (sw[i] && sw_press_start[i] == 0)
        {
            sw_press_start[i] = now;
            sw_long_issued[i] = false;
        }
        /* --------------------------------------------------------
           RELEASE → SHORT PRESS IF NO LONG WAS ISSUED
           -------------------------------------------------------- */
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
        /* --------------------------------------------------------
           HOLD → LONG PRESS
           -------------------------------------------------------- */
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

/* ================================================================
   MOTOR RESTART HANDLING
   ================================================================ */

static bool restartingMotor = false;
static uint32_t restartDeadline = 0;

/* ================================================================
   MAIN SWITCH HANDLER
   ================================================================ */

/***************************************************************
 *   FINAL Screen_HandleSwitches() — FULL VERSION
 *   Supports:
 *   ✔ Short Press SW4 → Start/Stop Countdown
 *   ✔ Long Press SW4  → Increase Countdown minutes
 *   ✔ Works inside Dashboard + Countdown + Edit screen
 *   ✔ Does NOT affect Twist Mode logic
 ***************************************************************/
void Screen_HandleSwitches(void)
{
    UiButton b = BTN_NONE;

    /* Map switch events to UiButton */
    SwitchEvent e0 = Switch_GetEvent(0);
    SwitchEvent e1 = Switch_GetEvent(1);
    SwitchEvent e2 = Switch_GetEvent(2);
    SwitchEvent e3 = Switch_GetEvent(3);

    if (e0 == SWITCH_EVT_SHORT)      b = BTN_RESET;
    else if (e0 == SWITCH_EVT_LONG)  b = BTN_RESET_LONG;

    else if (e1 == SWITCH_EVT_SHORT)     b = BTN_SELECT;
    else if (e1 == SWITCH_EVT_LONG)      b = BTN_SELECT_LONG;

    else if (e2 == SWITCH_EVT_SHORT)     b = BTN_UP;
    else if (e2 == SWITCH_EVT_LONG)      b = BTN_UP_LONG;

    else if (e3 == SWITCH_EVT_SHORT)     b = BTN_DOWN;
    else if (e3 == SWITCH_EVT_LONG)      b = BTN_DOWN_LONG;

    if (b == BTN_NONE) return;

    refreshInactivityTimer();

    /***************************************************************
     * ----------------- HANDLE BUTTON EVENTS ---------------------
     ***************************************************************/

    switch (b)
    {

    /* ============================================================
       SW1 (RED) → BACK / EXIT
       ============================================================ */
    case BTN_RESET:
        if (ui == UI_MENU ||
            ui == UI_TIMER ||
            ui == UI_TIMER_SLOT_SELECT ||
            ui == UI_AUTO_MENU ||
            ui == UI_TWIST ||
            ui == UI_SEMI_AUTO ||
            ui == UI_MANUAL ||
            ui == UI_COUNTDOWN ||
            ui == UI_COUNTDOWN_EDIT_MIN)
        {
            ui = UI_DASH;
            screenNeedsRefresh = true;
        }
        break;


    /* ============================================================
       SW1 LONG PRESS → MANUAL MODE TOGGLE
       ============================================================ */
    case BTN_RESET_LONG:
        ModelHandle_ToggleManual();
        ui = UI_MANUAL;
        screenNeedsRefresh = true;
        break;


    /* ============================================================
       SW2 SHORT (Yellow P) → SELECT / ENTER
       ============================================================ */
    case BTN_SELECT:
        if (ui == UI_DASH) {
            ui = UI_MENU;
            menu_idx = 0;
            screenNeedsRefresh = true;
        }
        else if (ui == UI_MENU) {
            if (menu_idx == 0) ui = UI_TIMER;
            else if (menu_idx == 1) ui = UI_AUTO_MENU;
            else if (menu_idx == 2) ui = UI_TWIST;
            else if (menu_idx == 3) ui = UI_SEMI_AUTO;
            else if (menu_idx == 4) ui = UI_DASH;
            screenNeedsRefresh = true;
        }
        break;


    /* ============================================================
       SW2 LONG PRESS → OPEN MAIN MENU
       ============================================================ */
    case BTN_SELECT_LONG:
        ui = UI_MENU;
        menu_idx = 0;
        screenNeedsRefresh = true;
        break;


    /* ============================================================
       SW3 SHORT → NAVIGATE / INCREASE GENERAL VALUES
       ============================================================ */
    case BTN_UP:
        switch (ui)
        {
            case UI_MENU:
                if (menu_idx > 0) menu_idx--;
                screenNeedsRefresh = true;
                break;

            case UI_COUNTDOWN_EDIT_MIN:
                edit_countdown_min = (edit_countdown_min + 1) % 999;
                screenNeedsRefresh = true;
                break;

            default:
                break;
        }
        break;


    /* ============================================================
       SW3 LONG → CONTINUOUS INCREMENT
       ============================================================ */
    case BTN_UP_LONG:
        if (ui == UI_COUNTDOWN_EDIT_MIN) {
            if (HAL_GetTick() - last_repeat_time > COUNTDOWN_INC_MS) {
                edit_countdown_min = (edit_countdown_min + 1) % 999;
                last_repeat_time = HAL_GetTick();
                screenNeedsRefresh = true;
            }
        }
        break;


    /* ============================================================
       SW4 SHORT (DOWN) → START / STOP COUNTDOWN MODE
       ============================================================ */
    case BTN_DOWN:

        /*********** From Dashboard → enter CD mode ***********/
        if (ui == UI_DASH) {
            ui = UI_COUNTDOWN;

            if (!countdownActive)
                ModelHandle_StartCountdown(edit_countdown_min * 60);
            else
                ModelHandle_StopCountdown();

            screenNeedsRefresh = true;
            break;
        }

        /*********** Inside Countdown Screen ***********/
        if (ui == UI_COUNTDOWN) {
            if (countdownActive)
                ModelHandle_StopCountdown();
            else
                ModelHandle_StartCountdown(edit_countdown_min * 60);

            screenNeedsRefresh = true;
            break;
        }

        /*********** Inside Countdown Edit ***********/
        if (ui == UI_COUNTDOWN_EDIT_MIN) {
            edit_countdown_min = (edit_countdown_min + 1) % 999;
            screenNeedsRefresh = true;
        }

        break;


    /* ============================================================
       SW4 LONG (DOWN LONG PRESS)
       → ENTER EDIT + CONTINUOUS INCREMENT
       ============================================================ */
    case BTN_DOWN_LONG:

        /*********** Enter Edit Mode from RUN or IDLE ***********/
        if (ui == UI_COUNTDOWN || ui == UI_DASH) {
            ui = UI_COUNTDOWN_EDIT_MIN;
            screenNeedsRefresh = true;
            last_repeat_time = HAL_GetTick();
        }

        /*********** Continuous increment ***********/
        if (ui == UI_COUNTDOWN_EDIT_MIN) {
            if (HAL_GetTick() - last_repeat_time > COUNTDOWN_INC_MS) {
                edit_countdown_min = (edit_countdown_min + 1) % 999;
                last_repeat_time = HAL_GetTick();
                screenNeedsRefresh = true;
            }
        }

        break;
    }

}

/* ================================================================
   LCD UPDATE ENGINE (REFRESH + BLINK + AUTO BACK)
   ================================================================ */

void Screen_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* Cursor blink modes */
    bool cursorBlinkActive = false;
    switch(ui)
    {
        case UI_MENU:
        case UI_TIMER_EDIT_SLOT_ON_H:
        case UI_TIMER_EDIT_SLOT_ON_M:
        case UI_TIMER_EDIT_SLOT_OFF_H:
        case UI_TIMER_EDIT_SLOT_OFF_M:
        case UI_COUNTDOWN_EDIT_MIN:
        case UI_TWIST_EDIT_ON:
        case UI_TWIST_EDIT_OFF:
        case UI_TWIST_EDIT_ON_H:
        case UI_TWIST_EDIT_ON_M:
        case UI_TWIST_EDIT_OFF_H:
        case UI_TWIST_EDIT_OFF_M:
        case UI_AUTO_EDIT_GAP:
        case UI_AUTO_EDIT_MAXRUN:
        case UI_AUTO_EDIT_RETRY:
            cursorBlinkActive = true;
            break;

        default:
            cursorBlinkActive = false;
            cursorVisible = true;
            break;
    }

    /* Cursor blink */
    if (cursorBlinkActive && (now - lastCursorToggle >= CURSOR_BLINK_MS))
    {
        cursorVisible = !cursorVisible;
        lastCursorToggle = now;
        screenNeedsRefresh = true;
    }

    /* Auto-transition from WELCOME */
    if (ui == UI_WELCOME && now - lastLcdUpdateTime >= WELCOME_MS)
    {
        ui = UI_DASH;
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }

    /* Auto-return to DASH after inactivity */
    if (ui != UI_WELCOME &&
        ui != UI_DASH &&
        now - lastUserAction >= AUTO_BACK_MS)
    {
        ui = UI_DASH;
        screenNeedsRefresh = true;
    }

    /* Refresh DASH every second */
    if (ui == UI_DASH && (now - lastLcdUpdateTime >= 1000))
    {
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }

    /* ============================================================
       DO THE DRAWING
       ============================================================ */
    if (screenNeedsRefresh || ui != last_ui)
    {
        bool fullRefresh = (ui != last_ui);
        last_ui = ui;
        screenNeedsRefresh = false;

        if (fullRefresh)
            lcd_clear();

        switch(ui)
        {
            case UI_WELCOME:                show_welcome(); break;
            case UI_DASH:                   show_dash(); break;

            case UI_MENU:                   show_menu(); break;

            case UI_MANUAL:                 show_manual(); break;
            case UI_SEMI_AUTO:              show_semi_auto(); break;

            case UI_TIMER:                  show_timer(); break;
            case UI_TIMER_EDIT_SLOT_ON_H:   show_timer_on_h(); break;
            case UI_TIMER_EDIT_SLOT_ON_M:   show_timer_on_m(); break;
            case UI_TIMER_EDIT_SLOT_OFF_H:  show_timer_off_h(); break;
            case UI_TIMER_EDIT_SLOT_OFF_M:  show_timer_off_m(); break;

            case UI_AUTO_MENU:              show_auto_menu(); break;
            case UI_AUTO_EDIT_GAP:          show_auto_gap(); break;
            case UI_AUTO_EDIT_MAXRUN:       show_auto_maxrun(); break;
            case UI_AUTO_EDIT_RETRY:        show_auto_retry(); break;

            case UI_COUNTDOWN:              show_countdown(); break;
            case UI_COUNTDOWN_EDIT_MIN:     show_countdown_edit_min(); break;

            case UI_TWIST:                  show_twist(); break;
            case UI_TWIST_EDIT_ON:          show_twist_on_sec(); break;
            case UI_TWIST_EDIT_OFF:         show_twist_off_sec(); break;
            case UI_TWIST_EDIT_ON_H:        show_twist_on_h(); break;
            case UI_TWIST_EDIT_ON_M:        show_twist_on_m(); break;
            case UI_TWIST_EDIT_OFF_H:       show_twist_off_h(); break;
            case UI_TWIST_EDIT_OFF_M:       show_twist_off_m(); break;

            default:
                lcd_line0("Not Impl");
                lcd_line1(" ");
                break;
        }
    }
}

/***************************************************************
 *  END OF PART C — FULL SCREEN.C COMPLETE
 ***************************************************************/
