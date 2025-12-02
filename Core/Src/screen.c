/***************************************************************
 *  HELONIX Water Pump Controller
 *  SCREEN.C — FINAL FULLY UPDATED VERSION (2025)
 *
 *  PART 1 / 4
 *  - Enums
 *  - UI Globals
 *  - LCD Helpers
 *  - Welcome Screen
 *  - Dashboard Screen
 *  - Menu Screen
 *
 *  Button logic fully aligned with HELONIX Spec:
 *      SW1 SHORT  → Restart pump
 *      SW1 LONG   → Toggle MANUAL Mode
 *      SW2 SHORT  → Toggle AUTO Mode
 *      SW2 LONG   → Open MENU
 *      SW3 SHORT  → Activate nearest TIMER SLOT (Option B)
 *      SW3 LONG   → Toggle SEMI-AUTO
 *      SW4 SHORT  → Start/Stop COUNTDOWN
 *      SW4 LONG   → Increase countdown every 3 sec
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

    UI_MAX_
} UiState;

typedef enum {
    BTN_NONE = 0,
    BTN_RESET,        // SW1 short
    BTN_SELECT,       // SW2 short
    BTN_UP,           // SW3 short
    BTN_DOWN,         // SW4 short

    BTN_RESET_LONG,   // SW1 long
    BTN_SELECT_LONG,  // SW2 long
    BTN_UP_LONG,      // SW3 long
    BTN_DOWN_LONG     // SW4 long
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
#define COUNTDOWN_INC_MS      3000    // per spec: every 3 sec

/* Button press trackers */
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
   TEMP EDIT VARIABLES (UI Work Buffers)
   ================================================================ */

static uint8_t  edit_timer_on_h  = 0,  edit_timer_on_m  = 0;
static uint8_t  edit_timer_off_h = 0, edit_timer_off_m = 0;

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
    snprintf(buf, sizeof(buf), "val:%03u   Next>", v);
    lcd_line1(buf);
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

    for (int i = 0; i < 4; i++){
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
   MAIN DASH SCREEN + WELCOME
   ================================================================ */

static void show_welcome(void){
    lcd_clear();
    lcd_line0("   HELONIX");
    lcd_line1(" IntelligentSys");
}

static void show_dash(void){
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

    /* Water level */
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
   MENU RENDERING
   ================================================================ */

static void format_menu_line(char* buf,size_t size,int idx,bool sel){
    if (idx < 0 || idx >= MAIN_MENU_COUNT){
        snprintf(buf,size,"                ");
        return;
    }
    snprintf(buf,size,"%c%-15.15s", sel?'>':' ', main_menu[idx]);
}

static void show_menu(void){
    char l0[17], l1[17];

    if (menu_idx < menu_view_top)
        menu_view_top = menu_idx;
    else if (menu_idx > menu_view_top+1)
        menu_view_top = menu_idx - 1;

    format_menu_line(l0, sizeof(l0), menu_view_top,
                     cursorVisible && menu_idx==menu_view_top);

    format_menu_line(l1, sizeof(l1), menu_view_top+1,
                     cursorVisible && menu_idx==menu_view_top+1);

    lcd_line0(l0);
    lcd_line1(l1);
}
static void show_manual(void){
    lcd_line0("Manual Mode");
    lcd_line1(Motor_GetStatus() ? "val:STOP   Next>"
                                : "val:START  Next>");
}

/* ---------------- TIMER SCREEN ---------------- */

static void show_timer(void){
    char l0[17];

    TimerSlot *t = &timerSlots[currentSlot];

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

/* ---------------- AUTO MODE UI ---------------- */

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

/* ---------------- SEMI AUTO SCREEN ---------------- */

static void show_semi_auto(void){
    lcd_line0("Semi-Auto");
    lcd_line1(semiAutoActive ? "val:Disable Next>"
                             : "val:Enable  Next>");
}

/* ---------------- TWIST MODE UI ---------------- */

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

/* ---------------- COUNTDOWN UI ---------------- */

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
    lcd_val_next(edit_countdown_min);
}

/* ================================================================
   APPLY FUNCTIONS — TIMER / AUTO / TWIST / COUNTDOWN
   ================================================================ */

static void apply_timer_settings(void){
    TimerSlot *t = &timerSlots[currentSlot];
    t->enabled  = true;
    t->onHour   = edit_timer_on_h;
    t->onMinute = edit_timer_on_m;
    t->offHour  = edit_timer_off_h;
    t->offMinute= edit_timer_off_m;

    /* ⬇ Option-B: Recalculate & activate timer engine instantly */
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
        edit_countdown_min = 1;     // Safety: no zero countdown

    countdownDuration = (uint32_t)edit_countdown_min * 60u;
}
/***************************************************************
 *  SCREEN.C — PART 3 / 4
 *  Button Handling + Mode Logic
 ***************************************************************/

/* ================================================================
   BUTTON DECODER — robust short/long press detection
   ================================================================ */

static UiButton decode_button_press(void){
    bool sw1 = Switch_IsPressed(0);     // RED
    bool sw2 = Switch_IsPressed(1);     // YELLOW "P"
    bool sw3 = Switch_IsPressed(2);     // UP
    bool sw4 = Switch_IsPressed(3);     // DOWN

    bool sw[4] = {sw1, sw2, sw3, sw4};

    uint32_t now = HAL_GetTick();
    UiButton out = BTN_NONE;

    for(int i = 0; i < 4; i++)
    {
        /* Start of press */
        if(sw[i] && sw_press_start[i] == 0){
            sw_press_start[i] = now;
            sw_long_issued[i] = false;
        }

        /* Release → short press */
        else if(!sw[i] && sw_press_start[i] != 0){
            if(!sw_long_issued[i]){
                switch(i){
                    case 0: out = BTN_RESET;       break;  // SW1 short
                    case 1: out = BTN_SELECT;      break;  // SW2 short
                    case 2: out = BTN_UP;          break;  // SW3 short
                    case 3: out = BTN_DOWN;        break;  // SW4 short
                }
            }
            sw_press_start[i] = 0;
            sw_long_issued[i] = false;
        }

        /* Hold long enough → long press */
        else if(sw[i] && !sw_long_issued[i]){
            if(now - sw_press_start[i] >= LONG_PRESS_MS){
                sw_long_issued[i] = true;
                switch(i){
                    case 0: out = BTN_RESET_LONG;  break;  // Manual toggle
                    case 1: out = BTN_SELECT_LONG; break;  // Open menu
                    case 2: out = BTN_UP_LONG;     break;  // Semi-auto toggle
                    case 3: out = BTN_DOWN_LONG;   break;  // Countdown increment
                }
            }
        }
    }

    return out;
}

/* ================================================================
   MENU ENTER / FLOW CONTROL
   ================================================================ */

static void goto_menu_top(void){
    menu_idx = 0;
    menu_view_top = 0;
}

static void menu_select(void){
    refreshInactivityTimer();

    /* WELCOME → DASH */
    if(ui == UI_WELCOME){
        ui = UI_DASH;
        screenNeedsRefresh = true;
        return;
    }

    /* DASH → MENU */
    if(ui == UI_DASH){
        ui = UI_MENU;
        goto_menu_top();
        screenNeedsRefresh = true;
        return;
    }

    /* MAIN MENU */
    if(ui == UI_MENU){
        switch(menu_idx)
        {
            case 0:     ui = UI_TIMER; currentSlot = 0; break;
            case 1:     ui = UI_AUTO_MENU; break;
            case 2:     ui = UI_TWIST; break;
            case 3:     ui = UI_DASH; break;  // SETTINGS HIDDEN HERE
            case 4:     ui = UI_DASH; break;  // BACK
        }
        screenNeedsRefresh = true;
        return;
    }

    /* ---------------- TIMER MODE EDIT FLOW ---------------- */

    if(ui == UI_TIMER){
        ui = UI_TIMER_EDIT_SLOT_ON_H;
        screenNeedsRefresh = true;
        return;
    }

    if(ui == UI_TIMER_EDIT_SLOT_ON_H){
        ui = UI_TIMER_EDIT_SLOT_ON_M;
        screenNeedsRefresh = true;
        return;
    }

    if(ui == UI_TIMER_EDIT_SLOT_ON_M){
        ui = UI_TIMER_EDIT_SLOT_OFF_H;
        screenNeedsRefresh = true;
        return;
    }

    if(ui == UI_TIMER_EDIT_SLOT_OFF_H){
        ui = UI_TIMER_EDIT_SLOT_OFF_M;
        screenNeedsRefresh = true;
        return;
    }

    if(ui == UI_TIMER_EDIT_SLOT_OFF_M){
        apply_timer_settings();
        ui = UI_TIMER;
        screenNeedsRefresh = true;
        return;
    }

    /* ---------------- AUTO MODE EDIT FLOW ---------------- */

    if(ui == UI_AUTO_MENU){
        ui = UI_AUTO_EDIT_GAP;
        screenNeedsRefresh = true;
        return;
    }

    if(ui == UI_AUTO_EDIT_GAP){
        ui = UI_AUTO_EDIT_MAXRUN;
        screenNeedsRefresh = true;
        return;
    }

    if(ui == UI_AUTO_EDIT_MAXRUN){
        ui = UI_AUTO_EDIT_RETRY;
        screenNeedsRefresh = true;
        return;
    }

    if(ui == UI_AUTO_EDIT_RETRY){
        apply_auto_settings();
        ui = UI_AUTO_MENU;
        screenNeedsRefresh = true;
        return;
    }

    /* ---------------- TWIST MODE FLOW ---------------- */

    if(ui == UI_TWIST){
        ui = UI_TWIST_EDIT_ON;
        screenNeedsRefresh = true;
        return;
    }

    if(ui == UI_TWIST_EDIT_ON){
        ui = UI_TWIST_EDIT_OFF;
        screenNeedsRefresh = true;
        return;
    }

    if(ui == UI_TWIST_EDIT_OFF){
        ui = UI_TWIST_EDIT_ON_H;
        screenNeedsRefresh = true;
        return;
    }

    if(ui == UI_TWIST_EDIT_ON_H){
        ui = UI_TWIST_EDIT_ON_M;
        screenNeedsRefresh = true;
        return;
    }

    if(ui == UI_TWIST_EDIT_ON_M){
        ui = UI_TWIST_EDIT_OFF_H;
        screenNeedsRefresh = true;
        return;
    }

    if(ui == UI_TWIST_EDIT_OFF_H){
        ui = UI_TWIST_EDIT_OFF_M;
        screenNeedsRefresh = true;
        return;
    }

    if(ui == UI_TWIST_EDIT_OFF_M){
        apply_twist_settings();
        ui = UI_TWIST;
        screenNeedsRefresh = true;
        return;
    }

    /* ---------------- COUNTDOWN MODE FLOW ---------------- */

    if(ui == UI_COUNTDOWN){
        if(countdownActive){
            ModelHandle_StopCountdown();
            screenNeedsRefresh = true;
        }
        else{
            ui = UI_COUNTDOWN_EDIT_MIN;
            screenNeedsRefresh = true;
        }
        return;
    }

    if(ui == UI_COUNTDOWN_EDIT_MIN){
        apply_countdown_settings();
        ui = UI_COUNTDOWN;
        screenNeedsRefresh = true;
        return;
    }
}

/* ================================================================
   MAIN BUTTON HANDLER (HELONIX NORMAL MODE LOGIC)
   ================================================================ */
/* ================================================================
   EDIT VALUE ENGINE (INCREASE / DECREASE)
   ================================================================ */

void increase_edit_value(void)
{
    switch(ui)
    {
        /* ---- TIMER ON/OFF EDIT ---- */
        case UI_TIMER_EDIT_SLOT_ON_H:  if(edit_timer_on_h  < 23) edit_timer_on_h++;  break;
        case UI_TIMER_EDIT_SLOT_ON_M:  if(edit_timer_on_m  < 59) edit_timer_on_m++;  break;
        case UI_TIMER_EDIT_SLOT_OFF_H: if(edit_timer_off_h < 23) edit_timer_off_h++; break;
        case UI_TIMER_EDIT_SLOT_OFF_M: if(edit_timer_off_m < 59) edit_timer_off_m++; break;

        /* ---- AUTO SETTINGS ---- */
        case UI_AUTO_EDIT_GAP:         edit_auto_gap_s++;      break;
        case UI_AUTO_EDIT_MAXRUN:      edit_auto_maxrun_min++; break;
        case UI_AUTO_EDIT_RETRY:       edit_auto_retry++;      break;

        /* ---- TWIST SETTINGS ---- */
        case UI_TWIST_EDIT_ON:         edit_twist_on_s++;      break;
        case UI_TWIST_EDIT_OFF:        edit_twist_off_s++;     break;

        case UI_TWIST_EDIT_ON_H:       if(edit_twist_on_hh  < 23) edit_twist_on_hh++;  break;
        case UI_TWIST_EDIT_ON_M:       if(edit_twist_on_mm  < 59) edit_twist_on_mm++;  break;
        case UI_TWIST_EDIT_OFF_H:      if(edit_twist_off_hh < 23) edit_twist_off_hh++; break;
        case UI_TWIST_EDIT_OFF_M:      if(edit_twist_off_mm < 59) edit_twist_off_mm++; break;

        /* ---- COUNTDOWN ---- */
        case UI_COUNTDOWN_EDIT_MIN:    if(edit_countdown_min < 999) edit_countdown_min++; break;

        default: break;
    }
}

void decrease_edit_value(void)
{
    switch(ui)
    {
        /* ---- TIMER ---- */
        case UI_TIMER_EDIT_SLOT_ON_H:  if(edit_timer_on_h  > 0) edit_timer_on_h--;  break;
        case UI_TIMER_EDIT_SLOT_ON_M:  if(edit_timer_on_m  > 0) edit_timer_on_m--;  break;
        case UI_TIMER_EDIT_SLOT_OFF_H: if(edit_timer_off_h > 0) edit_timer_off_h--; break;
        case UI_TIMER_EDIT_SLOT_OFF_M: if(edit_timer_off_m > 0) edit_timer_off_m--; break;

        /* ---- AUTO SETTINGS ---- */
        case UI_AUTO_EDIT_GAP:         if(edit_auto_gap_s > 0) edit_auto_gap_s--; break;
        case UI_AUTO_EDIT_MAXRUN:      if(edit_auto_maxrun_min > 0) edit_auto_maxrun_min--; break;
        case UI_AUTO_EDIT_RETRY:       if(edit_auto_retry > 0) edit_auto_retry--; break;

        /* ---- TWIST ---- */
        case UI_TWIST_EDIT_ON:         if(edit_twist_on_s > 0) edit_twist_on_s--; break;
        case UI_TWIST_EDIT_OFF:        if(edit_twist_off_s > 0) edit_twist_off_s--; break;

        case UI_TWIST_EDIT_ON_H:       if(edit_twist_on_hh  > 0) edit_twist_on_hh--; break;
        case UI_TWIST_EDIT_ON_M:       if(edit_twist_on_mm  > 0) edit_twist_on_mm--; break;
        case UI_TWIST_EDIT_OFF_H:      if(edit_twist_off_hh > 0) edit_twist_off_hh--; break;
        case UI_TWIST_EDIT_OFF_M:      if(edit_twist_off_mm > 0) edit_twist_off_mm--; break;

        /* ---- COUNTDOWN ---- */
        case UI_COUNTDOWN_EDIT_MIN:    if(edit_countdown_min > 1) edit_countdown_min--; break;

        default: break;
    }
}

void Screen_HandleSwitches(void)
{
    UiButton b = decode_button_press();
    uint32_t now = HAL_GetTick();

    bool sw3 = Switch_IsPressed(2);   // UP
    bool sw4 = Switch_IsPressed(3);   // DOWN

    /* ============================================================
       CONTINUOUS HOLD ENGINE (UP/DOWN) for MENU + EDIT
       ============================================================ */
    if (sw3 && sw_long_issued[2]) {
        if (now - last_repeat_time >= CONTINUOUS_STEP_MS) {
            last_repeat_time = now;

            if (ui == UI_MENU) {
                if (menu_idx > 0) menu_idx--;
            }
            else if (ui != UI_COUNTDOWN) {
                increase_edit_value();
            }

            screenNeedsRefresh = true;
        }
    }

    /* ============================================================
       COUNTDOWN LONG-PRESS SPECIAL MODE (UI_DASH or UI_COUNTDOWN)
       ============================================================ */
    if (sw4 && sw_long_issued[3] && (ui == UI_DASH || ui == UI_COUNTDOWN)) {

        static uint32_t cd_last_inc = 0;

        if (ui == UI_DASH) {
            ui = UI_COUNTDOWN;
            screenNeedsRefresh = true;
        }

        show_countdown();    // show updated live

        if (now - cd_last_inc >= COUNTDOWN_INC_MS) {
            cd_last_inc = now;
            edit_countdown_min++;
            countdownDuration = edit_countdown_min * 60;
            show_countdown();    // refresh again
        }

        return;   // No other handler
    }

    /* Normal continuous DOWN for menu/edit */
    if (sw4 && sw_long_issued[3] == false && ui != UI_DASH && ui != UI_COUNTDOWN) {
        if (now - last_repeat_time >= CONTINUOUS_STEP_MS) {
            last_repeat_time = now;

            if (ui == UI_MENU) {
                if (menu_idx < MAIN_MENU_COUNT - 1) menu_idx++;
            } else {
                decrease_edit_value();
            }

            screenNeedsRefresh = true;
        }
    }

    /* No event */
    if (b == BTN_NONE)
        return;

    refreshInactivityTimer();


    /* ============================================================
       EDIT MODE
       ============================================================ */
    bool editing =
        (ui == UI_TIMER_EDIT_SLOT_ON_H || ui == UI_TIMER_EDIT_SLOT_ON_M ||
         ui == UI_TIMER_EDIT_SLOT_OFF_H || ui == UI_TIMER_EDIT_SLOT_OFF_M ||
         ui == UI_AUTO_EDIT_GAP || ui == UI_AUTO_EDIT_MAXRUN ||
         ui == UI_AUTO_EDIT_RETRY || ui == UI_TWIST_EDIT_ON ||
         ui == UI_TWIST_EDIT_OFF || ui == UI_TWIST_EDIT_ON_H ||
         ui == UI_TWIST_EDIT_ON_M || ui == UI_TWIST_EDIT_OFF_H ||
         ui == UI_TWIST_EDIT_OFF_M || ui == UI_COUNTDOWN_EDIT_MIN);

    if (editing)
    {
        switch (b)
        {
        case BTN_UP:
            increase_edit_value();
            break;

        case BTN_UP_LONG:
            last_repeat_time = now;
            increase_edit_value();
            break;

        case BTN_DOWN:
            decrease_edit_value();
            break;

        case BTN_DOWN_LONG:
            last_repeat_time = now;
            decrease_edit_value();
            break;

        case BTN_RESET:
            ui = last_ui;
            break;

        case BTN_SELECT:
            menu_select();
            break;
        }
        screenNeedsRefresh = true;
        return;
    }

    /* ============================================================
       MENU MODE
       ============================================================ */
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
            menu_select();
            break;

        case BTN_RESET:
            ui = UI_DASH;
            break;
        }
        screenNeedsRefresh = true;
        return;
    }

    /* ============================================================
       DASHBOARD / MAIN LOGIC
       ============================================================ */
    switch (b)
    {
    /* ---------- SW1 RED ---------- */
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


    /* ---------- SW2 YELLOW ---------- */
    case BTN_SELECT:   // SHORT PRESS

        /* Priority: If countdown running → STOP countdown */
        if (countdownActive)
        {
            ModelHandle_StopCountdown();   // motor OFF
            ui = UI_DASH;                  // return to DASH
            screenNeedsRefresh = true;
            return;
        }

        /* Otherwise AUTO toggle */
        if (!autoActive)
            ModelHandle_StartAuto(edit_auto_gap_s, edit_auto_maxrun_min, edit_auto_retry);
        else
            ModelHandle_StopAuto();

        ui = UI_DASH;
        screenNeedsRefresh = true;
        return;


    case BTN_SELECT_LONG:  // OPEN MENU
        ui = UI_MENU;
        menu_idx = 0;
        menu_view_top = 0;
        screenNeedsRefresh = true;
        return;


    /* ---------- SW3 UP ---------- */
    case BTN_UP:   // Timer mode toggle
        if (!timerActive)
            ModelHandle_StartTimerNearestSlot();
        else
            ModelHandle_StopTimer();

        ui = UI_DASH;
        screenNeedsRefresh = true;
        return;

    case BTN_UP_LONG:  // Semi-auto toggle
        if (!semiAutoActive)
            ModelHandle_StartSemiAuto();
        else
            ModelHandle_StopSemiAuto();

        ui = UI_DASH;
        screenNeedsRefresh = true;
        return;


    /* ---------- SW4 DOWN ---------- */
    case BTN_DOWN:    // SHORT PRESS
    {
        /* If long press was happening (release event) */
        if (sw_long_issued[3])
        {
            apply_countdown_settings();
            countdownDuration = edit_countdown_min * 60;
            ModelHandle_StartCountdown(countdownDuration); // Motor ON

            sw_long_issued[3] = false;
            sw_press_start[3] = 0;

            ui = UI_COUNTDOWN;
            screenNeedsRefresh = true;
            return;
        }

        /* Normal short press toggle */
        if (!countdownActive)
        {
            ModelHandle_StartCountdown(edit_countdown_min * 60); // Motor ON
            ui = UI_COUNTDOWN;
        }
        else
        {
            /* STOP countdown */
            ModelHandle_StopCountdown();  // Motor OFF
            ui = UI_DASH;
        }

        screenNeedsRefresh = true;
        return;
    }

    case BTN_DOWN_LONG:
        last_repeat_time = now;
        return;


    default:
        return;
    }
}


/***************************************************************
 *  SCREEN.C — PART 4 / 4
 *  LCD Update Engine + Final Switch Dispatch
 ***************************************************************/

void Screen_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* ------------------------------------------------------------
       Cursor blinking only inside EDIT SCREENS
       ------------------------------------------------------------ */
    bool cursorBlinkActive =
        (ui == UI_MENU ||
         ui == UI_TIMER_EDIT_SLOT_ON_H ||
         ui == UI_TIMER_EDIT_SLOT_ON_M ||
         ui == UI_TIMER_EDIT_SLOT_OFF_H ||
         ui == UI_TIMER_EDIT_SLOT_OFF_M ||
         ui == UI_COUNTDOWN_EDIT_MIN ||
         ui == UI_TWIST_EDIT_ON ||
         ui == UI_TWIST_EDIT_OFF ||
         ui == UI_TWIST_EDIT_ON_H ||
         ui == UI_TWIST_EDIT_ON_M ||
         ui == UI_TWIST_EDIT_OFF_H ||
         ui == UI_TWIST_EDIT_OFF_M ||
         ui == UI_AUTO_EDIT_GAP ||
         ui == UI_AUTO_EDIT_MAXRUN ||
         ui == UI_AUTO_EDIT_RETRY);

    if(cursorBlinkActive && (now - lastCursorToggle >= CURSOR_BLINK_MS)){
        cursorVisible = !cursorVisible;
        lastCursorToggle = now;
        screenNeedsRefresh = true;
    }

    /* ------------------------------------------------------------
       Auto transition from WELCOME → DASHBOARD
       ------------------------------------------------------------ */
    if(ui == UI_WELCOME && now - lastLcdUpdateTime >= WELCOME_MS){
        ui = UI_DASH;
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }

    /* ------------------------------------------------------------
       Auto-return to DASH after inactivity (HELONIX spec)
       ------------------------------------------------------------ */
    if(ui != UI_WELCOME &&
       ui != UI_DASH &&
       now - lastUserAction >= AUTO_BACK_MS)
    {
        ui = UI_DASH;
        screenNeedsRefresh = true;
    }

    /* ------------------------------------------------------------
       DASH refresh every 1 second
       ------------------------------------------------------------ */
    if(ui == UI_DASH && now - lastLcdUpdateTime >= 1000){
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }
    if(ui == UI_COUNTDOWN && now - lastLcdUpdateTime >= 1000){
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }


    /* ------------------------------------------------------------
       PERFORM LCD REFRESH
       ------------------------------------------------------------ */
    if(screenNeedsRefresh || ui != last_ui){
        bool fullRefresh = (ui != last_ui);
        last_ui = ui;
        screenNeedsRefresh = false;

        if(fullRefresh)
            lcd_clear();

        switch(ui)
        {
            case UI_WELCOME:                show_welcome(); break;
            case UI_DASH:                   show_dash(); break;

            /* MAIN MENU */
            case UI_MENU:                   show_menu(); break;

            /* MANUAL / SEMI AUTO MODES */
            case UI_MANUAL:                 show_manual(); break;
            case UI_SEMI_AUTO:              show_semi_auto(); break;

            /* TIMER MODE + EDITORS */
            case UI_TIMER:                  show_timer(); break;
            case UI_TIMER_EDIT_SLOT_ON_H:   show_timer_on_h(); break;
            case UI_TIMER_EDIT_SLOT_ON_M:   show_timer_on_m(); break;
            case UI_TIMER_EDIT_SLOT_OFF_H:  show_timer_off_h(); break;
            case UI_TIMER_EDIT_SLOT_OFF_M:  show_timer_off_m(); break;

            /* AUTO MODE SETTINGS */
            case UI_AUTO_MENU:              show_auto_menu(); break;
            case UI_AUTO_EDIT_GAP:          show_auto_gap(); break;
            case UI_AUTO_EDIT_MAXRUN:       show_auto_maxrun(); break;
            case UI_AUTO_EDIT_RETRY:        show_auto_retry(); break;

            /* COUNTDOWN MODE */
            case UI_COUNTDOWN:              show_countdown(); break;
            case UI_COUNTDOWN_EDIT_MIN:     show_countdown_edit_min(); break;

            /* TWIST MODE */
            case UI_TWIST:                  show_twist(); break;
            case UI_TWIST_EDIT_ON:          show_twist_on_sec(); break;
            case UI_TWIST_EDIT_OFF:         show_twist_off_sec(); break;
            case UI_TWIST_EDIT_ON_H:        show_twist_on_h(); break;
            case UI_TWIST_EDIT_ON_M:        show_twist_on_m(); break;
            case UI_TWIST_EDIT_OFF_H:       show_twist_off_h(); break;
            case UI_TWIST_EDIT_OFF_M:       show_twist_off_m(); break;

            default:
                lcd_line0("Not Implemented");
                lcd_line1(" ");
                break;
        }
    }
}

/***************************************************************
 *  END OF FINAL SCREEN.C
 ***************************************************************/
