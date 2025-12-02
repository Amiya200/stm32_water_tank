/***************************************************************
 *  HELONIX Water Pump Controller
 *  SCREEN.C — UPDATED FINAL VERSION (2025)
 *
 *  PART 1 / 4
 *  - Enums
 *  - UI Globals
 *  - LCD Helpers
 *  - Smooth Cursor Engine
 *  - Welcome Screen
 *  - Dashboard Screen
 *  - Menu Screen (Updated)
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
   UPDATED MENU RENDERER (NO FLICKER)
   ================================================================ */

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

    draw_menu_cursor();  // draw cursor separately (smooth blink)
}

/***************************************************************
 *  SCREEN.C — PART 2 / 4 (UPDATED)
 *  TIMER / AUTO / TWIST / COUNTDOWN UI
 *  APPLY FUNCTIONS
 *  Supports 5 TIMER SLOTS (Option A Layout)
 ***************************************************************/

/* ================================================================
   LCD VALUE FORMATTER
   ================================================================ */
static inline void lcd_val_next(uint16_t v){
    char buf[17];
    snprintf(buf, sizeof(buf), "val:%03u   Next>", v);
    lcd_line1(buf);
}
void Screen_Init(void)
{
    lcd_init();                   // Initialize I2C LCD
    lcd_clear();

    ui = UI_WELCOME;              // Start at welcome screen
    last_ui = UI_MAX_;            // Force full refresh on first update
    screenNeedsRefresh = true;

    cursorVisible = true;
    lastCursorToggle = HAL_GetTick();
    lastLcdUpdateTime = HAL_GetTick();
    refreshInactivityTimer();

    /* Reset all button press trackers */
    for (int i = 0; i < 4; i++)
    {
        sw_press_start[i] = 0;
        sw_long_issued[i] = false;
    }

    /* Default menu state */
    menu_idx = 0;
    menu_view_top = 0;

    /* Timer slot UI state */

    /* Initial welcome screen */
    lcd_put_cur(0, 0);
    lcd_send_string("   HELONIX");

    lcd_put_cur(1, 0);
    lcd_send_string(" IntelligentSys");
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
   TIMER MODE — OPTION A (5 Slots)
   ================================================================ */
/*
   Display Layout:

   PAGE 1:
   >S1 S2
    S3 S4

   PAGE 2:
   >S3 S4
    S5 Back

   PAGE 3:
   >S5
    Back
*/

static int timer_page = 0;  // 0,1,2 depending on scroll

static void show_timer_slot_select(void){
    lcd_clear();

    char l0[17] = {0};
    char l1[17] = {0};

    /*
       Timer Page Layouts
       Page 0: S1 S2 / S3 S4
       Page 1: S3 S4 / S5 Back
       Page 2: S5 / Back
    */

    if (timer_page == 0){
        snprintf(l0, sizeof(l0),
                 "%cS1  S2",
                 (currentSlot == 0 || currentSlot == 1) ? '>' : ' ');

        snprintf(l1, sizeof(l1),
                 "%cS3  S4",
                 (currentSlot == 2 || currentSlot == 3) ? '>' : ' ');
    }
    else if (timer_page == 1){
        snprintf(l0, sizeof(l0),
                 "%cS3  S4",
                 (currentSlot == 2 || currentSlot == 3) ? '>' : ' ');

        snprintf(l1, sizeof(l1),
                 "%cS5  Back",
                 (currentSlot == 4) ? '>' :
                 (currentSlot == 5) ? '>' : ' ');
    }
    else if (timer_page == 2){
        snprintf(l0, sizeof(l0),
                 "%cS5",
                 (currentSlot == 4) ? '>' : ' ');

        snprintf(l1, sizeof(l1),
                 "%cBack",
                 (currentSlot == 5) ? '>' : ' ');
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
   AUTO MODE SETTINGS
   ================================================================ */

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
    lcd_val_next(edit_countdown_min);
}

/* ================================================================
   APPLY FUNCTIONS (MENU-SAFE)
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
/***************************************************************
 *  SCREEN.C — PART 3 / 4 (UPDATED FINAL VERSION)
 *  BUTTON DECODER + MENU LOCK + TIMER SLOT ENGINE (OPTION-A)
 ***************************************************************/

/* ================================================================
   BUTTON DECODER — Short / Long press logic
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
   MENU SELECT — Updated for 5 Timer Slots (Option-A Layout)
   ================================================================ */

static void goto_menu_top(void){
    menu_idx = 0;
    menu_view_top = 0;
}

/*
    TIMER SLOT INDEX MAPPING (Option-A)

    currentSlot = 0 → S1
    currentSlot = 1 → S2
    currentSlot = 2 → S3
    currentSlot = 3 → S4
    currentSlot = 4 → S5
    currentSlot = 5 → BACK
*/

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

    /* MAIN MENU */
    if (ui == UI_MENU){
        switch(menu_idx)
        {
            case 0: // TIMER MODE
                ui = UI_TIMER_SLOT_SELECT;
                timer_page = 0;
                currentSlot = 0;
                break;

            case 1: // AUTO MODE
                ui = UI_AUTO_MENU;
                break;

            case 2: // TWIST MODE
                ui = UI_TWIST;
                break;

            case 3: // SETTINGS (Your future extension)
                ui = UI_DASH;
                break;

            case 4: // BACK
                ui = UI_DASH;
                break;
        }
        screenNeedsRefresh = true;
        return;
    }

    /* =========== TIMER MODE =========== */

    if (ui == UI_TIMER_SLOT_SELECT)
    {
        /* BACK selected? */
        if (currentSlot == 5){
            ui = UI_DASH;
            screenNeedsRefresh = true;
            return;
        }

        /* Load slot into edit buffer */
        edit_timer_on_h  = timerSlots[currentSlot].onHour;
        edit_timer_on_m  = timerSlots[currentSlot].onMinute;
        edit_timer_off_h = timerSlots[currentSlot].offHour;
        edit_timer_off_m = timerSlots[currentSlot].offMinute;

        ui = UI_TIMER_EDIT_SLOT_ON_H;
        screenNeedsRefresh = true;
        return;
    }

    /* TIMER EDIT SEQUENCE */
    if (ui == UI_TIMER_EDIT_SLOT_ON_H){
        ui = UI_TIMER_EDIT_SLOT_ON_M;
        screenNeedsRefresh = true;
        return;
    }
    if (ui == UI_TIMER_EDIT_SLOT_ON_M){
        ui = UI_TIMER_EDIT_SLOT_OFF_H;
        screenNeedsRefresh = true;
        return;
    }
    if (ui == UI_TIMER_EDIT_SLOT_OFF_H){
        ui = UI_TIMER_EDIT_SLOT_OFF_M;
        screenNeedsRefresh = true;
        return;
    }
    if (ui == UI_TIMER_EDIT_SLOT_OFF_M){
        apply_timer_settings();
        ui = UI_TIMER_SLOT_SELECT;
        screenNeedsRefresh = true;
        return;
    }

    /* AUTO MODE SEQUENCE */
    if (ui == UI_AUTO_MENU){
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

    /* TWIST SEQUENCE */
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

    /* COUNTDOWN SETTINGS */
    if (ui == UI_COUNTDOWN_EDIT_MIN){
        apply_countdown_settings();
        ui = UI_COUNTDOWN;
        screenNeedsRefresh = true;
        return;
    }
}

/* ================================================================
   EDIT VALUE ENGINE
   ================================================================ */

void increase_edit_value(void){
    switch(ui)
    {
        case UI_TIMER_EDIT_SLOT_ON_H:  if(edit_timer_on_h < 23) edit_timer_on_h++; break;
        case UI_TIMER_EDIT_SLOT_ON_M:  if(edit_timer_on_m < 59) edit_timer_on_m++; break;
        case UI_TIMER_EDIT_SLOT_OFF_H: if(edit_timer_off_h < 23) edit_timer_off_h++; break;
        case UI_TIMER_EDIT_SLOT_OFF_M: if(edit_timer_off_m < 59) edit_timer_off_m++; break;

        case UI_AUTO_EDIT_GAP:         edit_auto_gap_s++; break;
        case UI_AUTO_EDIT_MAXRUN:      edit_auto_maxrun_min++; break;
        case UI_AUTO_EDIT_RETRY:       edit_auto_retry++; break;

        case UI_TWIST_EDIT_ON:         edit_twist_on_s++; break;
        case UI_TWIST_EDIT_OFF:        edit_twist_off_s++; break;

        case UI_TWIST_EDIT_ON_H:       if(edit_twist_on_hh < 23) edit_twist_on_hh++; break;
        case UI_TWIST_EDIT_ON_M:       if(edit_twist_on_mm < 59) edit_twist_on_mm++; break;
        case UI_TWIST_EDIT_OFF_H:      if(edit_twist_off_hh < 23) edit_twist_off_hh++; break;
        case UI_TWIST_EDIT_OFF_M:      if(edit_twist_off_mm < 59) edit_twist_off_mm++; break;

        case UI_COUNTDOWN_EDIT_MIN:    if(edit_countdown_min < 999) edit_countdown_min++; break;

        default: break;
    }
}

void decrease_edit_value(void){
    switch(ui)
    {
        case UI_TIMER_EDIT_SLOT_ON_H:  if(edit_timer_on_h > 0) edit_timer_on_h--; break;
        case UI_TIMER_EDIT_SLOT_ON_M:  if(edit_timer_on_m > 0) edit_timer_on_m--; break;
        case UI_TIMER_EDIT_SLOT_OFF_H: if(edit_timer_off_h > 0) edit_timer_off_h--; break;
        case UI_TIMER_EDIT_SLOT_OFF_M: if(edit_timer_off_m > 0) edit_timer_off_m--; break;

        case UI_AUTO_EDIT_GAP:         if(edit_auto_gap_s > 0) edit_auto_gap_s--; break;
        case UI_AUTO_EDIT_MAXRUN:      if(edit_auto_maxrun_min > 0) edit_auto_maxrun_min--; break;
        case UI_AUTO_EDIT_RETRY:       if(edit_auto_retry > 0) edit_auto_retry--; break;

        case UI_TWIST_EDIT_ON:         if(edit_twist_on_s > 0) edit_twist_on_s--; break;
        case UI_TWIST_EDIT_OFF:        if(edit_twist_off_s > 0) edit_twist_off_s--; break;

        case UI_TWIST_EDIT_ON_H:       if(edit_twist_on_hh > 0) edit_twist_on_hh--; break;
        case UI_TWIST_EDIT_ON_M:       if(edit_twist_on_mm > 0) edit_twist_on_mm--; break;
        case UI_TWIST_EDIT_OFF_H:      if(edit_twist_off_hh > 0) edit_twist_off_hh--; break;
        case UI_TWIST_EDIT_OFF_M:      if(edit_twist_off_mm > 0) edit_twist_off_mm--; break;

        case UI_COUNTDOWN_EDIT_MIN:    if(edit_countdown_min > 1) edit_countdown_min--; break;

        default: break;
    }
}

/* ================================================================
   MAIN BUTTON HANDLER — MENU LOCK + TIMER SLOT ENGINE
   ================================================================ */

void Screen_HandleSwitches(void)
{
    UiButton b = decode_button_press();
    uint32_t now = HAL_GetTick();

    bool sw3 = Switch_IsPressed(2);
    bool sw4 = Switch_IsPressed(3);

    /* ==========================================
       CONTINUOUS HOLD FOR MENU + EDIT ONLY
       ========================================== */

    if (sw3 && sw_long_issued[2]) {
        if (now - last_repeat_time >= CONTINUOUS_STEP_MS) {
            last_repeat_time = now;

            if (ui == UI_MENU){
                if (menu_idx > 0) menu_idx--;
            }
            else if (ui == UI_TIMER_SLOT_SELECT){
                /* scroll UP among 0..5 */
                if (currentSlot > 0) currentSlot--;
                if (currentSlot < 2) timer_page = 0;
                else if (currentSlot < 5) timer_page = 1;
                else timer_page = 2;
            }
            else {
                increase_edit_value();
            }
            screenNeedsRefresh = true;
        }
    }

    if (sw4 && sw_long_issued[3]) {
        if (now - last_repeat_time >= CONTINUOUS_STEP_MS) {
            last_repeat_time = now;

            if (ui == UI_MENU){
                if (menu_idx < MAIN_MENU_COUNT-1) menu_idx++;
            }
            else if (ui == UI_TIMER_SLOT_SELECT){
                if (currentSlot < 5) currentSlot++;
                if (currentSlot < 2) timer_page = 0;
                else if (currentSlot < 5) timer_page = 1;
                else timer_page = 2;
            }
            else {
                decrease_edit_value();
            }
            screenNeedsRefresh = true;
        }
    }

    /* NO BUTTON EVENT */
    if (b == BTN_NONE)
        return;

    refreshInactivityTimer();

    /* ==========================================
       IF MENU IS OPEN → BLOCK MODE ACTIONS
       Only navigation + edit allowed
       ========================================== */

    bool menu_open =
        (ui == UI_MENU ||
         ui == UI_TIMER_SLOT_SELECT ||
         ui == UI_TIMER_EDIT_SLOT_ON_H ||
         ui == UI_TIMER_EDIT_SLOT_ON_M ||
         ui == UI_TIMER_EDIT_SLOT_OFF_H ||
         ui == UI_TIMER_EDIT_SLOT_OFF_M ||
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
         ui == UI_COUNTDOWN_EDIT_MIN);

    /* ==========================================
       HANDLE MENU BUTTONS (LOCK ACTIVE)
       ========================================== */

    if (menu_open)
    {
        /* -------- EDIT MODE -------- */
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
             ui == UI_COUNTDOWN_EDIT_MIN);

        if (editing){
            switch(b){
                case BTN_UP: increase_edit_value(); break;
                case BTN_UP_LONG: last_repeat_time = now; increase_edit_value(); break;

                case BTN_DOWN: decrease_edit_value(); break;
                case BTN_DOWN_LONG: last_repeat_time = now; decrease_edit_value(); break;

                case BTN_RESET: ui = UI_TIMER_SLOT_SELECT; break;
                case BTN_SELECT: menu_select(); break;
                default: break;
            }
            screenNeedsRefresh = true;
            return;
        }

        /* -------- TIMER SLOT SELECT -------- */
        if (ui == UI_TIMER_SLOT_SELECT){
            switch(b){
                case BTN_UP:
                    if (currentSlot > 0) currentSlot--;
                    if (currentSlot < 2) timer_page = 0;
                    else if (currentSlot < 5) timer_page = 1;
                    else timer_page = 2;
                    break;

                case BTN_DOWN:
                    if (currentSlot < 5) currentSlot++;
                    if (currentSlot < 2) timer_page = 0;
                    else if (currentSlot < 5) timer_page = 1;
                    else timer_page = 2;
                    break;

                case BTN_SELECT:
                    menu_select(); break;

                case BTN_RESET:
                    ui = UI_MENU; break;

                default: break;
            }
            screenNeedsRefresh = true;
            return;
        }

        /* -------- MAIN MENU NAVIGATION -------- */
        if (ui == UI_MENU){
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

        /* For all other menu screens */
        if (b == BTN_SELECT)
            menu_select();
        else if (b == BTN_RESET)
            ui = UI_MENU;

        screenNeedsRefresh = true;
        return;
    }

    /* ============================================================
       BELOW THIS POINT → MENU IS NOT OPEN
       NORMAL MODE BEHAVIOR (Manual / Auto / Timer / Countdown etc.)
       ============================================================ */

    switch(b)
    {
        /* ---------- SW1 (RED) ---------- */
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

        /* ---------- SW2 (YELLOW) ---------- */
        case BTN_SELECT:
            if(countdownActive){
                ModelHandle_StopCountdown();
                ui = UI_DASH;
                screenNeedsRefresh = true;
                return;
            }

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

        /* ---------- SW3 (UP) ---------- */
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

        /* ---------- SW4 (DOWN) ---------- */
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
 *  SCREEN.C — PART 4 / 4
 *  LCD Update Engine + Final UI Dispatcher
 *  (Smooth Cursor, No Flicker, Menu Lock Enabled)
 ***************************************************************/

void Screen_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* ============================================================
       CURSOR BLINK — ONLY FOR MENU MODE
       ============================================================ */
    bool cursorBlinkActive = (ui == UI_MENU);

    if (cursorBlinkActive && (now - lastCursorToggle >= CURSOR_BLINK_MS))
    {
        cursorVisible = !cursorVisible;
        lastCursorToggle = now;
        draw_menu_cursor();      // Draw only the cursor (fast, no flicker)
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
       AUTO-RETURN TO DASH AFTER INACTIVITY (1 minute)
       ============================================================ */
    if (ui != UI_WELCOME &&
        ui != UI_DASH &&
        now - lastUserAction >= AUTO_BACK_MS)
    {
        ui = UI_DASH;
        screenNeedsRefresh = true;
    }

    /* ============================================================
       AUTO DASHBOARD REFRESH (every 1 sec)
       ============================================================ */
    if ((ui == UI_DASH || ui == UI_COUNTDOWN) &&
        now - lastLcdUpdateTime >= 1000)
    {
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }

    /* ============================================================
       REFRESH LCD IF NEEDED OR UI CHANGED
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
            /* ------------------
               WELCOME / DASHBOARD
               ------------------ */
            case UI_WELCOME:
                show_welcome();
                break;

            case UI_DASH:
                show_dash();
                break;

            /* ------------------
               MAIN MENU
               ------------------ */
            case UI_MENU:
                show_menu();
                break;

            /* ------------------
               TIMER MODE (5 Slots)
               ------------------ */
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

            /* ------------------
               AUTO MODE
               ------------------ */
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

            /* ------------------
               COUNTDOWN
               ------------------ */
            case UI_COUNTDOWN:
                show_countdown();
                break;

            case UI_COUNTDOWN_EDIT_MIN:
                show_countdown_edit_min();
                break;

            /* ------------------
               TWIST MODE
               ------------------ */
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

            /* ------------------
               DEFAULT
               ------------------ */
            default:
                lcd_line0("Not Implemented");
                lcd_line1(" ");
                break;
        }

        /* Draw cursor again AFTER rendering if inside menu */
        if (ui == UI_MENU)
            draw_menu_cursor();
    }
}

/***************************************************************
 *  END OF FINAL SCREEN.C
 ***************************************************************/
