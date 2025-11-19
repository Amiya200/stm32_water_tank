/* screen.c
   Redesigned 16x2 cursor-based UI with menu + edit flows for:
   Manual, Semi-Auto, Timer, Search, Countdown, Twist
*/

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

/* ===== UI Buttons ===== */
typedef enum {
    BTN_NONE = 0,
    BTN_RESET,   // SW1 → Manual Toggle (short) / Restart (long)
    BTN_SELECT,  // SW2
    BTN_UP,      // SW3
    BTN_DOWN     // SW4 → Down / Back (long)
} UiButton;

/* ===== Internal UI States ===== */
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

    UI_AUTO_MENU,           // Auto menu root
    UI_AUTO_EDIT_GAP,       // Dry gap time
    UI_AUTO_EDIT_MAXRUN,    // Max run time
    UI_AUTO_EDIT_RETRY,     // Retry count

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

char buf[17];

/* ===== Timing ===== */
static uint32_t lastLcdUpdateTime = 0;
static const uint32_t WELCOME_MS = 3000;
static const uint32_t CURSOR_BLINK_MS = 500;
static const uint32_t AUTO_BACK_MS = 60000;

/* ===== UI state ===== */
static UiState ui = UI_WELCOME;
static UiState last_ui = UI_MAX_;
static bool screenNeedsRefresh = false;
static bool cursorVisible = true;
static uint32_t lastCursorToggle = 0;
static uint32_t lastUserAction = 0;

/* ===== Externals ===== */
extern ADC_Data adcData;

extern TimerSlot timerSlots[5];
static uint8_t currentSlot = 0;
extern TwistSettings  twistSettings;
extern volatile bool  countdownActive;
extern volatile uint32_t countdownDuration;
extern volatile bool  countdownMode;
extern volatile bool  manualActive;
extern volatile bool  semiAutoActive;
extern volatile bool  timerActive;
extern volatile bool  twistActive;
// ---- TWIST extended params ----
static uint8_t edit_twist_on_hh = 6;
static uint8_t edit_twist_on_mm = 0;
static uint8_t edit_twist_off_hh = 18;
static uint8_t edit_twist_off_mm = 0;

static bool semiAutoEnabled = false;

extern bool Motor_GetStatus(void);

/* ===== Menu definitions ===== */
static const char * const main_menu[] = {
    "Timer Mode",
    "Auto Mode",
    "Twist Mode",
	"Settings",
    "Back"
};

#define MAIN_MENU_COUNT (sizeof(main_menu)/sizeof(main_menu[0]))

/* ===== Cursor/menu bookkeeping ===== */
static int menu_idx = 0;
static int menu_view_top = 0;

/* ===== Temp edit variables ===== */
static uint8_t  edit_timer_on_h  = 6,  edit_timer_on_m  = 30;
static uint8_t  edit_timer_off_h = 18, edit_timer_off_m = 30;
static uint16_t edit_search_gap_s = 60, edit_search_dry_s = 10;
static uint16_t edit_twist_on_s = 5, edit_twist_off_s = 5;
static uint16_t edit_countdown_min = 5;
// Long press detect for SELECT (SW2)
static uint32_t selectPressStart = 0;
static bool selectLongPressHandled = false;

#define SELECT_LONG_PRESS_MS 3000

/* ===== LCD Helpers ===== */
static inline void lcd_line(uint8_t row, const char* s) {
    char ln[17];
    snprintf(ln, sizeof(ln), "%-16.16s", s);
    lcd_put_cur(row, 0);
    lcd_send_string(ln);
}
static inline void lcd_line0(const char* s){ lcd_line(0,s); }
static inline void lcd_line1(const char* s){ lcd_line(1,s); }

/* ===== Utilities ===== */
static void refreshInactivityTimer(void){ lastUserAction = HAL_GetTick(); }
static void goto_menu_top(void){ menu_idx = 0; menu_view_top = 0; }

/* ===== Helper: format menu line ===== */
static void format_menu_line(char* buf, size_t bufsize, int idx, bool selected){
    if (idx < (int)MAIN_MENU_COUNT && idx >= 0) {
        char prefix = selected ? '>' : ' ';
        char item[16];
        snprintf(item, sizeof(item), "%-15.15s", main_menu[idx]);
        snprintf(buf, bufsize, "%c%s", prefix, item);
    } else {
        snprintf(buf, bufsize, "                ");
    }
}

/* ===== Render functions ===== */
static void show_welcome(void){
    lcd_clear();
    lcd_line0("  Welcome to ");
    lcd_line1("   HELONIX   ");
}

static void show_dash(void) {
    char line0[17], line1[17];
    const char *motor = Motor_GetStatus() ? "ON " : "OFF";

    const char *mode = "IDLE";
    if (manualActive)        mode = "Manual";
    else if (semiAutoActive) mode = "SemiAuto";
    else if (timerActive)    mode = "Timer";
    else if (countdownActive)mode = "Countdown";
    else if (twistActive)    mode = "Twist";

    snprintf(line0,sizeof(line0),"M:%s %s",motor,mode);

    int submergedCount = 0;
    for (int i=1; i<6; i++) {
        if (adcData.voltages[i] < 0.1f) submergedCount++;
    }

    const char *level;
    switch (submergedCount) {
        case 0:  level = "00%"; break;
        case 1:  level = "20%";   break;
        case 2:  level = "40%";  break;
        case 3:  level = "60%";   break;
        case 4:  level = "80%";   break;
        default: level = "100%";  break;
    }

    snprintf(line1,sizeof(line1),"Water:%-5s",level);

    lcd_line0(line0);
    lcd_line1(line1);
}

static void show_menu(void){
    char line0[17], line1[17];
    if (menu_idx < menu_view_top) menu_view_top = menu_idx;
    else if (menu_idx > menu_view_top+1) menu_view_top = menu_idx-1;
    format_menu_line(line0,sizeof(line0),menu_view_top,menu_idx==menu_view_top && cursorVisible);
    format_menu_line(line1,sizeof(line1),menu_view_top+1,menu_idx==(menu_view_top+1) && cursorVisible);
    lcd_line0(line0);
    lcd_line1(line1);
}

static void show_manual(void){
    char line0[17], line1[17];
    snprintf(line0,sizeof(line0),"Manual Mode");
    if (Motor_GetStatus()) snprintf(line1,sizeof(line1),">Stop     Back");
    else snprintf(line1,sizeof(line1),">Start    Back");
    lcd_line0(line0);
    lcd_line1(line1);
}
static uint16_t edit_auto_gap_s = 10;
static uint16_t edit_auto_maxrun_min = 120;
static uint16_t edit_auto_retry = 0;

static void show_auto_menu(void)
{
    char l0[17], l1[17];
    switch(ui)
    {
        case UI_AUTO_MENU:
            lcd_line0(">Gap / MaxRun");
            lcd_line1(" Retry  Back");
            break;

        case UI_AUTO_EDIT_GAP:
            snprintf(l0,sizeof(l0),"Gap: %us",edit_auto_gap_s);
            lcd_line0(l0);
            lcd_line1("UP/DN  Next");
            break;

        case UI_AUTO_EDIT_MAXRUN:
            snprintf(l0,sizeof(l0),"MaxRun %umin",edit_auto_maxrun_min);
            lcd_line0(l0);
            lcd_line1("UP/DN  Next");
            break;

        case UI_AUTO_EDIT_RETRY:
            snprintf(l0,sizeof(l0),"Retry: %u",edit_auto_retry);
            lcd_line0(l0);
            lcd_line1("UP/DN  Save");
            break;

        default:
            break;
    }
}

static void show_semi_auto(void){
    char line0[17], line1[17];
    snprintf(line0,sizeof(line0),"Semi-Auto Mode");
    if (semiAutoEnabled) snprintf(line1,sizeof(line1),">Disable  Back");
    else snprintf(line1,sizeof(line1),">Enable   Back");
    lcd_line0(line0);
    lcd_line1(line1);
}

static void show_timer(void)
{
    char l0[17], l1[17];
    TimerSlot *ts = &timerSlots[currentSlot];
    snprintf(l0, sizeof(l0), "T%d %02d:%02d-%02d:%02d",
             currentSlot + 1, ts->onHour, ts->onMinute, ts->offHour, ts->offMinute);
    snprintf(l1, sizeof(l1), "EN:%s  Edit>Next",
             ts->enabled ? "Y" : "N");
    lcd_line0(l0);
    lcd_line1(l1);
}


static void apply_timer_slot(uint8_t slot)
{
    if (slot >= 5) return;
    TimerSlot *ts = &timerSlots[slot];
    ts->onHour    = edit_timer_on_h;
    ts->onMinute  = edit_timer_on_m;
    ts->offHour   = edit_timer_off_h;
    ts->offMinute = edit_timer_off_m;
    ts->enabled   = true;
}


static void show_countdown(void){
    char l0[17], l1[17];

    if (countdownActive) {
        uint32_t sec = countdownDuration;
        uint32_t min = sec / 60;
        uint32_t s   = sec % 60;

        snprintf(l0, sizeof(l0), "Time %02d:%02d", (int)min, (int)s);
        snprintf(l1, sizeof(l1), ">Stop     Back");
    }
    else {
        snprintf(l0, sizeof(l0), "Countdown Mode");
        snprintf(l1, sizeof(l1), ">Set      Back");
    }

    lcd_line0(l0);
    lcd_line1(l1);
}




static void show_twist(void){
    char l0[17], l1[17];

    const char* status = twistActive ? "ON " : "OFF";
    snprintf(l0,sizeof(l0),"Tw %s %2ds/%2ds",
             status,
             (int)twistSettings.onDurationSeconds,
             (int)twistSettings.offDurationSeconds);

    snprintf(l1,sizeof(l1),">%s   Edit",
             twistActive ? "Disable" : "Enable");

    lcd_line0(l0);
    lcd_line1(l1);
}


/* ===== Apply functions ===== */


static void apply_twist_settings(void)
{
    twistSettings.onDurationSeconds  = edit_twist_on_s;
    twistSettings.offDurationSeconds = edit_twist_off_s;

    twistSettings.onHour   = edit_twist_on_hh;
    twistSettings.onMinute = edit_twist_on_mm;
    twistSettings.offHour  = edit_twist_off_hh;
    twistSettings.offMinute= edit_twist_off_mm;
}

static void apply_countdown_settings(void){
    // kept for compatibility if other code depends on countdownDuration mirror
    countdownDuration = (uint32_t)edit_countdown_min * 60u;
}

static void enable_semi_auto(void){
    ModelHandle_ClearManualOverride();
    ModelHandle_StartSemiAuto();
    semiAutoEnabled = true;
}
static void disable_semi_auto(void){
    ModelHandle_SetMotor(false);
    semiAutoEnabled = false;
}

static void apply_timer_settings(void)
{
    if (currentSlot >= 5) return;

    timerSlots[currentSlot].enabled = true;
    timerSlots[currentSlot].onHour  = edit_timer_on_h;
    timerSlots[currentSlot].onMinute = edit_timer_on_m;
    timerSlots[currentSlot].offHour  = edit_timer_off_h;
    timerSlots[currentSlot].offMinute = edit_timer_off_m;

    /* 🔥 Immediately re-evaluate timer logic */
    extern void ModelHandle_TimerRecalculateNow(void);
    ModelHandle_TimerRecalculateNow();
}



void Screen_ShowCurrentMode(void)
{
}



/* ===== Initialization / Reset ===== */
void Screen_Init(void)
{
    lcd_init();
    ui = UI_WELCOME;
    last_ui = UI_MAX_;
    screenNeedsRefresh = true;
    lastLcdUpdateTime = HAL_GetTick();
    refreshInactivityTimer();

    /* --- Existing modes --- */
    edit_twist_on_s   = twistSettings.onDurationSeconds;
    edit_twist_off_s  = twistSettings.offDurationSeconds;

    edit_countdown_min = (uint16_t)(countdownDuration / 60u);
    if (edit_countdown_min == 0)
        edit_countdown_min = 5;
}

void Screen_ResetToHome(void){
    ui = UI_WELCOME;
    last_ui = UI_MAX_;
    screenNeedsRefresh = true;
    lastLcdUpdateTime = HAL_GetTick();
    refreshInactivityTimer();
}
/* ============================
   MENU SELECTION LOGIC
   ============================ */
/* ============================================================
   MENU SELECT (SW2 short press)
   ============================================================ */
static void menu_select(void)
{
    refreshInactivityTimer();

    switch (ui)
    {
        /* --------------------------
           WELCOME SCREEN
        --------------------------- */
        case UI_WELCOME:
            ui = UI_DASH;
            break;

        /* --------------------------
           DASHBOARD → MENU
        --------------------------- */
        case UI_DASH:
            ui = UI_MENU;
            goto_menu_top();
            break;

        /* ======================================================
           MAIN MENU ITEMS
           ====================================================== */
        case UI_MENU:
            switch (menu_idx)
            {
                /* ------------------------------------
                   0) TIMER MODE
                ------------------------------------- */
                case 0:
                    ui = UI_TIMER;
                    currentSlot = 0;

                    // Load slot into edit fields
                    edit_timer_on_h  = timerSlots[currentSlot].onHour;
                    edit_timer_on_m  = timerSlots[currentSlot].onMinute;
                    edit_timer_off_h = timerSlots[currentSlot].offHour;
                    edit_timer_off_m = timerSlots[currentSlot].offMinute;

                    break;

                /* ------------------------------------
                   1) AUTO MODE SETTINGS
                   ------------------------------------ */
                case 1:
                    ui = UI_AUTO_MENU;     // NOTE: create UI_AUTO_MENU if you want full editor
                    break;

                /* ------------------------------------
                   2) TWIST MODE
                   ------------------------------------ */
                case 2:
                    ui = UI_TWIST;
                    break;

                /* ------------------------------------
                   3) BACK → DASHBOARD
                   ------------------------------------ */
                case 3:
                    ui = UI_DASH;
                    break;

                default:
                    ui = UI_DASH;
                    break;
            }
            break;

        /* ======================================================
           MANUAL MODE (should not be in menu anymore)
           ====================================================== */
        case UI_MANUAL:
            ModelHandle_ToggleManual();
            screenNeedsRefresh = true;
            break;

        /* ======================================================
           SEMI-AUTO MODE
           ====================================================== */
        case UI_SEMI_AUTO:
            if (!semiAutoEnabled) enable_semi_auto();
            else disable_semi_auto();
            ui = UI_DASH;
            break;

        /* ======================================================
           TIMER MODE SLOT SELECT / EDIT FLOW
           ====================================================== */
        case UI_TIMER:
            // Timer state — short press does nothing
            break;

        case UI_TIMER_SLOT_SELECT:
            ui = UI_TIMER_EDIT_SLOT_ON_H;
            break;

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
            ui = UI_TIMER_EDIT_SLOT_ENABLE;
            break;

        case UI_TIMER_EDIT_SLOT_ENABLE:
            apply_timer_settings();
            ui = UI_TIMER;
            break;

        /* ======================================================
           COUNTDOWN EDIT FLOW
           ====================================================== */
        case UI_COUNTDOWN:
            if (countdownActive)
                ModelHandle_StopCountdown();
            else
                ui = UI_COUNTDOWN_EDIT_MIN;
            break;

        case UI_COUNTDOWN_EDIT_MIN:
            ui = UI_COUNTDOWN_TOGGLE;
            break;

        case UI_COUNTDOWN_TOGGLE:
            apply_countdown_settings();
            ui = UI_COUNTDOWN;
            break;

        /* ======================================================
           TWIST MODE EDIT FLOW
           ====================================================== */
        case UI_TWIST:
            if (twistActive)
                ModelHandle_StopTwist();
            else
                ui = UI_TWIST_EDIT_ON;
            break;

        case UI_TWIST_EDIT_ON:
            ui = UI_TWIST_EDIT_OFF;
            break;

        case UI_TWIST_EDIT_OFF:
            ui = UI_TWIST_EDIT_ON_H;
            break;

        case UI_TWIST_EDIT_ON_H:
            ui = UI_TWIST_EDIT_ON_M;
            break;

        case UI_TWIST_EDIT_ON_M:
            ui = UI_TWIST_EDIT_OFF_H;
            break;

        case UI_TWIST_EDIT_OFF_H:
            ui = UI_TWIST_EDIT_OFF_M;
            break;

        case UI_TWIST_EDIT_OFF_M:
            apply_twist_settings();
            ModelHandle_StartTwist(
                twistSettings.onDurationSeconds,
                twistSettings.offDurationSeconds,
                twistSettings.onHour,
                twistSettings.onMinute,
                twistSettings.offHour,
                twistSettings.offMinute
            );
            ui = UI_TWIST;
            break;

        case UI_AUTO_EDIT_RETRY:
            // Save to model_handle
            extern void ModelHandle_SetAutoSettings(uint16_t gap_s, uint16_t maxrun_min, uint16_t retry);
            ModelHandle_SetAutoSettings(edit_auto_gap_s, edit_auto_maxrun_min, edit_auto_retry);
            ui = UI_AUTO_MENU;
            break;

        /* ------------------------------- */
        default:
            break;
    }

    screenNeedsRefresh = true;
}

/* ============================
   MENU RESET / BACK LOGIC
   ============================ */
static void menu_reset(void){
    refreshInactivityTimer();

    switch (ui) {
        /* Return to MENU from submodes */
        case UI_MANUAL:
        case UI_SEMI_AUTO:
        case UI_TIMER:
        case UI_COUNTDOWN:
        case UI_TWIST:
            ui = UI_MENU;
            break;

        /* From MENU → DASH, from DASH → WELCOME */
        case UI_MENU:
            ui = UI_DASH;
            break;
        case UI_DASH:
            ui = UI_WELCOME;
            break;

        /* Timer editing states → back to timer list */
        case UI_TIMER_SLOT_SELECT:
        case UI_TIMER_EDIT_SLOT_ON_H:
        case UI_TIMER_EDIT_SLOT_ON_M:
        case UI_TIMER_EDIT_SLOT_OFF_H:
        case UI_TIMER_EDIT_SLOT_OFF_M:
        case UI_TIMER_EDIT_SLOT_ENABLE:
            ui = UI_TIMER;
            break;

        case UI_TWIST_EDIT_ON:
        case UI_TWIST_EDIT_OFF:
        case UI_TWIST_EDIT_ON_H:
        case UI_TWIST_EDIT_ON_M:
        case UI_TWIST_EDIT_OFF_H:
        case UI_TWIST_EDIT_OFF_M:
            ui = UI_TWIST;
            break;


        /* Countdown editing → back to countdown */
        case UI_COUNTDOWN_EDIT_MIN:
            ui = UI_COUNTDOWN;
            break;

        case UI_COUNTDOWN_TOGGLE:
            ui = UI_COUNTDOWN;
            break;



        default:
            ui = UI_MENU;
            break;
    }

    screenNeedsRefresh = true;
}

/* ============================
   SCREEN UPDATE (LCD REFRESH)
   ============================ */
void Screen_Update(void){
    uint32_t now = HAL_GetTick();

    bool cursorBlinkActive = false;
    switch (ui) {
        case UI_MENU:
        case UI_TIMER_EDIT_SLOT_ON_H:
        case UI_TIMER_EDIT_SLOT_ON_M:
        case UI_TIMER_EDIT_SLOT_OFF_H:
        case UI_TIMER_EDIT_SLOT_OFF_M:
        case UI_COUNTDOWN_EDIT_MIN:
        case UI_TWIST_EDIT_ON:
        case UI_TWIST_EDIT_OFF:
            cursorBlinkActive = true;
            break;
        default:
            cursorBlinkActive = false;
            cursorVisible = true;
            break;
    }

    if (cursorBlinkActive && (now - lastCursorToggle >= CURSOR_BLINK_MS)) {
        cursorVisible = !cursorVisible;
        lastCursorToggle = now;
        screenNeedsRefresh = true;
    }

    if (ui == UI_WELCOME && now - lastLcdUpdateTime >= WELCOME_MS) {
        ui = UI_DASH;
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }

    if (ui != UI_WELCOME && ui != UI_DASH && (now - lastUserAction >= AUTO_BACK_MS)) {
        ui = UI_DASH;
        screenNeedsRefresh = true;
    }

    if (ui == UI_DASH && (now - lastLcdUpdateTime >= 1000)) {
        screenNeedsRefresh = true;
        lastLcdUpdateTime = now;
    }

    if (screenNeedsRefresh || ui != last_ui) {
        bool fullRedraw = (ui != last_ui);
        last_ui = ui;
        screenNeedsRefresh = false;
        if (fullRedraw) lcd_clear();

        switch (ui) {
            case UI_WELCOME:     show_welcome(); break;
            case UI_DASH:        show_dash(); break;
            case UI_MENU:        show_menu(); break;
            case UI_MANUAL:      show_manual(); break;
            case UI_SEMI_AUTO:   show_semi_auto(); break;
            case UI_TIMER:       show_timer(); break;
            case UI_COUNTDOWN:   show_countdown(); break;
            case UI_TWIST:       show_twist(); break;

            /* ==== TIMER EDITING ==== */
            case UI_TIMER_SLOT_SELECT: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Select Slot:%d", currentSlot + 1);
                snprintf(l1,sizeof(l1),">Edit   Back");
                lcd_line0(l0); lcd_line1(l1);
                break;
            }

            case UI_TIMER_EDIT_SLOT_ON_H: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"T%d ON Hour:%02d", currentSlot + 1, edit_timer_on_h);
                snprintf(l1,sizeof(l1),">UpDn   Next");
                lcd_line0(l0); lcd_line1(l1);
                break;
            }

            case UI_TIMER_EDIT_SLOT_ON_M: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"T%d ON Min:%02d", currentSlot + 1, edit_timer_on_m);
                snprintf(l1,sizeof(l1),">UpDn   Next");
                lcd_line0(l0); lcd_line1(l1);
                break;
            }

            case UI_TIMER_EDIT_SLOT_OFF_H: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"T%d OFF Hr:%02d", currentSlot + 1, edit_timer_off_h);
                snprintf(l1,sizeof(l1),">UpDn   Next");
                lcd_line0(l0); lcd_line1(l1);
                break;
            }

            case UI_TIMER_EDIT_SLOT_OFF_M: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"T%d OFF Mn:%02d", currentSlot + 1, edit_timer_off_m);
                snprintf(l1,sizeof(l1),">UpDn   Next");
                lcd_line0(l0); lcd_line1(l1);
                break;
            }

            case UI_TIMER_EDIT_SLOT_ENABLE: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Enable Slot %d?", currentSlot + 1);
                snprintf(l1,sizeof(l1),">YES=Sel Back");
                lcd_line0(l0); lcd_line1(l1);
                break;
            }

            /* ==== SEARCH EDIT SCREENS (FIXED & PROPER) ==== */


            case UI_TWIST_EDIT_ON_H:
                lcd_line0("Twist ON Hour");
                snprintf(buf,16,">%02d", edit_twist_on_hh);
                lcd_line1(buf);
                break;

            case UI_TWIST_EDIT_ON_M:
                lcd_line0("Twist ON Min");
                snprintf(buf,16,">%02d", edit_twist_on_mm);
                lcd_line1(buf);
                break;

            case UI_TWIST_EDIT_OFF_H:
                lcd_line0("Twist OFF Hour");
                snprintf(buf,16,">%02d", edit_twist_off_hh);
                lcd_line1(buf);
                break;

            case UI_TWIST_EDIT_OFF_M:
                lcd_line0("Twist OFF Min");
                snprintf(buf,16,">%02d", edit_twist_off_mm);
                lcd_line1(buf);
                break;


            /* ==== COUNTDOWN ==== */
            case UI_COUNTDOWN_EDIT_MIN: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Set Min: %3u", edit_countdown_min);
                snprintf(l1,sizeof(l1),">UpDn   Next");
                lcd_line0(l0); lcd_line1(l1);
                break;
            }

            case UI_COUNTDOWN_TOGGLE: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Start Countdown?");
                snprintf(l1,sizeof(l1),">Yes     Back");
                lcd_line0(l0);
                lcd_line1(l1);
                break;
            }



            /* ==== TWIST ==== */
            case UI_TWIST_EDIT_ON: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Edit T ON:%3ds", edit_twist_on_s);
                snprintf(l1,sizeof(l1),">UpDn   Next");
                lcd_line0(l0); lcd_line1(l1);
                break;
            }

            case UI_TWIST_EDIT_OFF: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Edit T OFF:%3ds", edit_twist_off_s);
                snprintf(l1,sizeof(l1),">UpDn   Save");
                lcd_line0(l0); lcd_line1(l1);
                break;
            }

            default:
                lcd_line0("Not Implemented");
                lcd_line1("                ");
                break;
        }
    }
}

/* ============================
   BUTTON HANDLER
   ============================ */
static void increase_edit_value(void)
{
    switch (ui)
    {
        case UI_TIMER_EDIT_SLOT_ON_H:  if (++edit_timer_on_h  > 23) edit_timer_on_h = 0;  break;
        case UI_TIMER_EDIT_SLOT_ON_M:  if (++edit_timer_on_m  > 59) edit_timer_on_m = 0;  break;
        case UI_TIMER_EDIT_SLOT_OFF_H: if (++edit_timer_off_h > 23) edit_timer_off_h = 0; break;
        case UI_TIMER_EDIT_SLOT_OFF_M: if (++edit_timer_off_m > 59) edit_timer_off_m = 0; break;

        case UI_TWIST_EDIT_ON:    if (++edit_twist_on_s  > 999) edit_twist_on_s = 0;  break;
        case UI_TWIST_EDIT_OFF:   if (++edit_twist_off_s > 999) edit_twist_off_s = 0; break;

        case UI_COUNTDOWN_EDIT_MIN: if (++edit_countdown_min > 999) edit_countdown_min = 1; break;

        default: break;
    }
}

static void decrease_edit_value(void)
{
    switch (ui)
    {
        case UI_TIMER_EDIT_SLOT_ON_H:  if (edit_timer_on_h--  == 0) edit_timer_on_h  = 23; break;
        case UI_TIMER_EDIT_SLOT_ON_M:  if (edit_timer_on_m--  == 0) edit_timer_on_m  = 59; break;
        case UI_TIMER_EDIT_SLOT_OFF_H: if (edit_timer_off_h-- == 0) edit_timer_off_h = 23; break;
        case UI_TIMER_EDIT_SLOT_OFF_M: if (edit_timer_off_m-- == 0) edit_timer_off_m = 59; break;

        case UI_TWIST_EDIT_ON:    if (edit_twist_on_s--  == 0) edit_twist_on_s = 999;  break;
        case UI_TWIST_EDIT_OFF:   if (edit_twist_off_s-- == 0) edit_twist_off_s = 999; break;

        case UI_COUNTDOWN_EDIT_MIN: if (edit_countdown_min-- == 1) edit_countdown_min = 999; break;

        default: break;
    }
}

void Screen_HandleSwitches(void)
{
    /* Map raw switch events */
    SwitchEvent ev_sw1 = Switch_GetEvent(0);   // RED
    SwitchEvent ev_sw2 = Switch_GetEvent(1);   // Yellow P
    SwitchEvent ev_sw3 = Switch_GetEvent(2);   // UP
    SwitchEvent ev_sw4 = Switch_GetEvent(3);   // DOWN

    bool inMenu =
        (ui == UI_MENU) ||
        (ui == UI_TIMER) ||
        (ui == UI_TIMER_EDIT_SLOT_ON_H) ||
        (ui == UI_TIMER_EDIT_SLOT_ON_M) ||
        (ui == UI_TIMER_EDIT_SLOT_OFF_H) ||
        (ui == UI_TIMER_EDIT_SLOT_OFF_M) ||
        (ui == UI_AUTO_MENU) ||
        (ui == UI_AUTO_EDIT_GAP) ||
        (ui == UI_AUTO_EDIT_MAXRUN) ||
        (ui == UI_AUTO_EDIT_RETRY) ||
        (ui == UI_COUNTDOWN) ||
        (ui == UI_COUNTDOWN_EDIT_MIN) ||
        (ui == UI_TWIST) ||
        (ui == UI_TWIST_EDIT_ON) ||
        (ui == UI_TWIST_EDIT_OFF);

    /* ===========================================================
       NORMAL MODE (DASHBOARD)
       =========================================================== */
    if (!inMenu)
    {
        /* ------------------------
           SW1 (RED)
           SHORT = Restart Pump
           LONG  = Toggle Manual
           ------------------------ */
        if (ev_sw1 == SWITCH_EVT_SHORT)
        {
            ModelHandle_SetMotor(false);
            ModelHandle_SetMotor(true);
        }
        else if (ev_sw1 == SWITCH_EVT_LONG)
        {
            ModelHandle_ToggleManual();
        }

        /* ------------------------
           SW2 (YELLOW P)
           SHORT = Toggle TIMER (AUTO equivalent)
           LONG  = Open menu
           ------------------------ */
        if (ev_sw2 == SWITCH_EVT_SHORT)
        {
            if (timerActive)
                ModelHandle_StopAllModesAndMotor();
            else
                ModelHandle_StartTimer();
        }
        else if (ev_sw2 == SWITCH_EVT_LONG)
        {
            ui = UI_MENU;
            goto_menu_top();
            screenNeedsRefresh = true;
            return;
        }

        /* ------------------------
           SW3 (UP)
           SHORT = Timer slot
           LONG  = Semi-auto toggle
           ------------------------ */
        if (ev_sw3 == SWITCH_EVT_SHORT)
        {
            ModelHandle_StartTimer();
        }
        else if (ev_sw3 == SWITCH_EVT_LONG)
        {
            if (semiAutoActive)
                ModelHandle_StopAllModesAndMotor();
            else
                ModelHandle_StartSemiAuto();
        }

        /* ------------------------
           SW4 (DOWN)
           SHORT = Start countdown
           LONG  = Increase countdown
           ------------------------ */
        if (ev_sw4 == SWITCH_EVT_SHORT)
        {
            ModelHandle_StartCountdown(countdownDuration, 1);
        }
        else if (ev_sw4 == SWITCH_EVT_LONG)
        {
            edit_countdown_min++;
            countdownDuration = edit_countdown_min * 60;
            screenNeedsRefresh = true;
        }

        return;
    }

    /* ===========================================================
       MENU MODE
       =========================================================== */

    /* SW1 BACK */
    if (ev_sw1 == SWITCH_EVT_SHORT)
    {
        ui = UI_DASH;
        screenNeedsRefresh = true;
        return;
    }

    /* SW2 SELECT */
    if (ev_sw2 == SWITCH_EVT_SHORT)
    {
        menu_select();
        screenNeedsRefresh = true;
        return;
    }

    /* ------------------------
       SW3 = UP (and increment)
       ------------------------ */
    if (ev_sw3 == SWITCH_EVT_SHORT)
    {
        menu_idx--;
        if (menu_idx < 0) menu_idx = MAIN_MENU_COUNT - 1;
        screenNeedsRefresh = true;
    }
    else if (ev_sw3 == SWITCH_EVT_LONG)
    {
        switch (ui)
        {
            case UI_TIMER_EDIT_SLOT_ON_H:  if (edit_timer_on_h < 23) edit_timer_on_h++; break;
            case UI_TIMER_EDIT_SLOT_ON_M:  if (edit_timer_on_m < 59) edit_timer_on_m++; break;
            case UI_TIMER_EDIT_SLOT_OFF_H: if (edit_timer_off_h < 23) edit_timer_off_h++; break;
            case UI_TIMER_EDIT_SLOT_OFF_M: if (edit_timer_off_m < 59) edit_timer_off_m++; break;

            case UI_AUTO_EDIT_GAP:         edit_auto_gap_s++; break;
            case UI_AUTO_EDIT_MAXRUN:      edit_auto_maxrun_min++; break;
            case UI_AUTO_EDIT_RETRY:       edit_auto_retry++; break;

            case UI_COUNTDOWN_EDIT_MIN:    edit_countdown_min++; break;

            case UI_TWIST_EDIT_ON:         edit_twist_on_s++; break;
            case UI_TWIST_EDIT_OFF:        edit_twist_off_s++; break;
        }
        screenNeedsRefresh = true;
    }

    /* ------------------------
       SW4 = DOWN (and decrement)
       ------------------------ */
    if (ev_sw4 == SWITCH_EVT_SHORT)
    {
        menu_idx++;
        if (menu_idx >= MAIN_MENU_COUNT) menu_idx = 0;
        screenNeedsRefresh = true;
    }
    else if (ev_sw4 == SWITCH_EVT_LONG)
    {
        switch (ui)
        {
            case UI_TIMER_EDIT_SLOT_ON_H:  if (edit_timer_on_h > 0) edit_timer_on_h--; break;
            case UI_TIMER_EDIT_SLOT_ON_M:  if (edit_timer_on_m > 0) edit_timer_on_m--; break;
            case UI_TIMER_EDIT_SLOT_OFF_H: if (edit_timer_off_h > 0) edit_timer_off_h--; break;
            case UI_TIMER_EDIT_SLOT_OFF_M: if (edit_timer_off_m > 0) edit_timer_off_m--; break;

            case UI_AUTO_EDIT_GAP:         if (edit_auto_gap_s > 0) edit_auto_gap_s--; break;
            case UI_AUTO_EDIT_MAXRUN:      if (edit_auto_maxrun_min > 1) edit_auto_maxrun_min--; break;
            case UI_AUTO_EDIT_RETRY:       if (edit_auto_retry > 0) edit_auto_retry--; break;

            case UI_COUNTDOWN_EDIT_MIN:    if (edit_countdown_min > 1) edit_countdown_min--; break;

            case UI_TWIST_EDIT_ON:         if (edit_twist_on_s > 1) edit_twist_on_s--; break;
            case UI_TWIST_EDIT_OFF:        if (edit_twist_off_s > 1) edit_twist_off_s--; break;
        }
        screenNeedsRefresh = true;
    }
}

/* ============================
   BUTTON HANDLER  (FIXED)
   ============================ */

void Screen_HandleButton(UiButton b)
   {
       if (b == BTN_NONE) return;
       refreshInactivityTimer();

       /* ===========================
          SW1 SHORT → MANUAL TOGGLE
          =========================== */
       if (b == BTN_RESET) {
           ModelHandle_ToggleManual();
           screenNeedsRefresh = true;
           return;
       }


       /* ======================================================
          ======================= UP KEY ========================
          ====================================================== */
       if (b == BTN_UP)
       {
           switch (ui)
           {
               /* -------- MENU -------- */
               case UI_MENU:
                   if (menu_idx > 0) menu_idx--;
                   break;

               /* -------- TIMER MODE -------- */
               case UI_TIMER:
                   currentSlot = (currentSlot < 4) ? currentSlot + 1 : 0;
                   break;

               case UI_TIMER_EDIT_SLOT_ON_H:  if (edit_timer_on_h  < 23) edit_timer_on_h++;  break;
               case UI_TIMER_EDIT_SLOT_ON_M:  if (edit_timer_on_m  < 59) edit_timer_on_m++;  break;
               case UI_TIMER_EDIT_SLOT_OFF_H: if (edit_timer_off_h < 23) edit_timer_off_h++; break;
               case UI_TIMER_EDIT_SLOT_OFF_M: if (edit_timer_off_m < 59) edit_timer_off_m++; break;

               /* -------- COUNTDOWN EDIT -------- */
               case UI_COUNTDOWN_EDIT_MIN:  edit_countdown_min++; break;

               /* -------- TWIST EDIT -------- */
               case UI_TWIST_EDIT_ON:       if (edit_twist_on_s  < 999) edit_twist_on_s++;  break;
               case UI_TWIST_EDIT_OFF:      if (edit_twist_off_s < 999) edit_twist_off_s++; break;

               case UI_TWIST_EDIT_ON_H:     if (edit_twist_on_hh < 23) edit_twist_on_hh++;  break;
               case UI_TWIST_EDIT_ON_M:     if (edit_twist_on_mm < 59) edit_twist_on_mm++;  break;
               case UI_TWIST_EDIT_OFF_H:    if (edit_twist_off_hh < 23) edit_twist_off_hh++; break;
               case UI_TWIST_EDIT_OFF_M:    if (edit_twist_off_mm < 59) edit_twist_off_mm++; break;

               default: break;
           }

           screenNeedsRefresh = true;
           return;
       }


       /* ======================================================
          ===================== DOWN KEY ========================
          ====================================================== */
       if (b == BTN_DOWN)
       {
           switch (ui)
           {
               case UI_MENU:
                   if (menu_idx < (MAIN_MENU_COUNT - 1)) menu_idx++;
                   break;

               case UI_TIMER:
                   currentSlot = (currentSlot > 0) ? currentSlot - 1 : 4;
                   break;

               case UI_TIMER_EDIT_SLOT_ON_H:  if (edit_timer_on_h  > 0) edit_timer_on_h--;  break;
               case UI_TIMER_EDIT_SLOT_ON_M:  if (edit_timer_on_m  > 0) edit_timer_on_m--;  break;
               case UI_TIMER_EDIT_SLOT_OFF_H: if (edit_timer_off_h > 0) edit_timer_off_h--; break;
               case UI_TIMER_EDIT_SLOT_OFF_M: if (edit_timer_off_m > 0) edit_timer_off_m--; break;

               case UI_COUNTDOWN_EDIT_MIN:  if (edit_countdown_min > 1) edit_countdown_min--; break;

               case UI_TWIST_EDIT_ON:       if (edit_twist_on_s  > 1) edit_twist_on_s--;  break;
               case UI_TWIST_EDIT_OFF:      if (edit_twist_off_s > 1) edit_twist_off_s--; break;

               case UI_TWIST_EDIT_ON_H:     if (edit_twist_on_hh > 0) edit_twist_on_hh--;    break;
               case UI_TWIST_EDIT_ON_M:     if (edit_twist_on_mm > 0) edit_twist_on_mm--;    break;
               case UI_TWIST_EDIT_OFF_H:    if (edit_twist_off_hh > 0) edit_twist_off_hh--;  break;
               case UI_TWIST_EDIT_OFF_M:    if (edit_twist_off_mm > 0) edit_twist_off_mm--;  break;

               default: break;
           }

           screenNeedsRefresh = true;
           return;
       }


       /* ======================================================
          ===================== SELECT KEY ======================
          ====================================================== */
       if (b == BTN_SELECT)
       {
           switch (ui)
           {
               /* ----- TIMER FLOW ------ */
               case UI_TIMER:                 ui = UI_TIMER_SLOT_SELECT; break;
               case UI_TIMER_SLOT_SELECT:     ui = UI_TIMER_EDIT_SLOT_ON_H; break;
               case UI_TIMER_EDIT_SLOT_ON_H:  ui = UI_TIMER_EDIT_SLOT_ON_M; break;
               case UI_TIMER_EDIT_SLOT_ON_M:  ui = UI_TIMER_EDIT_SLOT_OFF_H; break;
               case UI_TIMER_EDIT_SLOT_OFF_H: ui = UI_TIMER_EDIT_SLOT_OFF_M; break;
               case UI_TIMER_EDIT_SLOT_OFF_M: ui = UI_TIMER_EDIT_SLOT_ENABLE; break;

               case UI_TIMER_EDIT_SLOT_ENABLE:
                   apply_timer_slot(currentSlot);
                   ui = UI_TIMER;
                   break;


               /* ----- TWIST FLOW ------ */
               case UI_TWIST: ui = UI_TWIST_EDIT_ON; break;
               case UI_TWIST_EDIT_ON:        ui = UI_TWIST_EDIT_OFF; break;
               case UI_TWIST_EDIT_OFF:       ui = UI_TWIST_EDIT_ON_H; break;
               case UI_TWIST_EDIT_ON_H:      ui = UI_TWIST_EDIT_ON_M; break;
               case UI_TWIST_EDIT_ON_M:      ui = UI_TWIST_EDIT_OFF_H; break;
               case UI_TWIST_EDIT_OFF_H:     ui = UI_TWIST_EDIT_OFF_M; break;

               case UI_TWIST_EDIT_OFF_M:
                   apply_twist_settings();
                   ModelHandle_StartTwist(
                       twistSettings.onDurationSeconds,
                       twistSettings.offDurationSeconds,
                       twistSettings.onHour,
                       twistSettings.onMinute,
                       twistSettings.offHour,
                       twistSettings.offMinute
                   );
                   ui = UI_TWIST;
                   break;

               /* ----- COUNTDOWN FLOW ------ */
               case UI_COUNTDOWN:
                   if (countdownActive) ModelHandle_StopCountdown();
                   else ui = UI_COUNTDOWN_EDIT_MIN;
                   break;

               case UI_COUNTDOWN_EDIT_MIN: ui = UI_COUNTDOWN_TOGGLE; break;

               case UI_COUNTDOWN_TOGGLE:
                   apply_countdown_settings();
                   ModelHandle_StartCountdown(edit_countdown_min * 60, 1);
                   ui = UI_COUNTDOWN;
                   break;

               /* ----- DEFAULT MENU ------ */
               default:
                   menu_select();
                   break;
           }

           screenNeedsRefresh = true;
           return;
       }
   }
