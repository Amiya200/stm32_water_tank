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

char buf[17];

static bool restartingMotor = false;
static uint32_t restartDeadline = 0;
/* ===== Timing ===== */
static uint32_t lastLcdUpdateTime = 0;
static const uint32_t WELCOME_MS = 3000;
static const uint32_t CURSOR_BLINK_MS = 500;
static const uint32_t AUTO_BACK_MS = 60000;
/* ==== Long-Press Timing ==== */
#define LONG_PRESS_MS        3000
#define CONTINUOUS_STEP_MS    300     // for auto increment in edit fields
#define COUNTDOWN_INC_MS     3000     // SW4 long press increments every 3 sec

static uint32_t sw_press_start[4] = {0,0,0,0};
static bool     sw_long_issued[4] = {false,false,false,false};
static uint32_t last_repeat_time = 0;

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
static void menu_navigate_up(void)
{
    if (ui == UI_MENU)
    {
        if (menu_idx > 0) menu_idx--;
    }
    else
    {
        // edit modes use increment
        menu_increment(+1);
    }
}

static void menu_navigate_down(void)
{
    if (ui == UI_MENU)
    {
        if (menu_idx < MAIN_MENU_COUNT-1) menu_idx++;
    }
    else
    {
        menu_increment(-1);
    }
}

static void show_dash(void) {
    char line0[17], line1[17];
    const char *motor = Motor_GetStatus() ? "ON " : "OFF";

    const char *mode = "IDLE";
    if (manualActive)        mode = "Manual";
    else if (semiAutoActive) mode = "SemiAuto";
    else if (autoActive) mode = "Auto";
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
        /* WELCOME → DASH */
        case UI_WELCOME:
            ui = UI_DASH;
            screenNeedsRefresh = true;
            return;

        /* DASH → MENU */
        case UI_DASH:
            ui = UI_MENU;
            goto_menu_top();
            screenNeedsRefresh = true;
            return;

        /* =============================
           MAIN MENU SELECT
           ============================= */
        case UI_MENU:
            switch (menu_idx)
            {
                case 0: // TIMER MODE
                    ui = UI_TIMER;
                    currentSlot = 0;
                    screenNeedsRefresh = true;
                    return;

                case 1: // AUTO MODE SETTINGS
                    ui = UI_AUTO_MENU;
                    screenNeedsRefresh = true;
                    return;

                case 2: // TWIST
                    ui = UI_TWIST;
                    screenNeedsRefresh = true;
                    return;

                case 3: // BACK
                    ui = UI_DASH;
                    screenNeedsRefresh = true;
                    return;
            }
            return;

        /* =============================
           TIMER MODE SELECT
           ============================= */
        case UI_TIMER:
            // enter ON HOUR edit
            ui = UI_TIMER_EDIT_SLOT_ON_H;
            screenNeedsRefresh = true;
            return;

        case UI_TIMER_EDIT_SLOT_ON_H:
            ui = UI_TIMER_EDIT_SLOT_ON_M;
            screenNeedsRefresh = true;
            return;

        case UI_TIMER_EDIT_SLOT_ON_M:
            ui = UI_TIMER_EDIT_SLOT_OFF_H;
            screenNeedsRefresh = true;
            return;

        case UI_TIMER_EDIT_SLOT_OFF_H:
            ui = UI_TIMER_EDIT_SLOT_OFF_M;
            screenNeedsRefresh = true;
            return;

        case UI_TIMER_EDIT_SLOT_OFF_M:
            apply_timer_settings();
            ui = UI_TIMER;
            screenNeedsRefresh = true;
            return;

        /* =============================
           COUNTDOWN EDIT SELECT
           ============================= */
        case UI_COUNTDOWN:
            if (countdownActive)
            {
                ModelHandle_StopCountdown();
                screenNeedsRefresh = true;
            }
            else
            {
                ui = UI_COUNTDOWN_EDIT_MIN;
                screenNeedsRefresh = true;
            }
            return;

        case UI_COUNTDOWN_EDIT_MIN:
            apply_countdown_settings();
            ui = UI_COUNTDOWN;
            screenNeedsRefresh = true;
            return;

        /* =============================
           TWIST MODE SELECT
           ============================= */
        case UI_TWIST:
            ui = UI_TWIST_EDIT_ON;
            screenNeedsRefresh = true;
            return;

        case UI_TWIST_EDIT_ON:
            ui = UI_TWIST_EDIT_OFF;
            screenNeedsRefresh = true;
            return;

        case UI_TWIST_EDIT_OFF:
            ui = UI_TWIST_EDIT_ON_H;
            screenNeedsRefresh = true;
            return;

        case UI_TWIST_EDIT_ON_H:
            ui = UI_TWIST_EDIT_ON_M;
            screenNeedsRefresh = true;
            return;

        case UI_TWIST_EDIT_ON_M:
            ui = UI_TWIST_EDIT_OFF_H;
            screenNeedsRefresh = true;
            return;

        case UI_TWIST_EDIT_OFF_H:
            ui = UI_TWIST_EDIT_OFF_M;
            screenNeedsRefresh = true;
            return;

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
            screenNeedsRefresh = true;
            return;

        default:
            return;
    }
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

    // ---- Motor Restart State Machine ----
    if (restartingMotor)
    {
        uint32_t now = HAL_GetTick();
        if ((int32_t)(restartDeadline - now) <= 0)
        {
            ModelHandle_SetMotor(true);      // turn ON after 3 sec
            restartingMotor = false;
            screenNeedsRefresh = true;
        }
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
        /* ----- TIMER EDIT ----- */
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

        /* ----- TWIST EDIT ----- */
        case UI_TWIST_EDIT_ON:
            if (++edit_twist_on_s > 999) edit_twist_on_s = 0;
            break;

        case UI_TWIST_EDIT_OFF:
            if (++edit_twist_off_s > 999) edit_twist_off_s = 0;
            break;

        /* ----- COUNTDOWN EDIT ----- */
        case UI_COUNTDOWN_EDIT_MIN:
            if (++edit_countdown_min > 999) edit_countdown_min = 1;
            break;

        default:
            break;
    }
}

static void decrease_edit_value(void)
{
    switch (ui)
    {
        /* ----- TIMER EDIT ----- */
        case UI_TIMER_EDIT_SLOT_ON_H:
            edit_timer_on_h = (edit_timer_on_h == 0) ? 23 : edit_timer_on_h - 1;
            break;

        case UI_TIMER_EDIT_SLOT_ON_M:
            edit_timer_on_m = (edit_timer_on_m == 0) ? 59 : edit_timer_on_m - 1;
            break;

        case UI_TIMER_EDIT_SLOT_OFF_H:
            edit_timer_off_h = (edit_timer_off_h == 0) ? 23 : edit_timer_off_h - 1;
            break;

        case UI_TIMER_EDIT_SLOT_OFF_M:
            edit_timer_off_m = (edit_timer_off_m == 0) ? 59 : edit_timer_off_m - 1;
            break;

        /* ----- TWIST EDIT ----- */
        case UI_TWIST_EDIT_ON:
            edit_twist_on_s = (edit_twist_on_s == 0) ? 999 : edit_twist_on_s - 1;
            break;

        case UI_TWIST_EDIT_OFF:
            edit_twist_off_s = (edit_twist_off_s == 0) ? 999 : edit_twist_off_s - 1;
            break;

        /* ----- COUNTDOWN EDIT ----- */
        case UI_COUNTDOWN_EDIT_MIN:
            edit_countdown_min = (edit_countdown_min == 1) ? 999 : edit_countdown_min - 1;
            break;

        default:
            break;
    }
}

static UiButton decode_button_press(void)
{
    // Detect raw press state
    bool sw1 = Switch_IsPressed(0);
    bool sw2 = Switch_IsPressed(1);
    bool sw3 = Switch_IsPressed(2);
    bool sw4 = Switch_IsPressed(3);

    uint32_t now = HAL_GetTick();

    UiButton output = BTN_NONE;

    bool sw[4] = {sw1, sw2, sw3, sw4};

    for (int i=0;i<4;i++)
    {
        if (sw[i] && sw_press_start[i] == 0)
        {
            // New press
            sw_press_start[i] = now;
            sw_long_issued[i] = false;
        }
        else if (!sw[i] && sw_press_start[i] != 0)
        {
            // Released → if long press not issued → short
            if (!sw_long_issued[i])
            {
                switch (i)
                {
                    case 0: output = BTN_RESET;  break;
                    case 1: output = BTN_SELECT; break;
                    case 2: output = BTN_UP;     break;
                    case 3: output = BTN_DOWN;   break;
                }
            }
            sw_press_start[i] = 0;
            sw_long_issued[i] = false;
        }
        else if (sw[i] && !sw_long_issued[i])
        {
            // Check long press
            if (now - sw_press_start[i] >= LONG_PRESS_MS)
            {
                sw_long_issued[i] = true;

                switch (i)
                {
                    case 0: output = BTN_RESET_LONG; break;
                    case 1: output = BTN_SELECT_LONG; break;
                    case 2: output = BTN_UP_LONG; break;
                    case 3: output = BTN_DOWN_LONG; break;
                }
            }
        }
    }

    return output;
}

/* ========= NON-BLOCKING MOTOR RESTART VARIABLES ========= */

/* ========================================================
   SCREEN HANDLE SWITCHES  (FINAL VERSION)
   ======================================================== */
void Screen_HandleSwitches(void)
{
    UiButton b = decode_button_press();
    uint32_t now = HAL_GetTick();

    if (b == BTN_NONE)
        return;

    refreshInactivityTimer();

    /* ---------- Restart motor (SW1 short) --- */
    if (restartingMotor)
    {
        if ((int32_t)(restartDeadline - now) <= 0)
        {
            ModelHandle_SetMotor(true);
            restartingMotor = false;
            screenNeedsRefresh = true;
        }
    }

    /* ============================================================
       MENU DETECTION
       ============================================================ */
    bool inMenu =
        (ui == UI_MENU) ||
        (ui >= UI_TIMER && ui <= UI_TWIST_EDIT_OFF_M) ||
        (ui == UI_AUTO_MENU) ||
        (ui == UI_AUTO_EDIT_GAP) ||
        (ui == UI_AUTO_EDIT_MAXRUN) ||
        (ui == UI_AUTO_EDIT_RETRY) ||
        (ui == UI_COUNTDOWN_EDIT_MIN) ||
        (ui == UI_COUNTDOWN_TOGGLE);

    /* ============================================================
       ===================== NORMAL MODE ==========================
       ============================================================ */
    if (!inMenu)
    {
        switch (b)
        {
            /* ====================================================
               SW1 – RED BUTTON
               SHORT → Restart Pump
               LONG → Toggle MANUAL MODE
               ==================================================== */
            case BTN_RESET:
            {
                ModelHandle_SetMotor(false);          // OFF immediately
                restartingMotor = true;
                restartDeadline = now + 3000;         // restart in 3 sec
                return;
            }

            case BTN_RESET_LONG:
            {
                if (manualActive)
                {
                    // manual is ON → turn it OFF
                    manualActive = false;
                    ModelHandle_SetMotor(false);
//                    ModelHandle_ClearManualOverride();
                }
                else
                {
                    // manual is OFF → turn it ON
                    ModelHandle_StopAllModesAndMotor();
                    ModelHandle_ToggleManual();
                }

                ui = UI_DASH;
                screenNeedsRefresh = true;
                return;
            }


            /* ====================================================
               SW2 – YELLOW "P"
               SHORT → Toggle AUTO MODE ON/OFF
               LONG → Open MAIN MENU
               ==================================================== */
            case BTN_SELECT:
            {
//                if (autoActive)
//                    ModelHandle_StopAllModesAndMotor();
//                else
//                    ModelHandle_StartAutoMode();  // your auto handler

                ui = UI_DASH;
                screenNeedsRefresh = true;
                return;
            }

            case BTN_SELECT_LONG:
            {
                ui = UI_MENU;
                goto_menu_top();
                screenNeedsRefresh = true;
                return;
            }

            /* ====================================================
               SW3 – UP BUTTON
               SHORT → Activate nearest Timer Slot
               LONG → Toggle SEMI-AUTO
               ==================================================== */
            case BTN_UP:
            {
                ModelHandle_StartTimer();
                return;
            }

            case BTN_UP_LONG:
            {
                if (semiAutoActive)
                    ModelHandle_StopAllModesAndMotor();
                else {
                    ModelHandle_StopAllModesAndMotor();
                    ModelHandle_StartSemiAuto();
                }
                ui = UI_DASH;
                screenNeedsRefresh = true;
                return;
            }

            /* ====================================================
               SW4 – DOWN BUTTON
               SHORT → Start Countdown (last-set value)
               LONG → Increase countdown every 3 sec
               ==================================================== */
            case BTN_DOWN:
            {
                ModelHandle_StartCountdown(countdownDuration);
                return;
            }

            case BTN_DOWN_LONG:
            {
                if (now - last_repeat_time >= COUNTDOWN_INC_MS)
                {
                    last_repeat_time = now;
                    edit_countdown_min++;
                    countdownDuration = (uint32_t)edit_countdown_min * 60;
                    screenNeedsRefresh = true;
                }
                return;
            }

            default:
                return;
        }
    }

    /* ============================================================
       ======================= MENU MODE ============================
       ============================================================ */
    switch (b)
    {
        /* ========================================================
           SW1 – BACK / EXIT
           ======================================================== */
        case BTN_RESET:
            menu_reset();
            return;

        /* ========================================================
           SW2 – SELECT / ENTER
           ======================================================== */
        case BTN_SELECT:
            menu_select();
            screenNeedsRefresh = true;
            return;

        /* ========================================================
           SW3 – UP / INCREASE
           ======================================================== */
        case BTN_UP:
            if (ui == UI_MENU)
            {
                if (menu_idx > 0)
                    menu_idx--;
            }
            else
                increase_edit_value();

            screenNeedsRefresh = true;
            return;

        case BTN_UP_LONG:
            if (now - last_repeat_time >= CONTINUOUS_STEP_MS)
            {
                last_repeat_time = now;
                increase_edit_value();
                screenNeedsRefresh = true;
            }
            return;

        /* ========================================================
           SW4 – DOWN / DECREASE
           ======================================================== */
        case BTN_DOWN:
            if (ui == UI_MENU)
            {
                if (menu_idx < MAIN_MENU_COUNT - 1)
                    menu_idx++;
            }
            else
                decrease_edit_value();

            screenNeedsRefresh = true;
            return;

        case BTN_DOWN_LONG:
            if (now - last_repeat_time >= CONTINUOUS_STEP_MS)
            {
                last_repeat_time = now;
                decrease_edit_value();
                screenNeedsRefresh = true;
            }
            return;

        default:
            return;
    }
}
