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
    UI_TIMER_EDIT_ON_H,
    UI_TIMER_EDIT_ON_M,
    UI_TIMER_EDIT_OFF_H,
    UI_TIMER_EDIT_OFF_M,
    UI_SEARCH,
    UI_SEARCH_EDIT_GAP,
    UI_SEARCH_EDIT_DRY,
    UI_COUNTDOWN,
    UI_COUNTDOWN_EDIT_MIN,
    UI_COUNTDOWN_EDIT_REP,
    UI_COUNTDOWN_TOGGLE,   // NEW: enable/disable screen
    UI_TWIST,
    UI_TWIST_EDIT_ON,
    UI_TWIST_EDIT_OFF,
    UI_MAX_
} UiState;

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
extern SearchSettings searchSettings;
extern TwistSettings  twistSettings;
extern volatile bool  countdownActive;
extern volatile uint32_t countdownDuration;
extern volatile bool  countdownMode;
extern volatile bool  manualActive;
extern volatile bool  semiAutoActive;
extern volatile bool  timerActive;
extern volatile bool  searchActive;
extern volatile bool  twistActive;

static bool semiAutoEnabled = false;

extern bool Motor_GetStatus(void);

/* ===== Menu definitions ===== */
static const char* main_menu[] = {
    "Manual Mode",
    "Semi-Auto Mode",
    "Timer Mode",
    "Search Mode",
    "Countdown",
    "Twist Mode",
    "Back to Dash"
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
static uint16_t edit_countdown_rep = 1;   // NEW

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
    else if (searchActive)   mode = "Search";
    else if (countdownActive)mode = "Cntdwn";
    else if (twistActive)    mode = "Twist";

    snprintf(line0,sizeof(line0),"M:%s %s",motor,mode);

    int submergedCount = 0;
    for (int i=0; i<5; i++) {
        if (adcData.voltages[i] < 0.1f) submergedCount++;
    }

    const char *level;
    switch (submergedCount) {
        case 0:  level = "EMPTY"; break;
        case 1:  level = "LOW";   break;
        case 2:  level = "HALF";  break;
        case 3:  level = "3/4";   break;
        default: level = "FULL";  break;
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

static void show_semi_auto(void){
    char line0[17], line1[17];
    snprintf(line0,sizeof(line0),"Semi-Auto Mode");
    if (semiAutoEnabled) snprintf(line1,sizeof(line1),">Disable  Back");
    else snprintf(line1,sizeof(line1),">Enable   Back");
    lcd_line0(line0);
    lcd_line1(line1);
}

static void show_timer(void){
    char l0[17], l1[17];
    snprintf(l0,sizeof(l0),"T1 ON %02d:%02d",edit_timer_on_h,edit_timer_on_m);
    snprintf(l1,sizeof(l1),"T1 OFF %02d:%02d",edit_timer_off_h,edit_timer_off_m);
    lcd_line0(l0);
    lcd_line1(l1);
}

static void show_search(void){
    char l0[17], l1[17];
    snprintf(l0,sizeof(l0),"Sr %s G%us P%us",
             searchSettings.searchActive ? "ON " : "OFF",
             (unsigned)searchSettings.testingGapSeconds,
             (unsigned)searchSettings.dryRunTimeSeconds);
    snprintf(l1,sizeof(l1),">%s   Edit",
             searchSettings.searchActive ? "Stop" : "Enable");
    lcd_line0(l0);
    lcd_line1(l1);
}

static void show_countdown(void){
    char l0[17], l1[17];
    extern volatile uint16_t countdownRemainingRuns;
    if (countdownActive) {
        uint32_t sec = countdownDuration;
        uint32_t min = sec/60;
        uint32_t s   = sec%60;
        snprintf(l0,sizeof(l0),"Run %02u %02d:%02d",
                 (unsigned)countdownRemainingRuns,(int)min,(int)s);
        snprintf(l1,sizeof(l1),">Stop     Back");
    } else {
        snprintf(l0,sizeof(l0),"Countdown Inact");
        snprintf(l1,sizeof(l1),">Set Start Back");
    }
    lcd_line0(l0);
    lcd_line1(l1);
}


static void show_twist(void){
    char l0[17], l1[17];

    const char* status = twistSettings.twistActive ? "ON " : "OFF";
    snprintf(l0,sizeof(l0),"Tw %s %2ds/%2ds", status,
             (int)twistSettings.onDurationSeconds,
             (int)twistSettings.offDurationSeconds);

    // primary action on SELECT is Enable/Stop; UP/DOWN goes to edit
    snprintf(l1,sizeof(l1),">%s   Edit",
             twistSettings.twistActive ? "Stop" : "Enable");

    lcd_line0(l0);
    lcd_line1(l1);
}


/* ===== Apply functions ===== */
static void apply_search_settings(void){
    searchSettings.testingGapSeconds = edit_search_gap_s;
    searchSettings.dryRunTimeSeconds = edit_search_dry_s;
}

static void apply_twist_settings(void){
    twistSettings.onDurationSeconds = edit_twist_on_s;
    twistSettings.offDurationSeconds = edit_twist_off_s;
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

static void apply_timer_settings(void){
    timerSlots[0].active = true;
    timerSlots[0].onTimeSeconds  = ModelHandle_TimeToSeconds(edit_timer_on_h, edit_timer_on_m);
    timerSlots[0].offTimeSeconds = ModelHandle_TimeToSeconds(edit_timer_off_h, edit_timer_off_m);
}

/* ===== Core Update Loop ===== */
void Screen_Update(void){
    uint32_t now = HAL_GetTick();

    bool cursorBlinkActive = false;
    switch (ui) {
        case UI_MENU:
        case UI_TIMER_EDIT_ON_H:
        case UI_TIMER_EDIT_ON_M:
        case UI_TIMER_EDIT_OFF_H:
        case UI_TIMER_EDIT_OFF_M:
        case UI_SEARCH_EDIT_GAP:
        case UI_SEARCH_EDIT_DRY:
        case UI_COUNTDOWN_EDIT_MIN:
        case UI_COUNTDOWN_EDIT_REP:   // NEW: blink on repeats editor
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
            case UI_WELCOME: show_welcome(); break;
            case UI_DASH: show_dash(); break;
            case UI_MENU: show_menu(); break;
            case UI_MANUAL: show_manual(); break;
            case UI_SEMI_AUTO: show_semi_auto(); break;
            case UI_TIMER: show_timer(); break;
            case UI_SEARCH: show_search(); break;
            case UI_COUNTDOWN: show_countdown(); break;
            case UI_TWIST: show_twist(); break;

            case UI_TIMER_EDIT_ON_H: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Edit ON Hour: %02d", edit_timer_on_h);
                snprintf(l1,sizeof(l1),">Up + Dn - SelOK");
                lcd_line0(l0); lcd_line1(l1); break;
            }
            case UI_TIMER_EDIT_ON_M: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Edit ON Min: %02d", edit_timer_on_m);
                snprintf(l1,sizeof(l1),">Up + Dn - SelOK");
                lcd_line0(l0); lcd_line1(l1); break;
            }
            case UI_TIMER_EDIT_OFF_H: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Edit OFF Hr: %02d", edit_timer_off_h);
                snprintf(l1,sizeof(l1),">Up + Dn - SelOK");
                lcd_line0(l0); lcd_line1(l1); break;
            }
            case UI_TIMER_EDIT_OFF_M: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Edit OFF Mn: %02d", edit_timer_off_m);
                snprintf(l1,sizeof(l1),">Up + Dn - SelOK");
                lcd_line0(l0); lcd_line1(l1); break;
            }
            case UI_SEARCH_EDIT_GAP: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Edit Gap: %3ds", edit_search_gap_s);
                snprintf(l1,sizeof(l1),">Up + Dn - SelOK");
                lcd_line0(l0); lcd_line1(l1); break;
            }
            case UI_SEARCH_EDIT_DRY: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Edit Dry: %3ds", edit_search_dry_s);
                snprintf(l1,sizeof(l1),">Up + Dn - SelOK");
                lcd_line0(l0); lcd_line1(l1); break;
            }
            case UI_COUNTDOWN_EDIT_MIN: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Set Min: %3u", edit_countdown_min);
                snprintf(l1,sizeof(l1),">Up+Dn- SelNext");
                lcd_line0(l0); lcd_line1(l1); break;
            }
            case UI_COUNTDOWN_EDIT_REP: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Set Reps: %3u", edit_countdown_rep);
                snprintf(l1,sizeof(l1),">Up+Dn- Start");
                lcd_line0(l0); lcd_line1(l1); break;
            }

            case UI_COUNTDOWN_TOGGLE: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Countdown Setup");
                snprintf(l1,sizeof(l1),">Enable   Edit");
                lcd_line0(l0); lcd_line1(l1);
                break;
            }

            case UI_TWIST_EDIT_ON: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Edit T ON: %3ds", edit_twist_on_s);
                snprintf(l1,sizeof(l1),">Up + Dn - SelOK");
                lcd_line0(l0); lcd_line1(l1); break;
            }
            case UI_TWIST_EDIT_OFF: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Edit T OFF:%3ds", edit_twist_off_s);
                snprintf(l1,sizeof(l1),">Up + Dn - SelOK");
                lcd_line0(l0); lcd_line1(l1); break;
            }
            default:
                lcd_line0("Not Implemented");
                lcd_line1("                ");
                break;
        }
    }
}

void Screen_ShowCurrentMode(void)
{
//    extern volatile bool manualActive;
//    extern volatile bool semiAutoActive;
//    extern volatile bool timerActive;
//    extern volatile bool searchActive;
//    extern volatile bool countdownActive;
//    extern volatile bool twistActive;
//
//    /* This function must NEVER allocate heap.
//       Only change logical UI state — let Screen_Update() do rendering. */
//
//    static UiState lastUi = UI_MAX_;
//    UiState newUi;
//
//    // Determine which logical mode is active
//    if (manualActive)        newUi = UI_MANUAL;
//    else if (semiAutoActive) newUi = UI_SEMI_AUTO;
//    else if (timerActive)    newUi = UI_TIMER;
//    else if (searchActive)   newUi = UI_SEARCH;
//    else if (countdownActive)newUi = UI_COUNTDOWN;
//    else if (twistActive)    newUi = UI_TWIST;
//    else                     newUi = UI_DASH;
//
//    // Only switch UI state if mode actually changed
//    if (newUi != lastUi) {
//        ui = newUi;
//        lastUi = newUi;
//        screenNeedsRefresh = true;
//        // no lcd_clear(), no snprintf(), no lcd_send_string()
//    }
}



/* ===== Initialization / Reset ===== */
void Screen_Init(void){
    lcd_init();
    ui = UI_WELCOME;
    last_ui = UI_MAX_;
    screenNeedsRefresh = true;
    lastLcdUpdateTime = HAL_GetTick();
    refreshInactivityTimer();

    edit_search_gap_s = searchSettings.testingGapSeconds;
    edit_search_dry_s = searchSettings.dryRunTimeSeconds;
    edit_twist_on_s   = twistSettings.onDurationSeconds;
    edit_twist_off_s  = twistSettings.offDurationSeconds;
    edit_countdown_min = (uint16_t)(countdownDuration / 60u);
    if (edit_countdown_min == 0) edit_countdown_min = 5; // sane default
    if (edit_countdown_rep == 0) edit_countdown_rep = 1;
}

void Screen_ResetToHome(void){
    ui = UI_WELCOME;
    last_ui = UI_MAX_;
    screenNeedsRefresh = true;
    lastLcdUpdateTime = HAL_GetTick();
    refreshInactivityTimer();
}

/* ===== Menu actions ===== */
static void menu_select(void){
    refreshInactivityTimer();

    switch (ui){
        case UI_WELCOME: ui = UI_DASH; break;
        case UI_DASH: ui = UI_MENU; goto_menu_top(); break;

        case UI_MENU:
            switch (menu_idx){
                case 0: ui = UI_MANUAL; break;
                case 1: ui = UI_SEMI_AUTO; break;
                case 2: ui = UI_TIMER; break;
                case 3: ui = UI_SEARCH; break;
                case 4: ui = UI_COUNTDOWN; break;
                case 5: ui = UI_TWIST; break;
                case 6: ui = UI_DASH; break;
                default: ui = UI_DASH; break;
            }
            break;

        case UI_MANUAL:
            ModelHandle_ToggleManual();
            screenNeedsRefresh = true;
            break;

        case UI_SEMI_AUTO:
            if (!semiAutoEnabled) enable_semi_auto();
            else disable_semi_auto();
            ui = UI_DASH;
            break;

        case UI_TIMER: ui = UI_TIMER_EDIT_ON_H; break;
        case UI_TIMER_EDIT_ON_H: ui = UI_TIMER_EDIT_ON_M; break;
        case UI_TIMER_EDIT_ON_M: ui = UI_TIMER_EDIT_OFF_H; break;
        case UI_TIMER_EDIT_OFF_H: ui = UI_TIMER_EDIT_OFF_M; break;
        case UI_TIMER_EDIT_OFF_M:
            apply_timer_settings();
            ui = UI_TIMER;
            break;

        case UI_SEARCH: ui = UI_SEARCH_EDIT_GAP; break;
        case UI_SEARCH_EDIT_GAP: ui = UI_SEARCH_EDIT_DRY; break;
        case UI_SEARCH_EDIT_DRY:
            apply_search_settings();
            ui = UI_SEARCH; break;

        case UI_COUNTDOWN:
            if (countdownActive) {
                ModelHandle_StopCountdown();
                screenNeedsRefresh = true;
            } else {
                ui = UI_COUNTDOWN_EDIT_MIN;
            }
            break;


        case UI_COUNTDOWN_EDIT_MIN:
            ui = UI_COUNTDOWN_EDIT_REP;
            break;

        case UI_COUNTDOWN_EDIT_REP:
            // after repeats, go to enable/edit screen
            ui = UI_COUNTDOWN_TOGGLE;
            break;

        case UI_COUNTDOWN_TOGGLE:
            // Enable or Edit based on menu index
            // For simplicity, SELECT always means "Enable"
            {
                uint32_t seconds = (uint32_t)edit_countdown_min * 60u;
                if (seconds == 0) seconds = 60;
                if (edit_countdown_rep == 0) edit_countdown_rep = 1;
                apply_countdown_settings();
                ModelHandle_StartCountdown(seconds, (uint16_t)edit_countdown_rep);
                ui = UI_COUNTDOWN;
            }
            break;


        case UI_TWIST:
            // SELECT toggles enable/stop; UP/DOWN will switch to edit states
            if (twistSettings.twistActive) {
                ModelHandle_StopTwist();
            } else {
                // use current edit buffers (or the applied values) to start
                ModelHandle_StartTwist(edit_twist_on_s, edit_twist_off_s);
            }
            screenNeedsRefresh = true;
            break;

        case UI_TWIST_EDIT_ON:      ui = UI_TWIST_EDIT_OFF; break;
        case UI_TWIST_EDIT_OFF:
            apply_twist_settings();   // writes into twistSettings
            // If active, apply live (optional, keeps running with new values)
            if (twistSettings.twistActive) {
                ModelHandle_StartTwist(twistSettings.onDurationSeconds,
                                       twistSettings.offDurationSeconds);
            }
            ui = UI_TWIST; break;


        default: break;
    }
    screenNeedsRefresh = true;
}

static void menu_reset(void){
    refreshInactivityTimer();

    switch (ui) {
        case UI_MANUAL:
        case UI_SEMI_AUTO:
        case UI_TIMER:
        case UI_SEARCH:
        case UI_COUNTDOWN:
        case UI_TWIST: ui = UI_MENU; break;
        case UI_MENU: ui = UI_DASH; break;
        case UI_DASH: ui = UI_WELCOME; break;
        case UI_TIMER_EDIT_ON_H:
        case UI_TIMER_EDIT_ON_M:
        case UI_TIMER_EDIT_OFF_H:
        case UI_TIMER_EDIT_OFF_M: ui = UI_TIMER; break;
        case UI_SEARCH_EDIT_GAP:
        case UI_SEARCH_EDIT_DRY: ui = UI_SEARCH; break;
        case UI_COUNTDOWN_EDIT_MIN:
        case UI_COUNTDOWN_EDIT_REP: ui = UI_COUNTDOWN; break;  // NEW: back from repeats editor
        case UI_TWIST_EDIT_ON:
        case UI_TWIST_EDIT_OFF: ui = UI_TWIST; break;
        default: ui = UI_MENU; break;
    }
    screenNeedsRefresh = true;
}

/* ===== Public button handler ===== */
void Screen_HandleButton(UiButton b)
{
    if (b == BTN_NONE) return;

    /* Quick RESET → manual toggle (your existing behavior) */
    if (b == BTN_RESET) {
        ModelHandle_ToggleManual();
        return;
    }

    /* =======================
       UP key
       ======================= */
    if (b == BTN_UP) {
        switch (ui) {
            case UI_MENU: if (menu_idx > 0) menu_idx--; break;

            /* Timer edits */
            case UI_TIMER_EDIT_ON_H:  if (edit_timer_on_h  < 23) edit_timer_on_h++;  break;
            case UI_TIMER_EDIT_ON_M:  if (edit_timer_on_m  < 59) edit_timer_on_m++;  break;
            case UI_TIMER_EDIT_OFF_H: if (edit_timer_off_h < 23) edit_timer_off_h++; break;
            case UI_TIMER_EDIT_OFF_M: if (edit_timer_off_m < 59) edit_timer_off_m++; break;

            /* Search edits */
            case UI_SEARCH:            ui = UI_SEARCH_EDIT_GAP; screenNeedsRefresh = true; return;
            case UI_SEARCH_EDIT_GAP:   edit_search_gap_s += 5;  break;
            case UI_SEARCH_EDIT_DRY:   edit_search_dry_s += 1;  break;

            /* Countdown edits */
            case UI_COUNTDOWN_EDIT_MIN: edit_countdown_min++; break;
            case UI_COUNTDOWN_EDIT_REP: edit_countdown_rep++; break;
            case UI_COUNTDOWN_TOGGLE:   ui = UI_COUNTDOWN_EDIT_MIN; screenNeedsRefresh = true; return;

            /* Twist edits */
            case UI_TWIST:             ui = UI_TWIST_EDIT_ON; screenNeedsRefresh = true; return;
            case UI_TWIST_EDIT_ON:     if (edit_twist_on_s  < 600) edit_twist_on_s++;  break;
            case UI_TWIST_EDIT_OFF:    if (edit_twist_off_s < 600) edit_twist_off_s++; break;

            default: break;
        }
        screenNeedsRefresh = true;
        return;
    }

    /* =======================
       DOWN key
       ======================= */
    if (b == BTN_DOWN) {
        switch (ui) {
            case UI_MENU: if (menu_idx < (int)(MAIN_MENU_COUNT - 1)) menu_idx++; break;

            /* Timer edits */
            case UI_TIMER_EDIT_ON_H:  if (edit_timer_on_h  > 0) edit_timer_on_h--;  break;
            case UI_TIMER_EDIT_ON_M:  if (edit_timer_on_m  > 0) edit_timer_on_m--;  break;
            case UI_TIMER_EDIT_OFF_H: if (edit_timer_off_h > 0) edit_timer_off_h--; break;
            case UI_TIMER_EDIT_OFF_M: if (edit_timer_off_m > 0) edit_timer_off_m--; break;

            /* Search edits */
            case UI_SEARCH:            ui = UI_SEARCH_EDIT_GAP; screenNeedsRefresh = true; return;
            case UI_SEARCH_EDIT_GAP:   if (edit_search_gap_s > 1) edit_search_gap_s -= 5; break;
            case UI_SEARCH_EDIT_DRY:   if (edit_search_dry_s > 1) edit_search_dry_s -= 1; break;

            /* Countdown edits */
            case UI_COUNTDOWN_EDIT_MIN: if (edit_countdown_min > 1) edit_countdown_min--; break;
            case UI_COUNTDOWN_EDIT_REP: if (edit_countdown_rep > 1) edit_countdown_rep--; break;

            /* Twist edits */
            case UI_TWIST_EDIT_ON:   if (edit_twist_on_s  > 1) edit_twist_on_s--;  break;
            case UI_TWIST_EDIT_OFF:  if (edit_twist_off_s > 1) edit_twist_off_s--; break;

            default: break;
        }
        screenNeedsRefresh = true;
        return;
    }

    /* =======================
       SELECT key
       ======================= */
    if (b == BTN_SELECT) {
        switch (ui) {

            /* ---- Search main: Enable/Stop ---- */
            case UI_SEARCH:
                if (searchSettings.searchActive) {
                    ModelHandle_StopSearch();
                } else {
                    uint16_t gap_s   = (uint16_t)edit_search_gap_s;  if (gap_s   == 0) gap_s   = 5;
                    uint16_t probe_s = (uint16_t)edit_search_dry_s;  if (probe_s == 0) probe_s = 3;
                    ModelHandle_StartSearch(gap_s, probe_s);
                }
                screenNeedsRefresh = true;
                return;

            /* ---- Search edit flow ---- */
            case UI_SEARCH_EDIT_GAP:
                ui = UI_SEARCH_EDIT_DRY;
                screenNeedsRefresh = true;
                return;

            case UI_SEARCH_EDIT_DRY:
                if (edit_search_gap_s < 1) edit_search_gap_s = 1;
                if (edit_search_dry_s < 1) edit_search_dry_s = 1;

                searchSettings.testingGapSeconds = (uint16_t)edit_search_gap_s;
                searchSettings.dryRunTimeSeconds = (uint16_t)edit_search_dry_s;

                if (searchSettings.searchActive) {
                    ModelHandle_StartSearch((uint16_t)searchSettings.testingGapSeconds,
                                            (uint16_t)searchSettings.dryRunTimeSeconds);
                }
                ui = UI_SEARCH;
                screenNeedsRefresh = true;
                return;

            /* ---- Twist main: Enable/Stop ---- */
            case UI_TWIST:
                if (twistSettings.twistActive) {
                    ModelHandle_StopTwist();
                } else {
                    uint16_t on_s  = (uint16_t)edit_twist_on_s;  if (on_s  == 0) on_s  = 1;
                    uint16_t off_s = (uint16_t)edit_twist_off_s; if (off_s == 0) off_s = 1;
                    ModelHandle_StartTwist(on_s, off_s);
                }
                screenNeedsRefresh = true;
                return;

            /* ---- Twist edit ---- */
            case UI_TWIST_EDIT_ON:
                ui = UI_TWIST_EDIT_OFF;
                screenNeedsRefresh = true;
                return;

            case UI_TWIST_EDIT_OFF:
                if (edit_twist_off_s < 1)   edit_twist_off_s = 1;
                if (edit_twist_off_s > 600) edit_twist_off_s = 600;
                if (edit_twist_on_s  < 1)   edit_twist_on_s  = 1;
                if (edit_twist_on_s  > 600) edit_twist_on_s  = 600;

                apply_twist_settings(); // copies edit_* to twistSettings

                if (twistSettings.twistActive) {
                    ModelHandle_StartTwist((uint16_t)twistSettings.onDurationSeconds,
                                           (uint16_t)twistSettings.offDurationSeconds);
                }
                ui = UI_TWIST;
                screenNeedsRefresh = true;
                return;

            /* ---- Others: keep your existing flow ---- */
            default:
                menu_select();
                return;
        }
    }
}
/* ===== Switch polling with long-press detection ===== */
/* ===== Switch polling with long-press detection ===== */
void Screen_HandleSwitches(void){
    static const struct { GPIO_TypeDef* port; uint16_t pin; UiButton btn; uint16_t ledPin; } switchMap[] = {
        {SWITCH1_GPIO_Port, SWITCH1_Pin, BTN_RESET,  LED1_Pin}, // Manual
        {SWITCH2_GPIO_Port, SWITCH2_Pin, BTN_SELECT, LED2_Pin}, // Select
        {SWITCH3_GPIO_Port, SWITCH3_Pin, BTN_UP,     LED3_Pin}, // Up  (→ Summer Save 5s long-press)
        {SWITCH4_GPIO_Port, SWITCH4_Pin, BTN_DOWN,   LED4_Pin}  // Down / Back
    };
    static bool prev[4] = {true,true,true,true};
    static uint32_t pressStart[4] = {0,0,0,0};

    for (int i=0; i<4; i++){
        bool pressed = (HAL_GPIO_ReadPin(switchMap[i].port, switchMap[i].pin) == GPIO_PIN_RESET);

        // edge: press started
        if (pressed && prev[i]) {
            prev[i] = false;
            pressStart[i] = HAL_GetTick();
        }
        // edge: released
        else if (!pressed && !prev[i]) {
            uint32_t pressDuration = HAL_GetTick() - pressStart[i];
            prev[i] = true;

            /* ---- SW3 special: >=5s => "Summer Save" burst (R2+R3 ON for 30s) ---- */
            if (i == 2 && pressDuration >= 3000UL) {
                ModelHandle_TriggerAuxBurst(30);

                // Quick LCD notice
                lcd_clear();
                lcd_put_cur(0, 0);
                lcd_send_string("Summer Save 30s");
                lcd_put_cur(1, 0);
                lcd_send_string("Relays 2&3 ON");

                // optional LED feedback
                HAL_GPIO_TogglePin(switchMap[i].port, switchMap[i].ledPin);
            }
            /* ---- SW1 ≥2s = manual long-press/restart (kept as-is) ---- */
            else if (i == 0 && pressDuration > 2000UL) {
                ModelHandle_ManualLongPress();
            }
            /* ---- SW4 ≥2s = Back/menu reset (kept as-is) ---- */
            else if (i == 3 && pressDuration > 2000UL) {
                menu_reset();
            }
            /* ---- Short press → normal handling ---- */
            else {
                HAL_GPIO_TogglePin(switchMap[i].port, switchMap[i].ledPin);
                Screen_HandleButton(switchMap[i].btn);
            }
        }
    }
}
