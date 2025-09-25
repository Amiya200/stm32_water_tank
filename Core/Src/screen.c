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
#include "stm32f1xx_hal.h" // for HAL_GetTick and GPIO

/* ===== UI Buttons ===== */
typedef enum {
    BTN_NONE = 0,
    BTN_RESET,   // SW1
    BTN_SELECT,  // SW2
    BTN_UP,      // SW3
    BTN_DOWN     // SW4
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
static uint8_t edit_timer_on_h = 6, edit_timer_on_m = 30;
static uint8_t edit_timer_off_h = 18, edit_timer_off_m = 30;
static uint16_t edit_search_gap_s = 60, edit_search_dry_s = 10;
static uint16_t edit_twist_on_s = 5, edit_twist_off_s = 5;
static uint16_t edit_countdown_min = 5;

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
    if (idx < MAIN_MENU_COUNT && idx >= 0) {
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
    // Motor status
    const char *motor = Motor_GetStatus() ? "ON" : "OFF";
    snprintf(line0,sizeof(line0),"Motor:%-3s",motor);

    // Water level based on count of sensors 0–4 reading <0.1V
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
    snprintf(l0,sizeof(l0),"Gap:%3ds Dry:%3ds",edit_search_gap_s,edit_search_dry_s);
    snprintf(l1,sizeof(l1),">Edit     Back");
    lcd_line0(l0);
    lcd_line1(l1);
}

static void show_countdown(void){
    char l0[17], l1[17];
    if (countdownActive) {
        uint32_t sec = countdownDuration;
        uint32_t min = sec/60;
        uint32_t s = sec%60;
        snprintf(l0,sizeof(l0),"Count %02d:%02d",(int)min,(int)s);
    } else {
        snprintf(l0,sizeof(l0),"Countdown Inact");
    }
    snprintf(l1,sizeof(l1),">Set Start Back");
    lcd_line0(l0);
    lcd_line1(l1);
}

static void show_twist(void){
    char l0[17], l1[17];
    snprintf(l0,sizeof(l0),"Twist ON:%3ds",edit_twist_on_s);
    snprintf(l1,sizeof(l1),"Twist OFF:%3ds",edit_twist_off_s);
    lcd_line0(l0);
    lcd_line1(l1);
}

/* === apply + rest of code remains the same (no changes needed) === */
// (keep your Screen_Update, Screen_Init, Screen_HandleButton etc. logic untouched)
// Just keep all snprintf() lines trimmed to <=16 characters like above

/* ===== Apply functions (write edits to real settings) ===== */
static void apply_search_settings(void){
    searchSettings.testingGapSeconds = edit_search_gap_s;
    searchSettings.dryRunTimeSeconds = edit_search_dry_s;
    // If you need to call any apply function or save to non-volatile, do it here.
}

static void apply_twist_settings(void){
    twistSettings.onDurationSeconds = edit_twist_on_s;
    twistSettings.offDurationSeconds = edit_twist_off_s;
}

static void apply_countdown_settings(void){
    countdownDuration = (uint32_t)edit_countdown_min * 60u;
}

static void apply_timer_settings(void){
    // You likely have a TimerSlot structure — update it here if fields are known.
    // For safety we only store local edit variables; adapt to your timerSlots[] structure.
    (void)edit_timer_on_h;
    (void)edit_timer_on_m;
    (void)edit_timer_off_h;
    (void)edit_timer_off_m;
    // e.g. timerSlots[0].onHour = edit_timer_on_h; ...
}

/* ===== Core Update Loop ===== */

void Screen_Update(void){
    uint32_t now = HAL_GetTick();

    // Cursor blink on menu and all edit screens
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
        case UI_TWIST_EDIT_ON:
        case UI_TWIST_EDIT_OFF:
            cursorBlinkActive = true;
            break;
        default:
            cursorBlinkActive = false;
            cursorVisible = true; // always visible outside blinking screens
            break;
    }

    if (cursorBlinkActive && (now - lastCursorToggle >= CURSOR_BLINK_MS)) {
        cursorVisible = !cursorVisible;
        lastCursorToggle = now;
        screenNeedsRefresh = true;
    }

    /* Welcome timeout */
    if (ui == UI_WELCOME && now - lastLcdUpdateTime >= WELCOME_MS) {
        ui = UI_DASH;
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }

    /* Optional auto-back to dashboard after inactivity in menus/edit screens */
    if (ui != UI_WELCOME && ui != UI_DASH && (now - lastUserAction >= AUTO_BACK_MS)) {
        ui = UI_DASH;
        screenNeedsRefresh = true;
    }

    // Always refresh dashboard every 1s
    if (ui == UI_DASH && (now - lastLcdUpdateTime >= 1000)) {
        screenNeedsRefresh = true;
        lastLcdUpdateTime = now;
    }

    if (screenNeedsRefresh || ui != last_ui) {
    	bool fullRedraw = (ui != last_ui);  // only clear if state changes
        last_ui = ui;
        screenNeedsRefresh = false;

        if (fullRedraw) {
               lcd_clear();   // clear only on state change
           }
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

            /* Editing screens show a small edit UI permitting up/down to change value. */
            case UI_TIMER_EDIT_ON_H: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Edit ON Hour: %02d  ", edit_timer_on_h);
                snprintf(l1,sizeof(l1),">Up +  Dn -  Sel OK");
                lcd_line0(l0);
                lcd_line1(l1);
                break;
            }
            case UI_TIMER_EDIT_ON_M: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Edit ON Min: %02d  ", edit_timer_on_m);
                snprintf(l1,sizeof(l1),">Up +  Dn -  Sel OK");
                lcd_line0(l0);
                lcd_line1(l1);
                break;
            }
            case UI_TIMER_EDIT_OFF_H: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Edit OFF Hr: %02d  ", edit_timer_off_h);
                snprintf(l1,sizeof(l1),">Up +  Dn -  Sel OK");
                lcd_line0(l0);
                lcd_line1(l1);
                break;
            }
            case UI_TIMER_EDIT_OFF_M: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Edit OFF Mn: %02d  ", edit_timer_off_m);
                snprintf(l1,sizeof(l1),">Up +  Dn -  Sel OK");
                lcd_line0(l0);
                lcd_line1(l1);
                break;
            }
            case UI_SEARCH_EDIT_GAP: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Edit Gap: %3ds  ", edit_search_gap_s);
                snprintf(l1,sizeof(l1),">Up +  Dn -  Sel OK");
                lcd_line0(l0);
                lcd_line1(l1);
                break;
            }
            case UI_SEARCH_EDIT_DRY: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Edit Dry: %3ds  ", edit_search_dry_s);
                snprintf(l1,sizeof(l1),">Up +  Dn -  Sel OK");
                lcd_line0(l0);
                lcd_line1(l1);
                break;
            }
            case UI_COUNTDOWN_EDIT_MIN: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Set Min: %3d    ", edit_countdown_min);
                snprintf(l1,sizeof(l1),">Up +  Dn -  Sel Start");
                lcd_line0(l0);
                lcd_line1(l1);
                break;
            }
            case UI_TWIST_EDIT_ON: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Edit T ON: %3ds  ", edit_twist_on_s);
                snprintf(l1,sizeof(l1),">Up +  Dn -  Sel OK");
                lcd_line0(l0);
                lcd_line1(l1);
                break;
            }
            case UI_TWIST_EDIT_OFF: {
                char l0[17], l1[17];
                snprintf(l0,sizeof(l0),"Edit T OFF: %3ds ", edit_twist_off_s);
                snprintf(l1,sizeof(l1),">Up +  Dn -  Sel OK");
                lcd_line0(l0);
                lcd_line1(l1);
                break;
            }
            default:
                lcd_line0("Not Implemented   ");
                lcd_line1("                  ");
                break;
        }
    }
}

/* ===== Initialization / Reset ===== */

void Screen_Init(void){
    lcd_init();
    ui = UI_WELCOME;
    last_ui = UI_MAX_;
    screenNeedsRefresh = true;
    lastLcdUpdateTime = HAL_GetTick();
    refreshInactivityTimer();

    // initialize edits from actual settings
    edit_search_gap_s = searchSettings.testingGapSeconds;
    edit_search_dry_s = searchSettings.dryRunTimeSeconds;
    edit_twist_on_s = twistSettings.onDurationSeconds;
    edit_twist_off_s = twistSettings.offDurationSeconds;
    // countdown: convert seconds to minutes if possible
    edit_countdown_min = (uint16_t)(countdownDuration / 60u);
}

void Screen_ResetToHome(void){
    ui = UI_WELCOME;
    last_ui = UI_MAX_;
    screenNeedsRefresh = true;
    lastLcdUpdateTime = HAL_GetTick();
    refreshInactivityTimer();
}

/* ===== Button actions ===== */

static void menu_move_up(void){
    if (ui == UI_MENU) {
        if (menu_idx > 0) menu_idx--;
        // adjust view top
        if (menu_idx < menu_view_top) menu_view_top = menu_idx;
        screenNeedsRefresh = true;
    } else if (ui == UI_MANUAL) {
        // Manual: UP/DOWN unused (you could implement speed or similar)
    } else {
        // in edit screens: increment value
        switch (ui){
            case UI_TIMER_EDIT_ON_H: if (edit_timer_on_h < 23) edit_timer_on_h++; break;
            case UI_TIMER_EDIT_ON_M: if (edit_timer_on_m < 59) edit_timer_on_m++; break;
            case UI_TIMER_EDIT_OFF_H: if (edit_timer_off_h < 23) edit_timer_off_h++; break;
            case UI_TIMER_EDIT_OFF_M: if (edit_timer_off_m < 59) edit_timer_off_m++; break;
            case UI_SEARCH_EDIT_GAP: if (edit_search_gap_s < 3600) edit_search_gap_s += 1; break;
            case UI_SEARCH_EDIT_DRY: if (edit_search_dry_s < 3600) edit_search_dry_s += 1; break;
            case UI_COUNTDOWN_EDIT_MIN: if (edit_countdown_min < 999) edit_countdown_min += 1; break;
            case UI_TWIST_EDIT_ON: if (edit_twist_on_s < 3600) edit_twist_on_s += 1; break;
            case UI_TWIST_EDIT_OFF: if (edit_twist_off_s < 3600) edit_twist_off_s += 1; break;
            default: break;
        }
        screenNeedsRefresh = true;
    }
    refreshInactivityTimer();
}

static void menu_move_down(void){
    if (ui == UI_MENU) {
        if (menu_idx < (MAIN_MENU_COUNT-1)) menu_idx++;
        // adjust view top
        if (menu_idx > menu_view_top + 1) menu_view_top = menu_idx - 1;
        screenNeedsRefresh = true;
    } else {
        // edit screens: decrement value
        switch (ui){
            case UI_TIMER_EDIT_ON_H: if (edit_timer_on_h > 0) edit_timer_on_h--; break;
            case UI_TIMER_EDIT_ON_M: if (edit_timer_on_m > 0) edit_timer_on_m--; break;
            case UI_TIMER_EDIT_OFF_H: if (edit_timer_off_h > 0) edit_timer_off_h--; break;
            case UI_TIMER_EDIT_OFF_M: if (edit_timer_off_m > 0) edit_timer_off_m--; break;
            case UI_SEARCH_EDIT_GAP: if (edit_search_gap_s > 1) edit_search_gap_s -= 1; break;
            case UI_SEARCH_EDIT_DRY: if (edit_search_dry_s > 1) edit_search_dry_s -= 1; break;
            case UI_COUNTDOWN_EDIT_MIN: if (edit_countdown_min > 0) edit_countdown_min -= 1; break;
            case UI_TWIST_EDIT_ON: if (edit_twist_on_s > 1) edit_twist_on_s -= 1; break;
            case UI_TWIST_EDIT_OFF: if (edit_twist_off_s > 1) edit_twist_off_s -= 1; break;
            default: break;
        }
        screenNeedsRefresh = true;
    }
    refreshInactivityTimer();
}

static void menu_select(void){
    refreshInactivityTimer();

    switch (ui){
        case UI_WELCOME:
            ui = UI_DASH;
            break;

        case UI_DASH:
            // Enter menu
            ui = UI_MENU;
            goto_menu_top();
            break;

        case UI_MENU:
            // perform action based on menu_idx
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
            // Toggle motor on select

            if (Motor_GetStatus()) ModelHandle_SetMotor(false);

            else ModelHandle_SetMotor(true);

            screenNeedsRefresh = true;
            break;

        case UI_SEMI_AUTO:
            semiAutoEnabled = !semiAutoEnabled;
            screenNeedsRefresh = true;
            break;

        case UI_TIMER:
            // From timer display, SELECT enters editing ON time first
            ui = UI_TIMER_EDIT_ON_H;
            break;

        case UI_TIMER_EDIT_ON_H:
            ui = UI_TIMER_EDIT_ON_M; break;
        case UI_TIMER_EDIT_ON_M:
            ui = UI_TIMER_EDIT_OFF_H; break;
        case UI_TIMER_EDIT_OFF_H:
            ui = UI_TIMER_EDIT_OFF_M; break;
        case UI_TIMER_EDIT_OFF_M:
            // finished editing timer; apply
            apply_timer_settings();
            ui = UI_TIMER; break;

        case UI_SEARCH:
            // enter edit gap
            ui = UI_SEARCH_EDIT_GAP; break;

        case UI_SEARCH_EDIT_GAP:
            // after editing gap go to dry
            apply_search_settings(); // partial apply
            ui = UI_SEARCH_EDIT_DRY; break;
        case UI_SEARCH_EDIT_DRY:
            apply_search_settings();
            ui = UI_SEARCH; break;

        case UI_COUNTDOWN:
            // if countdown active, SELECT stops it, otherwise go to set/start
            if (countdownActive) {
                countdownActive = false;
            } else {
                ui = UI_COUNTDOWN_EDIT_MIN;
            }
            break;
        case UI_COUNTDOWN_EDIT_MIN:
            apply_countdown_settings();
            // start countdown
            countdownActive = true;
            ui = UI_COUNTDOWN;
            break;

        case UI_TWIST:
            // SELECT enters edit ON duration
            ui = UI_TWIST_EDIT_ON; break;
        case UI_TWIST_EDIT_ON:
            // next selects edit off
            ui = UI_TWIST_EDIT_OFF; break;
        case UI_TWIST_EDIT_OFF:
            // finished editing twist -> apply
            apply_twist_settings();
            ui = UI_TWIST; break;

        /* Edit screens handled above */
        default:
            ui = UI_DASH;
            break;
    }

    screenNeedsRefresh = true;
}

static void menu_reset(void){
    // act as back/home
    refreshInactivityTimer();

    switch (ui){
        case UI_WELCOME:
            // do nothing
            break;
        case UI_DASH:
            // already at dash - go to welcome
            ui = UI_WELCOME;
            break;
        case UI_MENU:
            // go back to dash
            ui = UI_DASH;
            break;
        case UI_MANUAL:
        case UI_SEMI_AUTO:
        case UI_TIMER:
        case UI_SEARCH:
        case UI_COUNTDOWN:
        case UI_TWIST:
            // return to menu
            ui = UI_MENU;
            break;
        case UI_TIMER_EDIT_ON_H:
        case UI_TIMER_EDIT_ON_M:
        case UI_TIMER_EDIT_OFF_H:
        case UI_TIMER_EDIT_OFF_M:
        case UI_SEARCH_EDIT_GAP:
        case UI_SEARCH_EDIT_DRY:
        case UI_COUNTDOWN_EDIT_MIN:
        case UI_TWIST_EDIT_ON:
        case UI_TWIST_EDIT_OFF:
            // cancel edits, go to parent screen (menu for simplicity)
            ui = UI_MENU;
            break;
        default:
            ui = UI_DASH;
            break;
    }
    screenNeedsRefresh = true;
}

/* Public button handler used by switch polling */
void Screen_HandleButton(UiButton b){
    if (b == BTN_RESET){ menu_reset(); return; }
    if (b == BTN_UP) { menu_move_up(); return; }
    if (b == BTN_DOWN) { menu_move_down(); return; }
    if (b == BTN_SELECT) { menu_select(); return; }
}

/* Generic switch handler - keeps your previous mapping style (change ports/pins if different) */
void Screen_HandleSwitches(void){
    static const struct { GPIO_TypeDef* port; uint16_t pin; UiButton btn; uint16_t ledPin; } switchMap[] = {
        {SWITCH1_GPIO_Port, SWITCH1_Pin, BTN_RESET, LED1_Pin},
        {SWITCH2_GPIO_Port, SWITCH2_Pin, BTN_SELECT, LED2_Pin},
        {SWITCH3_GPIO_Port, SWITCH3_Pin, BTN_UP, LED3_Pin},
        {SWITCH4_GPIO_Port, SWITCH4_Pin, BTN_DOWN, LED4_Pin}
    };
    static bool prev[4] = {true,true,true,true};

    for (int i=0; i<4; i++){
        bool pressed = (HAL_GPIO_ReadPin(switchMap[i].port, switchMap[i].pin) == GPIO_PIN_RESET);
        if (pressed && prev[i]) {
            prev[i] = false;
            // toggle corresponding indicator LED (assuming LED pin is on same port)
            HAL_GPIO_TogglePin(switchMap[i].port, switchMap[i].ledPin);
            Screen_HandleButton(switchMap[i].btn);
        } else if (!pressed) prev[i] = true;
    }
}
