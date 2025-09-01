#include "screen.h"
#include "lcd_i2c.h"
#include "switches.h"
#include "model_handle.h"
#include "adc.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* ===== UI buttons ===== */
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
    UI_DASH_WATER,
    UI_DASH_MODE,
    UI_DASH_SEARCH,
    UI_DASH_TWIST,
    UI_MENU,
    UI_EDIT_COUNTDOWN_MIN,
    UI_EDIT_TIMER1_ON_H, UI_EDIT_TIMER1_ON_M,
    UI_EDIT_TIMER1_OFF_H, UI_EDIT_TIMER1_OFF_M,
    UI_EDIT_SEARCH_GAP_M, UI_EDIT_SEARCH_GAP_S,
    UI_EDIT_SEARCH_DRY_M, UI_EDIT_SEARCH_DRY_S,
    UI_EDIT_TWIST_ON_M, UI_EDIT_TWIST_ON_S,
    UI_EDIT_TWIST_OFF_M, UI_EDIT_TWIST_OFF_S,
    UI_CONFIRM_MANUAL_ON,
    UI_CONFIRM_MANUAL_OFF,
    UI_MAX_
} UiState;

/* ===== Timing ===== */
static uint32_t lastLcdUpdateTime = 0;
static const uint32_t WELCOME_MS = 3000;
static const uint32_t PAGE_MS    = 4000; // 4 sec per page
static const uint32_t CURSOR_BLINK_MS = 500;

static UiState ui = UI_WELCOME;
static UiState last_ui = UI_MAX_;
static bool screenNeedsRefresh = false;
static bool cursorVisible = true;
static uint32_t lastCursorToggle = 0;

/* ===== Externals ===== */
extern ADC_Data adcData;
extern TimerSlot timerSlots[5];
extern SearchSettings searchSettings;
extern TwistSettings  twistSettings;
extern volatile bool  countdownActive;
extern volatile uint32_t countdownDuration;
extern volatile bool  countdownMode;
extern bool Motor_GetStatus(void);

/* ===== Buffers ===== */
static char buf[21];
static uint8_t temp_h=0, temp_m=0, temp_s=0;
static uint8_t menu_idx = 0;

static const char* menu_items[] = {
    "Manual: ON", "Manual: OFF",
    "Countdown (min)",
    "Timer1 ON", "Timer1 OFF",
    "Search Gap", "Search DryRun",
    "Twist ON dur", "Twist OFF dur",
    "Back to Dash"
};
#define MENU_COUNT (sizeof(menu_items)/sizeof(menu_items[0]))

/* ===== Helper: LCD Wrappers ===== */
static inline void lcd_line(uint8_t row, const char* s) {
    char ln[21];
    snprintf(ln, sizeof(ln), "%-20s", s); // pad with spaces
    lcd_put_cur(row, 0);
    lcd_send_string(ln);
}
static inline void lcd_line0(const char* s){ lcd_line(0,s); }
static inline void lcd_line1(const char* s){ lcd_line(1,s); }

/* ================= UI Render Functions ================= */
static void show_welcome(void){
    lcd_clear();
    lcd_line0("Welcome to");
    lcd_line1("HELONIX");
}
static void show_dash_water(void){
//    snprintf(buf, sizeof(buf), "Water V0: %.2fV", adcData.voltages[0]);
    lcd_line0(buf);

    if      (adcData.voltages[0] > 2.5f)  lcd_line1("Status: Full");
    else if (adcData.voltages[0] > 1.0f)  lcd_line1("Status: Half");
    else if (adcData.voltages[0] > 0.1f)  lcd_line1("Status: Low");
    else                                  lcd_line1("Status: Empty");
}
static void show_dash_mode(void){
    snprintf(buf, sizeof(buf), "Motor:%s Cnt:%s",
             Motor_GetStatus() ? "ON":"OFF",
             countdownActive ? (countdownMode?"ON":"OFF") : "NA");
    lcd_line0(buf);
    lcd_line1("Menu: Press SEL");
}
static void show_dash_search(void){
    lcd_line0("Search Mode");
    if (searchSettings.searchActive) {
        snprintf(buf, sizeof(buf), "Gap:%ds Dry:%ds",
                 (int)searchSettings.testingGapSeconds,
                 (int)searchSettings.dryRunTimeSeconds);
        lcd_line1(buf);
    } else lcd_line1("Inactive");
}
static void show_dash_twist(void){
    lcd_line0("Twist Mode");
    if (twistSettings.twistActive) {
        snprintf(buf, sizeof(buf), "ON:%ds OFF:%ds",
                 (int)twistSettings.onDurationSeconds,
                 (int)twistSettings.offDurationSeconds);
        lcd_line1(buf);
    } else lcd_line1("Inactive");
}
static void show_menu(void){
    char line[21];
    snprintf(line, sizeof(line), " %s", menu_items[menu_idx]);
    lcd_put_cur(0, 1);
    lcd_send_string(line);
    lcd_line1("UP/DN:Move SEL:OK");

    // Cursor blinking handled separately
}

/* ================= Menu Handling ================= */
static void goto_dash_cycle(void) {
    if (ui < UI_DASH_WATER || ui > UI_DASH_TWIST) ui = UI_DASH_WATER;
}
static void apply_menu_action(void){
    static const UiState actions[] = {
        UI_CONFIRM_MANUAL_ON, UI_CONFIRM_MANUAL_OFF,
        UI_EDIT_COUNTDOWN_MIN,
        UI_EDIT_TIMER1_ON_H, UI_EDIT_TIMER1_OFF_H,
        UI_EDIT_SEARCH_GAP_M, UI_EDIT_SEARCH_DRY_M,
        UI_EDIT_TWIST_ON_M, UI_EDIT_TWIST_OFF_M,
        UI_DASH_WATER
    };
    if (menu_idx < MENU_COUNT) ui = actions[menu_idx];
}

/* ================= Screen Core ================= */
void Screen_Update(void){
    uint32_t now = HAL_GetTick();

    /* Cursor blink */
    if (ui == UI_MENU && (now - lastCursorToggle >= CURSOR_BLINK_MS)) {
        cursorVisible = !cursorVisible;
        lastCursorToggle = now;
        lcd_put_cur(0, 0);
        lcd_send_data(cursorVisible ? '>' : ' ');
    }

    /* Page cycling */
    if (ui >= UI_DASH_WATER && ui <= UI_DASH_TWIST &&
        now - lastLcdUpdateTime >= PAGE_MS) {
        ui = (UiState)(ui + 1);
        if (ui > UI_DASH_TWIST) ui = UI_DASH_WATER;
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }

    /* Welcome timeout */
    if (ui == UI_WELCOME && now - lastLcdUpdateTime >= WELCOME_MS) {
        ui = UI_DASH_WATER;
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }

    /* Render */
    if (screenNeedsRefresh || ui != last_ui) {
        lcd_clear();
        last_ui = ui;
        screenNeedsRefresh = false;

        switch(ui){
            case UI_WELCOME:      show_welcome();     break;
            case UI_DASH_WATER:   show_dash_water();  break;
            case UI_DASH_MODE:    show_dash_mode();   break;
            case UI_DASH_SEARCH:  show_dash_search(); break;
            case UI_DASH_TWIST:   show_dash_twist();  break;
            case UI_MENU:         show_menu();        break;
            default: break; // TODO: Add edit/confirm UIs
        }
    }
}

void Screen_Init(void){
    lcd_init();
    ui = UI_WELCOME;
    last_ui = UI_MAX_;
    screenNeedsRefresh = true;
    lastLcdUpdateTime = HAL_GetTick();
}

void Screen_ResetToHome(void){
    ui = UI_WELCOME;
    last_ui = UI_MAX_;
    screenNeedsRefresh = true;
    lastLcdUpdateTime = HAL_GetTick();
}

/* ================= Input Handling ================= */
void Screen_HandleButton(UiButton b){
    if (b == BTN_RESET){ Screen_ResetToHome(); return; }

    switch (ui) {
    case UI_WELCOME:
        if (b == BTN_SELECT) ui = UI_DASH_WATER;
        break;

    case UI_DASH_WATER:
    case UI_DASH_MODE:
    case UI_DASH_SEARCH:
    case UI_DASH_TWIST:
        if (b == BTN_SELECT) ui = UI_MENU;
        break;

    case UI_MENU:
        if (b == BTN_UP)   menu_idx = (menu_idx==0)? MENU_COUNT-1 : menu_idx-1;
        if (b == BTN_DOWN) menu_idx = (menu_idx+1) % MENU_COUNT;
        if (b == BTN_SELECT) apply_menu_action();
        break;

    default: break;
    }

    screenNeedsRefresh = true;
    lastLcdUpdateTime = HAL_GetTick();
}

/* Generic switch handler */
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
            HAL_GPIO_TogglePin(GPIOA, switchMap[i].ledPin);
            Screen_HandleButton(switchMap[i].btn);
        } else if (!pressed) prev[i] = true;
    }
}
