#include "screen.h"
#include "lcd_i2c.h"
#include "switches.h"
#include "model_handle.h"
#include "adc.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* ===== UI buttons (mapped in main via Switch_WasPressed) ===== */
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
    UI_MENU,          // landing menu
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
static const uint32_t PAGE_MS = 4000; // 4 sec per page
static UiState last_ui = UI_MAX_;
static bool cursorVisible = true;
static uint32_t lastCursorToggle = 0;

/* ===== Externals from your codebase ===== */
extern ADC_Data adcData;             // populated in main loop
extern TimerSlot timerSlots[5];      // slot[0] used here
extern SearchSettings searchSettings;
extern TwistSettings  twistSettings;
extern volatile bool  countdownActive;
extern volatile uint32_t countdownDuration;
extern volatile bool  countdownMode; // true=ON countdown, false=OFF
extern bool Motor_GetStatus(void);

/* ===== Module locals ===== */
static UiState ui = UI_WELCOME;
static char buf[21];
static uint8_t temp_h=0, temp_m=0, temp_s=0;
static uint8_t menu_idx = 0;
static const char* menu_items[] = {
    "Manual: ON",
    "Manual: OFF",
    "Countdown (min)",
    "Timer1  ON",
    "Timer1 OFF",
    "Search Gap",
    "Search DryRun",
    "Twist ON dur",
    "Twist OFF dur",
    "Back to Dash"
};
#define MENU_COUNT (sizeof(menu_items)/sizeof(menu_items[0]))

/* ================= Helpers ================= */
static void lcd_line0(const char* s){ lcd_put_cur(0,0); lcd_send_string(s); }
static void lcd_line1(const char* s){ lcd_put_cur(1,0); lcd_send_string(s); }
static void lcd_line0_full(const char* s) {
    char ln[21];
    snprintf(ln, sizeof(ln), "%-20s", s); // pad with spaces
    lcd_put_cur(0,0);
    lcd_send_string(ln);
}
static void lcd_line1_full(const char* s) {
    char ln[21];
    snprintf(ln, sizeof(ln), "%-20s", s);
    lcd_put_cur(1,0);
    lcd_send_string(ln);
}
static void goto_dash_cycle(void) {
    if (ui < UI_DASH_WATER || ui > UI_DASH_TWIST) ui = UI_DASH_WATER;
}

static void show_welcome(void){
    lcd_clear();
    lcd_line0("Welcome to");
    lcd_line1("HELONIX");
}

static void show_dash_water(void){
    lcd_clear();
    // headline
//    snprintf(buf, sizeof(buf), "Water V0: %.2fV", adcData.voltages[0]);
    lcd_line0(buf);

    // simple status from your earlier logic
    if      (adcData.voltages[0] > 2.5f)  lcd_line1("Status: Full");
    else if (adcData.voltages[0] > 1.0f)  lcd_line1("Status: Half");
    else if (adcData.voltages[0] > 0.1f)  lcd_line1("Status: Low");
    else                                  lcd_line1("Status: Empty");
}

static void show_dash_mode(void){
    lcd_clear();
    snprintf(buf, sizeof(buf), "Motor:%s Cnt:%s",
             Motor_GetStatus() ? "ON":"OFF",
             countdownActive ? (countdownMode?"ON":"OFF") : "NA");
    lcd_line0(buf);

    snprintf(buf, sizeof(buf), "Menu: Press SEL");
    lcd_line1(buf);
}

static void show_dash_search(void){
    lcd_clear();
    lcd_line0("Search Mode");
    if (searchSettings.searchActive) {
        snprintf(buf, sizeof(buf), "Gap:%ds Dry:%ds",
                 (int)searchSettings.testingGapSeconds,
                 (int)searchSettings.dryRunTimeSeconds);
        lcd_line1(buf);
    } else {
        lcd_line1("Inactive");
    }
}

static void show_dash_twist(void){
    lcd_clear();
    lcd_line0("Twist Mode");
    if (twistSettings.twistActive) {
        snprintf(buf, sizeof(buf), "ON:%ds OFF:%ds",
                 (int)twistSettings.onDurationSeconds,
                 (int)twistSettings.offDurationSeconds);
        lcd_line1(buf);
    } else {
        lcd_line1("Inactive");
    }
}

static void show_menu(void){
    char cursor = cursorVisible ? '>' : ' ';   // blink effect
    snprintf(buf, sizeof(buf), "%c%s", cursor, menu_items[menu_idx]);
    buf[20]='\0';
    lcd_line0_full(buf);
    lcd_line1_full("UP/DN:Move  SEL:OK");
}


/* generic editor screens */
static void show_edit_mm(const char* title, uint8_t mm){
    lcd_clear();
    lcd_line0(title);
    snprintf(buf, sizeof(buf), "Value: %02u", mm);
    lcd_line1(buf);
}

static void show_edit_ms(const char* title, uint8_t mm, uint8_t ss){
    lcd_clear();
    lcd_line0(title);
    snprintf(buf, sizeof(buf), "%02u:%02u  UP/DN  SEL:OK", mm, ss);
    lcd_line1(buf);
}

static void show_edit_hhmm(const char* title, uint8_t hh, uint8_t mm){
    lcd_clear();
    lcd_line0(title);
    snprintf(buf, sizeof(buf), "%02u:%02u  UP/DN  SEL:Next", hh, mm);
    lcd_line1(buf);
}

static void apply_menu_action(void){
    switch(menu_idx){
        case 0: ui = UI_CONFIRM_MANUAL_ON;  break;
        case 1: ui = UI_CONFIRM_MANUAL_OFF; break;
        case 2: temp_m = 10; ui = UI_EDIT_COUNTDOWN_MIN; break;
        case 3: /* Timer1 ON hh:mm */
            temp_h = 6; temp_m = 0; ui = UI_EDIT_TIMER1_ON_H; break;
        case 4: /* Timer1 OFF hh:mm */
            temp_h = 7; temp_m = 0; ui = UI_EDIT_TIMER1_OFF_H; break;
        case 5: /* Search gap mm:ss */
            temp_m = 0; temp_s = 30; ui = UI_EDIT_SEARCH_GAP_M; break;
        case 6: /* Search dry run mm:ss */
            temp_m = 0; temp_s = 10; ui = UI_EDIT_SEARCH_DRY_M; break;
        case 7: /* Twist ON mm:ss */
            temp_m = 0; temp_s = 20; ui = UI_EDIT_TWIST_ON_M; break;
        case 8: /* Twist OFF mm:ss */
            temp_m = 0; temp_s = 40; ui = UI_EDIT_TWIST_OFF_M; break;
        case 9: goto_dash_cycle(); break;
    }
}

/* send commands into your existing model handlers */
static void send_cmd(const char* cmd){
    ModelHandle_ProcessUartCommand(cmd);
}

/* ================= Public API ================= */

void Screen_Init(void){
    lcd_init();
    lastLcdUpdateTime = HAL_GetTick();
    ui = UI_WELCOME;
}

void Screen_ResetToHome(void){
    ui = UI_WELCOME;
    lastLcdUpdateTime = HAL_GetTick();
}

void Screen_HandleButton(UiButton b){
    if (b == BTN_RESET){
        Screen_ResetToHome();
        return;
    }

    switch (ui)
    {
    case UI_WELCOME:
        if (b == BTN_SELECT) ui = UI_DASH_WATER;
        break;

    /* ===== Dashboard pages (auto-cycle) ===== */
    case UI_DASH_WATER:
    case UI_DASH_MODE:
    case UI_DASH_SEARCH:
    case UI_DASH_TWIST:
        if (b == BTN_SELECT) { ui = UI_MENU; }
        lastLcdUpdateTime = HAL_GetTick(); // reset cycle timer
        break;

    /* ===== Menu navigation ===== */
    case UI_MENU:
        if (b == BTN_UP)   { if (menu_idx==0) menu_idx=MENU_COUNT-1; else menu_idx--; }
        if (b == BTN_DOWN) { menu_idx=(menu_idx+1)%MENU_COUNT; }
        if (b == BTN_SELECT){ apply_menu_action(); }
        break;

    /* ===== Edits & confirms ===== */
    case UI_CONFIRM_MANUAL_ON:
        if (b == BTN_SELECT){ send_cmd("MOTOR_ON"); goto_dash_cycle(); }
        if (b == BTN_DOWN || b == BTN_UP){ ui = UI_MENU; }
        break;

    case UI_CONFIRM_MANUAL_OFF:
        if (b == BTN_SELECT){ send_cmd("MOTOR_OFF"); goto_dash_cycle(); }
        if (b == BTN_DOWN || b == BTN_UP){ ui = UI_MENU; }
        break;

    case UI_EDIT_COUNTDOWN_MIN:
        if (b == BTN_UP)   { if (++temp_m > 59) temp_m = 0; }
        if (b == BTN_DOWN) { if (temp_m==0) temp_m = 59; else temp_m--; }
        if (b == BTN_SELECT){
            char cmd[32];
            snprintf(cmd,sizeof(cmd),"COUNTDOWN_ON:%u", temp_m);
            send_cmd(cmd);
            goto_dash_cycle();
        }
        break;

    case UI_EDIT_TIMER1_ON_H:
        if (b == BTN_UP)   { if (++temp_h > 23) temp_h = 0; }
        if (b == BTN_DOWN) { if (temp_h==0) temp_h = 23; else temp_h--; }
        if (b == BTN_SELECT){ ui = UI_EDIT_TIMER1_ON_M; }
        break;
    case UI_EDIT_TIMER1_ON_M:
        if (b == BTN_UP)   { if (++temp_m > 59) temp_m = 0; }
        if (b == BTN_DOWN) { if (temp_m==0) temp_m = 59; else temp_m--; }
        if (b == BTN_SELECT){
            char onStr[6]; snprintf(onStr,sizeof(onStr),"%02u:%02u",temp_h,temp_m);
            // jump to OFF editor next
            ui = UI_EDIT_TIMER1_OFF_H;
            // stash in temp_h/temp_m again when we arrive there
            // keep onStr in mind via static? Simpler: store to timerSlots on final confirm
            // Here: save into slot as partial (seconds)
            timerSlots[0].onTimeSeconds = ModelHandle_TimeToSeconds(temp_h, temp_m);
            timerSlots[0].active = true;
        }
        break;

    case UI_EDIT_TIMER1_OFF_H:
        if (b == BTN_UP)   { if (++temp_h > 23) temp_h = 0; }
        if (b == BTN_DOWN) { if (temp_h==0) temp_h = 23; else temp_h--; }
        if (b == BTN_SELECT){ ui = UI_EDIT_TIMER1_OFF_M; }
        break;
    case UI_EDIT_TIMER1_OFF_M:
        if (b == BTN_UP)   { if (++temp_m > 59) temp_m = 0; }
        if (b == BTN_DOWN) { if (temp_m==0) temp_m = 59; else temp_m--; }
        if (b == BTN_SELECT){
            timerSlots[0].offTimeSeconds = ModelHandle_TimeToSeconds(temp_h, temp_m);
            timerSlots[0].executedToday = false;
            timerSlots[0].active = true;

            char cmd[48];
            // use the model’s parser-friendly command
            // TIMER_SET:<slot>:HH:MM:HH:MM
            uint8_t onH,onM,offH,offM;
            ModelHandle_SecondsToTime(timerSlots[0].onTimeSeconds,&onH,&onM);
            ModelHandle_SecondsToTime(timerSlots[0].offTimeSeconds,&offH,&offM);
            snprintf(cmd,sizeof(cmd),"TIMER_SET:1:%02u:%02u:%02u:%02u",onH,onM,offH,offM);
            send_cmd(cmd);
            goto_dash_cycle();
        }
        break;

    case UI_EDIT_SEARCH_GAP_M:
        if (b == BTN_UP)   { if (++temp_m > 59) temp_m = 0; }
        if (b == BTN_DOWN) { if (temp_m==0) temp_m = 59; else temp_m--; }
        if (b == BTN_SELECT){ ui = UI_EDIT_SEARCH_GAP_S; }
        break;
    case UI_EDIT_SEARCH_GAP_S:
        if (b == BTN_UP)   { if (++temp_s > 59) temp_s = 0; }
        if (b == BTN_DOWN) { if (temp_s==0) temp_s = 59; else temp_s--; }
        if (b == BTN_SELECT){
            char cmd[32];
            snprintf(cmd,sizeof(cmd),"SEARCH_GAP:%02u:%02u", temp_m, temp_s);
            send_cmd(cmd);
            goto_dash_cycle();
        }
        break;

    case UI_EDIT_SEARCH_DRY_M:
        if (b == BTN_UP)   { if (++temp_m > 59) temp_m = 0; }
        if (b == BTN_DOWN) { if (temp_m==0) temp_m = 59; else temp_m--; }
        if (b == BTN_SELECT){ ui = UI_EDIT_SEARCH_DRY_S; }
        break;
    case UI_EDIT_SEARCH_DRY_S:
        if (b == BTN_UP)   { if (++temp_s > 59) temp_s = 0; }
        if (b == BTN_DOWN) { if (temp_s==0) temp_s = 59; else temp_s--; }
        if (b == BTN_SELECT){
            char cmd[32];
            snprintf(cmd,sizeof(cmd),"SEARCH_DRYRUN:%02u:%02u", temp_m, temp_s);
            send_cmd(cmd);
            goto_dash_cycle();
        }
        break;

    case UI_EDIT_TWIST_ON_M:
        if (b == BTN_UP)   { if (++temp_m > 59) temp_m = 0; }
        if (b == BTN_DOWN) { if (temp_m==0) temp_m = 59; else temp_m--; }
        if (b == BTN_SELECT){ ui = UI_EDIT_TWIST_ON_S; }
        break;
    case UI_EDIT_TWIST_ON_S:
        if (b == BTN_UP)   { if (++temp_s > 59) temp_s = 0; }
        if (b == BTN_DOWN) { if (temp_s==0) temp_s = 59; else temp_s--; }
        if (b == BTN_SELECT){
            char cmd[32];
            snprintf(cmd,sizeof(cmd),"TWIST_ONDUR:%02u:%02u", temp_m, temp_s);
            send_cmd(cmd);
            goto_dash_cycle();
        }
        break;

    case UI_EDIT_TWIST_OFF_M:
        if (b == BTN_UP)   { if (++temp_m > 59) temp_m = 0; }
        if (b == BTN_DOWN) { if (temp_m==0) temp_m = 59; else temp_m--; }
        if (b == BTN_SELECT){ ui = UI_EDIT_TWIST_OFF_S; }
        break;
    case UI_EDIT_TWIST_OFF_S:
        if (b == BTN_UP)   { if (++temp_s > 59) temp_s = 0; }
        if (b == BTN_DOWN) { if (temp_s==0) temp_s = 59; else temp_s--; }
        if (b == BTN_SELECT){
            char cmd[32];
            snprintf(cmd,sizeof(cmd),"TWIST_OFFDUR:%02u:%02u", temp_m, temp_s);
            send_cmd(cmd);
            goto_dash_cycle();
        }
        break;

    default: break;
    }
}

void Screen_Update(void){
    uint32_t now = HAL_GetTick();

    /* auto-advance dashboard pages */
    if (ui == UI_WELCOME){
        if (now - lastLcdUpdateTime < WELCOME_MS){
            show_welcome();
            return;
        }
        ui = UI_DASH_WATER;
        lastLcdUpdateTime = now;
    }

    /* periodic page refresh / cycle */
    if (ui >= UI_DASH_WATER && ui <= UI_DASH_TWIST){
        if (now - lastLcdUpdateTime >= PAGE_MS){
            ui = (UiState)(ui + 1);
            if (ui > UI_DASH_TWIST) ui = UI_DASH_WATER;
            lastLcdUpdateTime = now;
        }
    }
    // Clear only when UI page changes
    if (ui != last_ui) {
        lcd_clear();
        last_ui = ui;
    }

    /* render current state */
    switch(ui){
        case UI_DASH_WATER:  show_dash_water();  break;
        case UI_DASH_MODE:   show_dash_mode();   break;
        case UI_DASH_SEARCH: show_dash_search(); break;
        case UI_DASH_TWIST:  show_dash_twist();  break;
        case UI_MENU:        show_menu();        break;

        case UI_CONFIRM_MANUAL_ON:
            lcd_clear(); lcd_line0("Manual -> ON");
            lcd_line1("SEL:OK  UP/DN:Back");
            break;
        case UI_CONFIRM_MANUAL_OFF:
            lcd_clear(); lcd_line0("Manual -> OFF");
            lcd_line1("SEL:OK  UP/DN:Back");
            break;

        case UI_EDIT_COUNTDOWN_MIN:
            show_edit_mm("Countdown (min)", temp_m);
            break;

        case UI_EDIT_TIMER1_ON_H:
            show_edit_hhmm("Timer1 ON  HH", temp_h, temp_m);
            break;
        case UI_EDIT_TIMER1_ON_M:
            show_edit_hhmm("Timer1 ON  MM", temp_h, temp_m);
            break;
        case UI_EDIT_TIMER1_OFF_H:
            show_edit_hhmm("Timer1 OFF HH", temp_h, temp_m);
            break;
        case UI_EDIT_TIMER1_OFF_M:
            show_edit_hhmm("Timer1 OFF MM", temp_h, temp_m);
            break;

        case UI_EDIT_SEARCH_GAP_M:
            show_edit_ms("Search Gap  MM", temp_m, temp_s);
            break;
        case UI_EDIT_SEARCH_GAP_S:
            show_edit_ms("Search Gap  SS", temp_m, temp_s);
            break;

        case UI_EDIT_SEARCH_DRY_M:
            show_edit_ms("Dry Run     MM", temp_m, temp_s);
            break;
        case UI_EDIT_SEARCH_DRY_S:
            show_edit_ms("Dry Run     SS", temp_m, temp_s);
            break;

        case UI_EDIT_TWIST_ON_M:
            show_edit_ms("Twist ON    MM", temp_m, temp_s);
            break;
        case UI_EDIT_TWIST_ON_S:
            show_edit_ms("Twist ON    SS", temp_m, temp_s);
            break;

        case UI_EDIT_TWIST_OFF_M:
            show_edit_ms("Twist OFF   MM", temp_m, temp_s);
            break;
        case UI_EDIT_TWIST_OFF_S:
            show_edit_ms("Twist OFF   SS", temp_m, temp_s);
            break;

        default: break;
    }
}

/* ===== Convenience mapping (from main) ===== */
void Screen_HandleSwitches(void){
    if (Switch_WasPressed(1)) Screen_HandleButton(BTN_RESET);
    if (Switch_WasPressed(2)) Screen_HandleButton(BTN_SELECT);
    if (Switch_WasPressed(3)) Screen_HandleButton(BTN_UP);
    if (Switch_WasPressed(4)) Screen_HandleButton(BTN_DOWN);
}
