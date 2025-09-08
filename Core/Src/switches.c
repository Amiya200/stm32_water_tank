#include "switches.h"
#include "main.h"

#define SWITCH_COUNT 4
#define DEBOUNCE_MS  20       // debounce time (ms)
static uint16_t s_longPressMs = 700;  // default long-press threshold

/* Per-switch state */
static GPIO_PinState stable_state[SWITCH_COUNT];   // debounced state
static uint32_t      last_change[SWITCH_COUNT];    // last raw change
static GPIO_PinState last_raw[SWITCH_COUNT];       // last sampled raw pin

static uint32_t      press_start_ms[SWITCH_COUNT]; // for long press
static bool          long_fired[SWITCH_COUNT];

static inline uint32_t now_ms(void) { return HAL_GetTick(); }

/* --- Internal: map switch index -> GPIO --- */
static GPIO_PinState read_raw(uint8_t idx)
{
    switch (idx) {
        case 0: return HAL_GPIO_ReadPin(SWITCH1_GPIO_Port, SWITCH1_Pin);
        case 1: return HAL_GPIO_ReadPin(SWITCH2_GPIO_Port, SWITCH2_Pin);
        case 2: return HAL_GPIO_ReadPin(SWITCH3_GPIO_Port, SWITCH3_Pin);
        case 3: return HAL_GPIO_ReadPin(SWITCH4_GPIO_Port, SWITCH4_Pin);
        default: return GPIO_PIN_SET; // not pressed
    }
}

/* --- Init all switches as input pullup --- */
void Switches_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Pin  = SWITCH1_Pin|SWITCH2_Pin|SWITCH3_Pin|SWITCH4_Pin;
    HAL_GPIO_Init(SWITCH1_GPIO_Port, &GPIO_InitStruct);

    for (int i = 0; i < SWITCH_COUNT; ++i) {
        stable_state[i]   = GPIO_PIN_SET;  // released
        last_raw[i]       = GPIO_PIN_SET;
        last_change[i]    = now_ms();
        press_start_ms[i] = 0;
        long_fired[i]     = false;
    }
}

void Switches_SetLongPressMs(uint16_t ms) { s_longPressMs = ms; }

/* --- Update debounce state machine --- */
static void update_state(uint8_t idx)
{
    GPIO_PinState raw = read_raw(idx);
    uint32_t t = now_ms();

    if (raw != last_raw[idx]) {
        last_raw[idx] = raw;
        last_change[idx] = t;   // mark change time
    }

    if ((t - last_change[idx]) >= DEBOUNCE_MS) {
        stable_state[idx] = raw; // accept new stable state
    }
}

/* --- Current debounced level (true = pressed) --- */
bool Switch_IsPressed(uint8_t idx)
{
    if (idx >= SWITCH_COUNT) return false;
    update_state(idx);
    return (stable_state[idx] == GPIO_PIN_RESET);
}

/* --- One-shot on press edge --- */
bool Switch_WasPressed(uint8_t idx)
{
    static bool prev_state[SWITCH_COUNT] = {0};
    if (idx >= SWITCH_COUNT) return false;

    bool pressed = Switch_IsPressed(idx);
    bool fired = (!prev_state[idx] && pressed);  // edge detect
    prev_state[idx] = pressed;
    return fired;
}

/* --- Short/Long event generator --- */
SwitchEvent Switch_GetEvent(uint8_t idx)
{
    if (idx >= SWITCH_COUNT) return SWITCH_EVT_NONE;

    update_state(idx);
    uint32_t t = now_ms();
    bool held = (stable_state[idx] == GPIO_PIN_RESET);

    if (held) {
        if (press_start_ms[idx] == 0) {
            press_start_ms[idx] = t;
            long_fired[idx]     = false;
        }
        if (!long_fired[idx] && (t - press_start_ms[idx] >= s_longPressMs)) {
            long_fired[idx] = true;
            return SWITCH_EVT_LONG;
        }
        return SWITCH_EVT_NONE;
    } else {
        if (press_start_ms[idx] != 0) {
            if ((t - last_change[idx]) >= DEBOUNCE_MS) {
                bool was_long = long_fired[idx];
                press_start_ms[idx] = 0;
                long_fired[idx]     = false;
                return was_long ? SWITCH_EVT_NONE : SWITCH_EVT_SHORT;
            }
        }
    }
    return SWITCH_EVT_NONE;
}
