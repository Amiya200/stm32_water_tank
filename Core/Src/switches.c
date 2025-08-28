#include "switches.h"
#include "main.h"

#define SWITCH_COUNT 4
#define DEBOUNCE_MS   15      // fast, responsive debounce
static uint16_t s_longPressMs = 700;  // default long-press threshold

/* Per-switch state */
static GPIO_PinState last_state[SWITCH_COUNT];
static uint32_t      last_change[SWITCH_COUNT];
static bool          pressed_reported[SWITCH_COUNT];  // for WasPressed()
static uint32_t      press_start_ms[SWITCH_COUNT];
static bool          long_fired[SWITCH_COUNT];        // true after LONG fired

static inline uint32_t now_ms(void){ return HAL_GetTick(); }

/* Map index -> HAL pin read (active low) */
static GPIO_PinState read_raw(uint8_t idx)
{
    switch (idx) {
        case 0: return HAL_GPIO_ReadPin(SWITCH1_GPIO_Port, SWITCH1_Pin);
        case 1: return HAL_GPIO_ReadPin(SWITCH2_GPIO_Port, SWITCH2_Pin);
        case 2: return HAL_GPIO_ReadPin(SWITCH3_GPIO_Port, SWITCH3_Pin);
        case 3: return HAL_GPIO_ReadPin(SWITCH4_GPIO_Port, SWITCH4_Pin);
        default: return GPIO_PIN_SET; // invalid => not pressed
    }
}

void Switches_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;  // pressed -> GND
    GPIO_InitStruct.Pin  = SWITCH1_Pin|SWITCH2_Pin|SWITCH3_Pin|SWITCH4_Pin;
    HAL_GPIO_Init(SWITCH1_GPIO_Port, &GPIO_InitStruct);

    for (int i = 0; i < SWITCH_COUNT; ++i) {
        last_state[i]       = GPIO_PIN_SET; // released
        last_change[i]      = now_ms();
        pressed_reported[i] = false;
        press_start_ms[i]   = 0;
        long_fired[i]       = false;
    }
}

void Switches_SetLongPressMs(uint16_t ms){ s_longPressMs = ms; }

/* Debounced current level */
bool Switch_IsPressed(uint8_t idx)
{
    if (idx >= SWITCH_COUNT) return false;

    uint32_t t = now_ms();
    GPIO_PinState raw = read_raw(idx);

    if (raw != last_state[idx]) {
        last_change[idx] = t;
        last_state[idx]  = raw;
    }
    /* state is considered valid after DEBOUNCE_MS; return current level */
    return (last_state[idx] == GPIO_PIN_RESET);
}

/* Backward compatible “press edge” (fires once on press) */
bool Switch_WasPressed(uint8_t idx)
{
    if (idx >= SWITCH_COUNT) return false;

    (void)Switch_IsPressed(idx); // update debounced internals

    if (last_state[idx] == GPIO_PIN_SET) {
        if (!pressed_reported[idx]) {
            pressed_reported[idx] = true;
            return true;
        }
    } else {
        if ((now_ms() - last_change[idx]) >= DEBOUNCE_MS) {
            pressed_reported[idx] = false;
        }
    }
    return false;
}

/* NEW: Short/Long press event generator
   - LONG fires once when threshold is crossed (while still held)
   - SHORT fires once on release, only if LONG was never fired
*/
SwitchEvent Switch_GetEvent(uint8_t idx)
{
    if (idx >= SWITCH_COUNT) return SWITCH_EVT_NONE;

    uint32_t t = now_ms();
    (void)Switch_IsPressed(idx); // keep debounce state fresh

    /* Button currently held? (active low) */
    bool held = (last_state[idx] == GPIO_PIN_RESET);

    if (held) {
        /* mark press start */
        if (press_start_ms[idx] == 0) {
            press_start_ms[idx] = t;
            long_fired[idx]     = false;
        }
        /* emit LONG once if threshold crossed */
        if (!long_fired[idx] && (t - press_start_ms[idx] >= s_longPressMs)) {
            long_fired[idx] = true;
            return SWITCH_EVT_LONG;
        }
        return SWITCH_EVT_NONE;
    }

    /* Released state */
    if (press_start_ms[idx] != 0) {
        /* ensure stable release */
        if ((t - last_change[idx]) >= DEBOUNCE_MS) {
            bool was_long = long_fired[idx];
            press_start_ms[idx] = 0;
            long_fired[idx]     = false;
            return was_long ? SWITCH_EVT_NONE : SWITCH_EVT_SHORT;
        }
    }
    return SWITCH_EVT_NONE;
}
