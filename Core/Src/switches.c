#include "switches.h"
#include "main.h"

#define DEBOUNCE_MS 50

/* track last state and last change time for simple debounce */
static GPIO_PinState last_state[5];
static uint32_t last_change[5];

void Switches_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; /* pressed -> ground */
    GPIO_InitStruct.Pin = SWITCH1_Pin|SWITCH2_Pin|SWITCH3_Pin|SWITCH4_Pin;
    HAL_GPIO_Init(SWITCH1_GPIO_Port, &GPIO_InitStruct);

    /* initialize state */
    for (int i = 1; i <= 4; ++i) {
        last_state[i] = GPIO_PIN_SET;
        last_change[i] = HAL_GetTick();
    }
}

/* read raw: pressed -> true */
bool Switch_Read(uint8_t idx)
{
    GPIO_PinState st = GPIO_PIN_SET;
    switch (idx) {
        case 1: st = HAL_GPIO_ReadPin(SWITCH1_GPIO_Port, SWITCH1_Pin); break;
        case 2: st = HAL_GPIO_ReadPin(SWITCH2_GPIO_Port, SWITCH2_Pin); break;
        case 3: st = HAL_GPIO_ReadPin(SWITCH3_GPIO_Port, SWITCH3_Pin); break;
        case 4: st = HAL_GPIO_ReadPin(SWITCH4_GPIO_Port, SWITCH4_Pin); break;
        default: return false;
    }
    return (st == GPIO_PIN_RESET); /* active low */
}

/* returns true only once per press (debounced) */
bool Switch_WasPressed(uint8_t idx)
{
    GPIO_PinState st = GPIO_PIN_SET;
    uint32_t now = HAL_GetTick();
    switch (idx) {
        case 1: st = HAL_GPIO_ReadPin(SWITCH1_GPIO_Port, SWITCH1_Pin); break;
        case 2: st = HAL_GPIO_ReadPin(SWITCH2_GPIO_Port, SWITCH2_Pin); break;
        case 3: st = HAL_GPIO_ReadPin(SWITCH3_GPIO_Port, SWITCH3_Pin); break;
        case 4: st = HAL_GPIO_ReadPin(SWITCH4_GPIO_Port, SWITCH4_Pin); break;
        default: return false;
    }

    if (st != last_state[idx]) {
        last_change[idx] = now;
        last_state[idx] = st;
    }

    if ((now - last_change[idx]) > DEBOUNCE_MS) {
        /* stable state; detect falling edge (released -> pressed) */
        static bool reported[5] = {false, false, false, false, false};
        if (st == GPIO_PIN_RESET) {
            if (!reported[idx]) {
                reported[idx] = true;
                return true;
            }
        } else {
            reported[idx] = false;
        }
    }
    return false;
}
