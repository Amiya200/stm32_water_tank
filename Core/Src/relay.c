#include "relay.h"
#include "main.h"

void Relay_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = Relay1_Pin | Relay2_Pin | Relay3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Relay1_GPIO_Port, &GPIO_InitStruct);

    /* Ensure off on start (change if your relay is active-low) */
    HAL_GPIO_WritePin(Relay1_GPIO_Port, Relay1_Pin|Relay2_Pin|Relay3_Pin, GPIO_PIN_RESET);
}

void Relay_Set(uint8_t relay_no, bool on)
{
    switch (relay_no) {
        case 1: HAL_GPIO_WritePin(Relay1_GPIO_Port, Relay1_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET); break;
        case 2: HAL_GPIO_WritePin(Relay2_GPIO_Port, Relay2_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET); break;
        case 3: HAL_GPIO_WritePin(Relay3_GPIO_Port, Relay3_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET); break;
        default: break;
    }
}

bool Relay_Get(uint8_t relay_no)
{
    GPIO_PinState s = GPIO_PIN_RESET;
    switch (relay_no) {
        case 1: s = HAL_GPIO_ReadPin(Relay1_GPIO_Port, Relay1_Pin); break;
        case 2: s = HAL_GPIO_ReadPin(Relay2_GPIO_Port, Relay2_Pin); break;
        case 3: s = HAL_GPIO_ReadPin(Relay3_GPIO_Port, Relay3_Pin); break;
        default: return false;
    }
    return (s == GPIO_PIN_SET);
}

void Relay_All(bool on)
{
    Relay_Set(1, on);
    Relay_Set(2, on);
    Relay_Set(3, on);
}
