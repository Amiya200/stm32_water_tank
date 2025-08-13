#include "led.h"
#include "main.h"

/* Mapping:
   - Status LED  -> LED1 (PA8)
   - Func LED    -> LED2 (PA11)
   You can change mapping here if you prefer other LEDs (LED3..LED5 available).
*/

static GPIO_TypeDef* STATUS_PORT = LED1_GPIO_Port;
static const uint16_t STATUS_PIN = LED1_Pin;

static GPIO_TypeDef* FUNC_PORT = LED2_GPIO_Port;
static const uint16_t FUNC_PIN = LED2_Pin;

void LED_Init(void)
{
    /* CubeMX usually sets up the pins. This re-inits safely if required. */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO clocks (safe to call even if already enabled) */
    if (STATUS_PORT == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    if (STATUS_PORT == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    if (FUNC_PORT == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    if (FUNC_PORT == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = STATUS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(STATUS_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = FUNC_PIN;
    HAL_GPIO_Init(FUNC_PORT, &GPIO_InitStruct);

    /* Initialize off */
    HAL_GPIO_WritePin(STATUS_PORT, STATUS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(FUNC_PORT, FUNC_PIN, GPIO_PIN_RESET);
}

void LED_Status_On(void)  { HAL_GPIO_WritePin(STATUS_PORT, STATUS_PIN, GPIO_PIN_SET); }
void LED_Status_Off(void) { HAL_GPIO_WritePin(STATUS_PORT, STATUS_PIN, GPIO_PIN_RESET); }
void LED_Status_Toggle(void) { HAL_GPIO_TogglePin(STATUS_PORT, STATUS_PIN); }

void LED_Func_On(void)  { HAL_GPIO_WritePin(FUNC_PORT, FUNC_PIN, GPIO_PIN_SET); }
void LED_Func_Off(void) { HAL_GPIO_WritePin(FUNC_PORT, FUNC_PIN, GPIO_PIN_RESET); }

void LED_Func_Blink(uint32_t times, uint32_t delay_ms)
{
    for (uint32_t i = 0; i < times; ++i) {
        LED_Func_On();
        HAL_Delay(delay_ms);
        LED_Func_Off();
        HAL_Delay(delay_ms);
    }
}
