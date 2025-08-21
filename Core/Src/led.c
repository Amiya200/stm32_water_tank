#include "led.h"
#include "main.h"

/* ===========================================================
   LED Mapping (configure in CubeMX):
   - LED1 -> PA8
   - LED2 -> PA11
   - LED3 -> <set in CubeMX>
   - LED4 -> <set in CubeMX>
   - LED5 -> <set in CubeMX>
   =========================================================== */

static GPIO_TypeDef* LED_PORTS[5] = {
    LED1_GPIO_Port,
    LED2_GPIO_Port,
    LED3_GPIO_Port,
    LED4_GPIO_Port,
    LED5_GPIO_Port
};

static const uint16_t LED_PINS[5] = {
    LED1_Pin,
    LED2_Pin,
    LED3_Pin,
    LED4_Pin,
    LED5_Pin
};

/* ===========================================================
   Helper Functions
   =========================================================== */
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO clocks for all LED ports */
    for (int i = 0; i < 5; i++) {
        if (LED_PORTS[i] == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
        if (LED_PORTS[i] == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
        if (LED_PORTS[i] == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    }

    /* Configure pins as Output */
    for (int i = 0; i < 5; i++) {
        GPIO_InitStruct.Pin = LED_PINS[i];
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(LED_PORTS[i], &GPIO_InitStruct);

        /* Turn off at start */
        HAL_GPIO_WritePin(LED_PORTS[i], LED_PINS[i], GPIO_PIN_RESET);
    }
}

void LED_On(uint8_t led) {
    if (led < 5) HAL_GPIO_WritePin(LED_PORTS[led], LED_PINS[led], GPIO_PIN_SET);
}

void LED_Off(uint8_t led) {
    if (led < 5) HAL_GPIO_WritePin(LED_PORTS[led], LED_PINS[led], GPIO_PIN_RESET);
}

void LED_Toggle(uint8_t led) {
    if (led < 5) HAL_GPIO_TogglePin(LED_PORTS[led], LED_PINS[led]);
}

void LED_All_Off(void) {
    for (int i = 0; i < 5; i++) HAL_GPIO_WritePin(LED_PORTS[i], LED_PINS[i], GPIO_PIN_RESET);
}

void LED_All_On(void) {
    for (int i = 0; i < 5; i++) HAL_GPIO_WritePin(LED_PORTS[i], LED_PINS[i], GPIO_PIN_SET);
}

/* ===========================================================
   LED Patterns
   =========================================================== */

/* Blink all LEDs together */
void LED_Blink_All(uint32_t times, uint32_t delay_ms)
{
    for (uint32_t t = 0; t < times; t++) {
        LED_All_On();
        HAL_Delay(delay_ms);
        LED_All_Off();
        HAL_Delay(delay_ms);
    }
}

/* Sequential running LEDs (LED1 -> LED2 -> ... -> LED5) */
void LED_Running(uint32_t delay_ms)
{
    for (uint8_t i = 0; i < 5; i++) {
        LED_On(i);
        HAL_Delay(delay_ms);
        LED_Off(i);
    }
}

/* Knight Rider pattern (forward + backward) */
void LED_KnightRider(uint32_t delay_ms, uint32_t cycles)
{
    for (uint32_t c = 0; c < cycles; c++) {
        for (int i = 0; i < 5; i++) {
            LED_All_Off();
            LED_On(i);
            HAL_Delay(delay_ms);
        }
        for (int i = 3; i > 0; i--) {
            LED_All_Off();
            LED_On(i);
            HAL_Delay(delay_ms);
        }
    }
}

/* Alternating (Odd vs Even LEDs) */
void LED_Alternate(uint32_t times, uint32_t delay_ms)
{
    for (uint32_t t = 0; t < times; t++) {
        /* Odd LEDs on */
        LED_All_Off();
        LED_On(0);
        LED_On(2);
        LED_On(4);
        HAL_Delay(delay_ms);

        /* Even LEDs on */
        LED_All_Off();
        LED_On(1);
        LED_On(3);
        HAL_Delay(delay_ms);
    }
}

/* Wave pattern (one by one stays ON, does not turn off until end) */
void LED_Wave(uint32_t delay_ms)
{
    LED_All_Off();
    for (uint8_t i = 0; i < 5; i++) {
        LED_On(i);
        HAL_Delay(delay_ms);
    }
    HAL_Delay(delay_ms);
    LED_All_Off();
}
