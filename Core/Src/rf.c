#include "rf.h"
#include "stm32f1xx_hal.h"
extern TIM_HandleTypeDef htim3;
static void rf_delay_us(uint32_t us) {
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    while (__HAL_TIM_GET_COUNTER(&htim3) < us);
}
void RF_Init(void) {
    HAL_GPIO_WritePin(RF_DATA_GPIO_Port, RF_DATA_Pin, GPIO_PIN_RESET);
}
static void send_high_low(uint32_t high_us, uint32_t low_us) {
    HAL_GPIO_WritePin(RF_DATA_GPIO_Port, RF_DATA_Pin, GPIO_PIN_SET);
    rf_delay_us(high_us);
    HAL_GPIO_WritePin(RF_DATA_GPIO_Port, RF_DATA_Pin, GPIO_PIN_RESET);
    rf_delay_us(low_us);
}
static void send_bit(uint8_t bit) {
    if (bit) {
        send_high_low(300, 900);
    } else {
        send_high_low(900, 300);
    }
}
void RF_SendCode(uint32_t code, uint8_t bits) {
    for (int repeat = 0; repeat < 4; repeat++) {
        send_high_low(275, 9900);
        rf_delay_us(1000);
        for (int8_t i = bits - 1; i >= 0; i--) {
            send_bit((code >> i) & 1);
        }
        send_high_low(300, 900);
        rf_delay_us(10000);
    }
}
