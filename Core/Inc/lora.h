#ifndef __LORA_H__
#define __LORA_H__

#include "stm32f1xx_hal.h"

// === SPI handle (use SPI1 from CubeMX) ===
extern SPI_HandleTypeDef hspi1;

// === Pin definitions (Blue Pill wiring) ===
#define LORA_NSS_PORT      GPIOA
#define LORA_NSS_PIN       GPIO_PIN_4   // PA4 → NSS/CS

#define LORA_RESET_PORT    GPIOB
#define LORA_RESET_PIN     GPIO_PIN_0   // PB0 → RESET

#define LORA_DIO0_PORT     GPIOB
#define LORA_DIO0_PIN      GPIO_PIN_1   // PB1 → DIO0

// === API ===
void LoRa_Init(void);
void LoRa_Reset(void);

void LoRa_WriteReg(uint8_t addr, uint8_t data);
uint8_t LoRa_ReadReg(uint8_t addr);

void LoRa_WriteBuffer(uint8_t addr, uint8_t *buffer, uint8_t size);
void LoRa_ReadBuffer(uint8_t addr, uint8_t *buffer, uint8_t size);

void LoRa_SendPacket(uint8_t *buffer, uint8_t size);
uint8_t LoRa_ReceivePacket(uint8_t *buffer);

#endif
