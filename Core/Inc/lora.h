#ifndef __LORA_H__
#define __LORA_H__

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* Pin mapping (keep consistent with your main.h wiring) */
#define LORA_NSS_PORT    GPIOA
#define LORA_NSS_PIN     GPIO_PIN_15

#define LORA_RESET_PORT  GPIOB
#define LORA_RESET_PIN   GPIO_PIN_6

#define LORA_DIO0_PORT   GPIOB
#define LORA_DIO0_PIN    GPIO_PIN_7

/* External SPI handle */
extern SPI_HandleTypeDef hspi1;

/* API */
void LoRa_Reset(void);
void LoRa_Init(void);
void LoRa_SetFrequency(uint32_t freqHz);

void LoRa_WriteReg(uint8_t addr, uint8_t data);
uint8_t LoRa_ReadReg(uint8_t addr);

void LoRa_WriteBuffer(uint8_t addr, const uint8_t *buffer, uint8_t size);
void LoRa_ReadBuffer(uint8_t addr, uint8_t *buffer, uint8_t size);

void LoRa_SendPacket(const uint8_t *buffer, uint8_t size);
uint8_t LoRa_ReceivePacket(uint8_t *buffer);

#endif /* __LORA_H__ */
