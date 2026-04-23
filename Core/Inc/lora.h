/* ====================================================================
 * lora.h  —  Shared LoRa header (both transmitter and receiver nodes)
 * ==================================================================== */

#ifndef LORA_H
#define LORA_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

/* ── Mode selection ─────────────────────────────────────────────────── */
#define LORA_MODE_TRANSMITTER   0
#define LORA_MODE_RECEIVER      1

/* ── GPIO pin definitions — adjust to match your board ─────────────── */
#define LORA_NSS_PORT    GPIOA
#define LORA_NSS_PIN     GPIO_PIN_15

#define LORA_RESET_PORT  GPIOB
#define LORA_RESET_PIN   GPIO_PIN_6

#define LORA_DIO0_PORT   GPIOB
#define LORA_DIO0_PIN    GPIO_PIN_7

/* ── Global mode variable (defined in lora.c of each node) ─────────── */
extern uint8_t loraMode;

/* ── Core LoRa API (present on both nodes) ──────────────────────────── */
void    LoRa_Init(void);
void    LoRa_Task(void);
void    LoRa_WriteReg(uint8_t addr, uint8_t data);
uint8_t LoRa_ReadReg(uint8_t addr);
void    LoRa_WriteBuffer(uint8_t addr, const uint8_t *buffer, uint8_t size);

/* Transmitter only */
void    LoRa_SendPacket(const uint8_t *buffer, uint8_t size);

/* Receiver only */
void    LoRa_ReadBuffer(uint8_t addr, uint8_t *buffer, uint8_t size);
uint8_t LoRa_ReceivePacket(uint8_t *buffer, int16_t *rssi);

/* ── Wireless tank level API (receiver node only) ───────────────────
 *
 * These functions expose the parsed tank level that LoRa_Task()
 * extracts from incoming @TL:<percent># packets.
 *
 * LoRa_IsWirelessDataValid() returns false after WIRELESS_TIMEOUT_MS
 * (60 s) without a packet; ADC_ReadAllChannels() falls back to
 * local ADC probes in that case.
 * ──────────────────────────────────────────────────────────────────── */
uint8_t LoRa_GetWirelessTankLevel(void);
bool    LoRa_IsWirelessDataValid(void);

/* External RX counters — available for debug screens */
extern uint8_t  rxBuffer_l[64];
extern uint32_t rxPacketCount_l;

#endif /* LORA_H */
