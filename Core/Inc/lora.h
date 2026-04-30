/* ====================================================================
 * lora.h  —  RECEIVER project header
 *
 * FINAL DID-BASED PROTOCOL
 *
 * DATA packet:
 *   @TL:060,WD:0,DID:A1B2C3D4#
 *
 * ACK packet:
 *   @ACK:A1B2C3D4#
 *
 * Pairing:
 *   - When receiver pairing mode is ON, it can pair from DATA packet.
 *   - No SN / serial number is required.
 *
 * Place in:
 *   Core/Inc/lora.h
 * ==================================================================== */

#ifndef __LORA_H
#define __LORA_H

#define LORA_RECEIVER_NODE

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "main.h"
#include "device_id.h"

#define LORA_MODE_RECEIVER      0u
#define LORA_MODE_TRANSMITTER   1u

#define TX_PACKET_SIZE          96u

/* GPIO mapping — verify with your CubeMX/schematic */
#define LORA_NSS_PORT           GPIOA
#define LORA_NSS_PIN            LORA_SELECT_Pin

#define LORA_RESET_PORT         GPIOB
#define LORA_RESET_PIN          LORA_STATUS_Pin

#define LORA_DIO0_PORT          RF_DATA_GPIO_Port
#define LORA_DIO0_PIN           RF_DATA_Pin

typedef enum {
    LORA_TX_OK    = 0,
    LORA_TX_RETRY = 1,
    LORA_TX_FAIL  = 2
} LoRa_TxResult;

/* Public globals */
extern uint8_t  loraMode;
extern uint8_t  g_loraConnected;
extern uint8_t  rxBuffer_l[96];
extern uint32_t rxPacketCount_l;
extern bool     g_loraNewPacketFlag;

extern uint32_t g_lora_tx_ok;
extern uint32_t g_lora_tx_retry;
extern uint32_t g_lora_tx_fail;

/* Public functions */
void    LoRa_Init(void);
void    LoRa_Task(void);

bool    LoRa_IsWirelessDataValid(void);
uint8_t LoRa_GetWirelessTankLevel(void);
uint8_t LoRa_GetWirelessWellDry(void);

void     LoRa_EnterPairingMode(void);
void     LoRa_ExitPairingMode(void);
bool     LoRa_IsPairingMode(void);
bool     LoRa_IsPairingComplete(void);
uint32_t LoRa_GetLastPairedDID(void);

void    LoRa_WriteReg(uint8_t addr, uint8_t data);
uint8_t LoRa_ReadReg(uint8_t addr);
void    LoRa_WriteBuffer(uint8_t addr, const uint8_t *buf, uint8_t size);
void    LoRa_ReadBuffer(uint8_t addr, uint8_t *buf, uint8_t size);

#endif /* __LORA_H */
