/* ====================================================================
 * lora.h  —  RECEIVER public interface (state-machine version)
 *
 * THIS IS THE RECEIVER HEADER for the WT1.1 (motor controller) project.
 * The transmitter project (water_level) uses a different lora.h with
 * #ifndef LORA_TX_H.
 * ==================================================================== */

#ifndef LORA_RX_H
#define LORA_RX_H

#include "stm32f1xx_hal.h"
#include "main.h"     /* LORA_SELECT_Pin, LORA_STATUS_Pin, RF_DATA_Pin */
#include "lora_protocol.h"

/* ── Pin definitions (must match RX board wiring) ───────────────────── */
#define LORA_NSS_PORT    LORA_SELECT_GPIO_Port  /* GPIOA */
#define LORA_NSS_PIN     LORA_SELECT_Pin          /* GPIO_PIN_15 */
#define LORA_RESET_PORT  LORA_STATUS_GPIO_Port   /* GPIOB */
#define LORA_RESET_PIN   LORA_STATUS_Pin          /* GPIO_PIN_6 */
#define LORA_DIO0_PORT   RF_DATA_GPIO_Port        /* GPIOB */
#define LORA_DIO0_PIN    RF_DATA_Pin              /* GPIO_PIN_7 */

#define LORA_MODE_TRANSMITTER  0
#define LORA_MODE_RECEIVER     1

/* ── Debug variables for STM32CubeIDE Live Expressions ─────────────── *
 *
 * Add these to the Live Expressions view to monitor radio activity:
 *
 *   g_lastRxPacket   — last packet received over-the-air
 *   g_lastTxPacket   — last reply sent (ACK / PONG / REJ / SY)
 *   g_loraConnected  — 0 = WAITING, 1 = CONNECTED
 *
 * Example sequence after first boot with no paired devices:
 *
 *   g_lastRxPacket = "@HI:A1B2C3D4#"                ← TX handshakes
 *   g_lastTxPacket = "@ACK:A1B2C3D4#"               ← RX auto-pairs + ACK
 *   g_loraConnected = 1
 *
 *   g_lastRxPacket = "@TL:075,WD:0,SQ:00000003,DID:A1B2C3D4#"
 *   g_lastTxPacket = "@ACK:A1B2C3D4#"               ← data accepted
 * ──────────────────────────────────────────────────────────────────── */
extern volatile char g_lastRxPacket[64];
extern volatile char g_lastTxPacket[64];

/* ── Public globals ─────────────────────────────────────────────────── */
extern uint8_t  loraMode;
extern uint8_t  g_loraConnected;          /* 1 = CONNECTED, 0 = other  */
extern bool     g_loraNewPacketFlag;      /* set when fresh @TL accepted */

/* Cumulative counters (also visible in Live Expressions)              */
extern uint32_t g_lora_tx_ok;
extern uint32_t g_lora_tx_retry;
extern uint32_t g_lora_tx_fail;

/* ── Core API ────────────────────────────────────────────────────────── */
void              LoRa_Init           (void);
void              LoRa_Task           (void);   /* call every loop tick */

LoRa_ConnState_t  LoRa_GetState       (void);
const char *      LoRa_GetStateString (void);

/* ── Wireless data accessors (used by adc.c, main.c) ────────────────── */
uint8_t           LoRa_GetWirelessTankLevel(void);  /* 0..100 %         */
uint8_t           LoRa_GetWirelessWellDry  (void);  /* 0=ok, 1=dry      */
uint32_t          LoRa_GetLastSequence     (void);
uint32_t          LoRa_GetPacketsLost      (void);
bool              LoRa_IsWirelessDataValid (void);

/* ── Pairing API (used by screen.c menu) ─────────────────────────────── *
 *
 * Auto-pair is ALWAYS active when no devices are stored (open mode).
 * Manual pairing mode replaces the stored DID with the next incoming
 * one — useful for re-pairing after a transmitter is replaced.
 * ──────────────────────────────────────────────────────────────────── */
void     LoRa_EnterPairingMode (void);
void     LoRa_ExitPairingMode  (void);
bool     LoRa_IsPairingMode    (void);
bool     LoRa_IsPairingComplete(void);
uint32_t LoRa_GetLastPairedDID (void);

/* ── Low-level register access (kept for any module needing raw I/O) ──── */
void    LoRa_WriteReg(uint8_t addr, uint8_t data);
uint8_t LoRa_ReadReg (uint8_t addr);

#endif /* LORA_RX_H */
