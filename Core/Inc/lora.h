/* ====================================================================
 * lora.h  —  RECEIVER project copy  (motor-controller node)
 *
 * Each STM32CubeIDE project has its OWN copy of this file in its
 * Core/Inc/ folder.  The node type is fixed right here so no
 * project-level preprocessor symbol is needed.
 *
 * Protocol packet contract (both nodes must match):
 *
 *   TX  →  RX   hello :  @HI#
 *   RX  →  TX   hello :  @OK#
 *   TX  →  RX   data  :  @TL:<3d>,SN:<5d>#   e.g. @TL:080,SN:00003#
 *   RX  →  TX   ack   :  @ACK:<5d>#           e.g. @ACK:00003#
 *
 * g_loraConnected is 1 on BOTH nodes when the link is healthy.
 * ==================================================================== */

#ifndef LORA_H
#define LORA_H

/* ── Fix the node type for THIS project right here ──────────────────── */
/* Receiver (motor-controller) project: LORA_RECEIVER_NODE              */
/* Transmitter (water-tank)    project: LORA_TRANSMITTER_NODE           */
/* Only one must be defined per project.                                 */
#define LORA_RECEIVER_NODE

#include <stdint.h>
#include <stdbool.h>

/* ── Mode selection ─────────────────────────────────────────────────── */
#define LORA_MODE_TRANSMITTER  0
#define LORA_MODE_RECEIVER     1

extern uint8_t loraMode;

/* ── Hardware pin mapping ───────────────────────────────────────────── */
/* Adjust these to match your actual board wiring.                       */
#define LORA_NSS_PORT    GPIOA
#define LORA_NSS_PIN     GPIO_PIN_15

#define LORA_RESET_PORT  GPIOB
#define LORA_RESET_PIN   GPIO_PIN_6

#define LORA_DIO0_PORT   GPIOB
#define LORA_DIO0_PIN    GPIO_PIN_7

/* ── TX packet buffer size ──────────────────────────────────────────── */
#define TX_PACKET_SIZE   64

/* ══════════════════════════════════════════════════════════════════════
 *  SHARED CONNECTION STATE
 *
 *  g_loraConnected is 1 on BOTH nodes when the link is healthy.
 *    RX side → a valid data packet arrived within WIRELESS_TIMEOUT_MS
 *    TX side → last data packet was ACKed by the receiver
 *
 *  Your application code (main.c, adc.c, screen.c …) only needs to
 *  read this single flag to know whether the wireless link is live.
 * ════════════════════════════════════════════════════════════════════ */
extern uint8_t g_loraConnected;    /* 0 = disconnected  |  1 = connected */

/* ── TX result enum (used by transmitter; kept here so lora.c         ── */
/* ── compiles cleanly even when included from adc.c on RX side)       ── */
typedef enum
{
    LORA_TX_OK    = 0,
    LORA_TX_RETRY = 1,
    LORA_TX_FAIL  = 2,
} LoRa_TxResult;

/* ── TX statistics externs (defined in transmitter lora.c only)       ── */
/* ── Declared here so any file that includes lora.h won't get errors) ── */
extern uint32_t g_lora_tx_ok;
extern uint32_t g_lora_tx_retry;
extern uint32_t g_lora_tx_fail;

/* ── RX shared data ─────────────────────────────────────────────────── */
extern uint8_t  rxBuffer_l[64];
extern uint32_t rxPacketCount_l;

/* ── SPI register access (public — used by both nodes) ─────────────── */
void    LoRa_WriteReg   (uint8_t addr, uint8_t data);
uint8_t LoRa_ReadReg    (uint8_t addr);
void    LoRa_WriteBuffer(uint8_t addr, const uint8_t *buffer, uint8_t size);
void    LoRa_ReadBuffer (uint8_t addr,       uint8_t *buffer, uint8_t size);
/* LoRa_Reset() is static inside lora.c — not part of the public API.   */

/* ── Init (same name on both nodes) ────────────────────────────────── */
void LoRa_Init(void);

/* ══════════════════════════════════════════════════════════════════════
 *  NODE-SPECIFIC API  —  selected by the #define at the top of this file
 * ════════════════════════════════════════════════════════════════════ */
#if defined(LORA_RECEIVER_NODE)

/* RX node: LoRa_Task is void, called every ~10 ms in main loop */
void    LoRa_Task                (void);
uint8_t LoRa_GetWirelessTankLevel(void);   /* returns 0–100 % */
bool    LoRa_IsWirelessDataValid (void);   /* false when > 60 s silent */

#elif defined(LORA_TRANSMITTER_NODE)

/* TX node: LoRa_Task returns OK/RETRY/FAIL, called every TX interval */
LoRa_TxResult LoRa_SendPacket(const uint8_t *buffer, uint8_t size);
LoRa_TxResult LoRa_Task      (void);

#else
#error "lora.h: set #define LORA_RECEIVER_NODE or LORA_TRANSMITTER_NODE at the top of this file."
#endif

#endif /* LORA_H */
