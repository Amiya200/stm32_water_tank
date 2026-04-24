/* ====================================================================
 * lora.h  —  RECEIVER project copy  (motor-controller node)
 *
 * NODE TYPE: LORA_RECEIVER_NODE
 *
 * Each STM32CubeIDE project has its OWN copy of this file in its
 * Core/Inc/ folder.  The node type is fixed right here so no
 * project-level preprocessor symbol is needed.
 *
 * ── Protocol packet contract (both nodes must match exactly) ─────────
 *
 *   TX  → RX   hello  :  @HI#
 *   RX  → TX   hello  :  @OK#
 *   TX  → RX   data   :  @TL:<3d>,WD:<1d>,SN:<5d>#
 *                         e.g.  @TL:080,WD:0,SN:00003#
 *                           TL = tank level  0 / 20 / 40 / 60 / 80 / 100
 *                           WD = well-dry flag  0=OK  1=DRY ALARM
 *                           SN = 5-digit sequence number
 *   RX  → TX   ack    :  @ACK:<5d>#
 *                         e.g.  @ACK:00003#
 *
 * ── g_loraConnected truth table ─────────────────────────────────────
 *
 *   RX side : set 1 when a valid data packet arrives; cleared to 0
 *             when WIRELESS_TIMEOUT_MS passes with no new packet.
 *   TX side : set 1 when last data packet was ACKed; cleared to 0
 *             after CONN_FAIL_THRESHOLD consecutive un-ACKed packets.
 *
 * ── Offline fallback (RX side) ──────────────────────────────────────
 *
 *   When g_loraConnected == 0 / LoRa_IsWirelessDataValid() == false :
 *     • CH0–CH3 (tank level probes) ← local physical ADC
 *     • CH5     (dry-run sensor)    ← local physical ADC
 *   When g_loraConnected == 1 / LoRa_IsWirelessDataValid() == true  :
 *     • CH0–CH3 ← synthesised from wireless TL value
 *     • CH5     ← synthesised from wireless WD flag
 *     • CH4 (ground-water) is ALWAYS read from local ADC on RX side
 * ==================================================================== */

#ifndef LORA_H
#define LORA_H

/* ── Fix the node type for THIS project right here ──────────────────── */
/* Receiver (motor-controller) project : LORA_RECEIVER_NODE             */
/* Transmitter (water-tank)    project : LORA_TRANSMITTER_NODE          */
/* Only ONE must be defined per project.                                 */
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
 *  g_loraConnected  —  1 = link healthy  |  0 = link down
 *
 *  Safe to poll from any module (model_handle.c, screen.c, adc.c …).
 *  Do NOT write to it directly; it is managed by lora.c only.
 * ════════════════════════════════════════════════════════════════════ */
extern uint8_t g_loraConnected;    /* 0 = disconnected  |  1 = connected */

/* ── New-packet notification flag ───────────────────────────────────── */
/*
 * Set to true by LoRa_Task() each time a valid data packet is parsed.
 * Cleared by main.c after it triggers g_screenUpdatePending.
 * Allows the main loop to force an immediate LCD refresh without
 * waiting for the 400 ms dash blink timer to fire.
 */
extern bool g_loraNewPacketFlag;

/* ── TX result enum (used by transmitter; kept here so lora.c         ── */
/* ── compiles cleanly even when included from adc.c on RX side)       ── */
typedef enum
{
    LORA_TX_OK    = 0,
    LORA_TX_RETRY = 1,
    LORA_TX_FAIL  = 2,
} LoRa_TxResult;

/* ── TX statistics externs (defined in transmitter lora.c only)       ── */
/* ── Declared here so any file that includes lora.h won't get errors  ── */
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

/* ── Init (same function name on both nodes) ────────────────────────── */
void LoRa_Init(void);

/* ══════════════════════════════════════════════════════════════════════
 *  NODE-SPECIFIC API  —  selected by the #define at the top of this file
 * ════════════════════════════════════════════════════════════════════ */
#if defined(LORA_RECEIVER_NODE)

/*
 * LoRa_Task()
 *   Non-blocking.  Must be called every main-loop iteration (~10 ms).
 *   Polls DIO0, receives packet, sends ACK, updates wireless state.
 *
 * LoRa_GetWirelessTankLevel()
 *   Returns the last received tank level (0–100 %).
 *   Valid only while LoRa_IsWirelessDataValid() == true.
 *
 * LoRa_GetWirelessWellDry()
 *   Returns the last received well-dry flag from the transmitter.
 *     0 = source water OK  (motor may run)
 *     1 = well DRY ALARM   (motor should stop — dry-run protection)
 *   Valid only while LoRa_IsWirelessDataValid() == true.
 *
 * LoRa_IsWirelessDataValid()
 *   Returns true while valid data has arrived within
 *   WIRELESS_TIMEOUT_MS (60 000 ms).
 *   Clears g_loraConnected and starts RED-BLINK LED on timeout.
 */
void    LoRa_Task                (void);
uint8_t LoRa_GetWirelessTankLevel(void);
uint8_t LoRa_GetWirelessWellDry  (void);  /* NEW — WD field from TX  */
bool    LoRa_IsWirelessDataValid (void);

#elif defined(LORA_TRANSMITTER_NODE)

/* TX node: LoRa_Task returns OK/RETRY/FAIL, called every TX interval */
LoRa_TxResult LoRa_SendPacket(const uint8_t *buffer, uint8_t size);
LoRa_TxResult LoRa_Task      (void);

#else
#error "lora.h: set #define LORA_RECEIVER_NODE or LORA_TRANSMITTER_NODE at the top."
#endif

#endif /* LORA_H */
