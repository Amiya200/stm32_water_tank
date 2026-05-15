/* ====================================================================
 * rf.h  —  433 MHz OOK/ASK RF layer
 *
 * Shared between the TRANSMITTER firmware and the RECEIVER firmware.
 * Copy this file into both projects unchanged.
 *
 * Hardware
 *   TX board : FS1000A (or equivalent) 433 MHz ASK/OOK transmitter
 *              Data pin → RF_DATA_Pin  (must be OUTPUT_PP, HIGH-speed)
 *   RX board : XY-MK-5V / MX-RM-5V superheterodyne 433 MHz receiver
 *              Data pin ← RF_DATA_Pin  (GPIO_MODE_INPUT, no pull)
 *
 * Wire format (byte-level, MSB first within each byte):
 *
 *   [Preamble: RF_PREAMBLE_BYTES × 0xAA]
 *   [Sync1: 0x2D] [Sync2: 0xD4]
 *   [Length: 1 byte — count of payload bytes]
 *   [Payload: 1..RF_MAX_PAYLOAD bytes — same ASCII format as LoRa]
 *   [CRC-8: 1 byte — poly 0x07, init 0x00 over payload only]
 *
 * Bit encoding (Manchester-style OOK, 1 200 µs / bit ≈ 833 bps):
 *   bit '1' → HIGH 300 µs  + LOW  900 µs
 *   bit '0' → HIGH 900 µs  + LOW  300 µs
 *
 * Payload format is identical to LoRa — LoRa_ParsePacket() / builders
 * from lora_parser.c are reused on both sides.
 *
 * Timer dependency:
 *   Both rf.c files reconfigure TIM3 to a 1 MHz clock (1 µs / count)
 *   inside RF_Init().  TIM3 is used exclusively for µs-level timing;
 *   it does NOT conflict with HAL_Delay() (SysTick-based).
 *
 * GPIO change required in the TRANSMITTER MX_GPIO_Init():
 *   Find the block that currently sets RF_DATA_Pin as INPUT and change:
 *       GPIO_InitStruct.Pin   = RF_DATA_Pin;
 *       GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
 *       GPIO_InitStruct.Pull  = GPIO_NOPULL;
 *       GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
 *       HAL_GPIO_Init(RF_DATA_GPIO_Port, &GPIO_InitStruct);
 *   (The RECEIVER board already has INPUT — no change needed there.)
 *
 * Integrating RF data into the RECEIVER adc.c (optional):
 *   In ADC_ReadAllChannels(), after the LoRa override block, add:
 *
 *       if (LoRa_IsWirelessDataValid()) {
 *           inject_wireless_level(...LoRa data...);
 *       } else if (RF_IsWirelessDataValid()) {
 *           inject_wireless_level(data,
 *               RF_GetWirelessTankLevel(),
 *               RF_GetWirelessWellDry());
 *       }
 *
 *   This gives LoRa priority; RF acts as an automatic fallback.
 * ==================================================================== */

#ifndef RF_H
#define RF_H

#include <stdint.h>
#include <stdbool.h>

/* ── Wire-format constants (must be identical on TX and RX) ─────────── */
#define RF_PREAMBLE_BYTES           5u      /* 5 × 0xAA = 40 preamble bits  */
#define RF_SYNC_BYTE1               0x2Du
#define RF_SYNC_BYTE2               0xD4u
#define RF_MAX_PAYLOAD              48u     /* maximum payload bytes        */
#define RF_TX_REPEATS               3u      /* transmit each frame N times  */
#define RF_INTER_PKT_GAP_US         20000u  /* 20 ms quiet between repeats  */

/* ── Bit timing (µs) ────────────────────────────────────────────────── */
#define RF_BIT_ONE_HIGH_US          300u
#define RF_BIT_ONE_LOW_US           900u
#define RF_BIT_ZERO_HIGH_US         900u
#define RF_BIT_ZERO_LOW_US          300u
#define RF_BIT_PERIOD_US            1200u

/* ── RX decode thresholds: HIGH pulse duration ──────────────────────── */
/* Keep tolerances generous (~±33 %) to handle timing jitter.           */
#define RF_THRESH_ONE_MIN_US        100u
#define RF_THRESH_ONE_MAX_US        550u
#define RF_THRESH_ZERO_MIN_US       600u
#define RF_THRESH_ZERO_MAX_US       1100u

/* ── Cadence (TX) ───────────────────────────────────────────────────── */
#define RF_HELLO_INTERVAL_MS        5000u   /* HELLO every 5 s              */
#define RF_DATA_INTERVAL_MS         12000u  /* DATA every 12 s              */

/* ── Data validity window (RX) ──────────────────────────────────────── */
#define RF_WIRELESS_TIMEOUT_MS      90000u  /* data expires after 90 s      */

/* ── RX robustness ──────────────────────────────────────────────────── */
/* Minimum consecutive valid preamble bits before accepting a sync word. */
#define RF_MIN_VALID_PREAMBLE_BITS  16u
/* Maximum bits to scan while hunting for sync before giving up.         */
#define RF_SYNC_HUNT_MAX_BITS       120u

/* ─────────────────────────────────────────────────────────────────────
 *  Shared API
 * ──────────────────────────────────────────────────────────────────── */
void    RF_Init (void);
uint8_t RF_CRC8 (const uint8_t *data, uint8_t len);

/* ─────────────────────────────────────────────────────────────────────
 *  TX-only API  (compiled into the TRANSMITTER firmware)
 * ──────────────────────────────────────────────────────────────────── */
void RF_SendPacket (const char *payload, uint8_t len);
void RF_Service    (uint8_t tank_level, uint8_t well_dry);

/* ─────────────────────────────────────────────────────────────────────
 *  RX-only API  (compiled into the RECEIVER firmware)
 * ──────────────────────────────────────────────────────────────────── */
void    RF_Task                (void);
bool    RF_IsWirelessDataValid (void);
uint8_t RF_GetWirelessTankLevel(void);
uint8_t RF_GetWirelessWellDry  (void);
void    RF_ClearWirelessData   (void);

#endif /* RF_H */
