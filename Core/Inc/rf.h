/* ====================================================================
 * rf.h  —  433 MHz OOK/ASK RF layer
 *
 * Shared between TRANSMITTER and RECEIVER firmware.
 *
 * Hardware (RX board — schematic verified):
 *   XY-MK-5V DATA → PB7 (RF_DATA_Pin, GPIOB GPIO_PIN_7)
 *   GPIO config   : INPUT, no pull (module drives line actively)
 *   No AFIO remap : PB7 is a standard GPIO, no remap required.
 *
 * Hardware (TX board — transmitter schematic pending):
 *   FS1000A DATA  ← PB7 (or as per TX schematic)
 *   GPIO config   : OUTPUT_PP, HIGH speed
 *
 * PB7 is also connected to Ra-02 DIO0 via R4 (0Ω) on the PCB.
 * Safe because LoRa and RF433 are mutually exclusive (g_wireless_mode).
 *
 * Wire format:
 *   [Preamble: RF_PREAMBLE_BYTES × 0xAA]
 *   [Sync1: 0x2D] [Sync2: 0xD4]
 *   [Length: 1 byte]
 *   [Payload: 1..RF_MAX_PAYLOAD bytes]
 *   [CRC-8: poly=0x07, init=0x00, over payload only]
 *
 * Bit encoding (1200 µs / bit, ≈ 833 bps):
 *   bit '1' → HIGH 300 µs + LOW  900 µs
 *   bit '0' → HIGH 900 µs + LOW  300 µs
 *
 * Timer: RF_Init() reconfigures TIM3 → 1 MHz (prescaler = 63 at 64 MHz).
 *        TIM3 is on APB1; HAL doubles APB1 when APB1≠AHB.
 *        64 MHz AHB, APB1 = 32 MHz → TIM3 clk = 64 MHz.
 *        Prescaler 63 → 64 MHz / 64 = 1 MHz → 1 µs/count. ✓
 * ==================================================================== */

#ifndef RF_H
#define RF_H
#include <stdint.h>
#include <stdbool.h>

/* ── Wire-format constants (must be identical on TX and RX) ─────────── */
#define RF_PREAMBLE_BYTES           5u
#define RF_SYNC_BYTE1               0x2Du
#define RF_SYNC_BYTE2               0xD4u
#define RF_MAX_PAYLOAD              48u
#define RF_TX_REPEATS               3u
#define RF_INTER_PKT_GAP_US         20000u

/* ── Bit timing (µs) ────────────────────────────────────────────────── */
#define RF_BIT_ONE_HIGH_US          300u
#define RF_BIT_ONE_LOW_US           900u
#define RF_BIT_ZERO_HIGH_US         900u
#define RF_BIT_ZERO_LOW_US          300u
#define RF_BIT_PERIOD_US            1200u

/* ── RX decode thresholds ───────────────────────────────────────────── */
#define RF_THRESH_ONE_MIN_US        100u
#define RF_THRESH_ONE_MAX_US        550u
#define RF_THRESH_ZERO_MIN_US       600u
#define RF_THRESH_ZERO_MAX_US       1100u

/* ── TX cadence ─────────────────────────────────────────────────────── */
#define RF_HELLO_INTERVAL_MS        5000u
#define RF_DATA_INTERVAL_MS         12000u

/* ── RX data validity window ────────────────────────────────────────── */
#define RF_WIRELESS_TIMEOUT_MS      90000u

/* ── RX robustness ──────────────────────────────────────────────────── */
#define RF_MIN_VALID_PREAMBLE_BITS  16u
#define RF_SYNC_HUNT_MAX_BITS       120u

void    RF_Init(void);
void    RF_Task(void);

bool    RF_IsWirelessDataValid(void);
uint8_t RF_GetWirelessTankLevel(void);
uint8_t RF_GetWirelessWellDry(void);
void    RF_ClearWirelessData(void);

bool          RF_HasReceivedPacket(void);
const char*   RF_GetLastRawPacket(void);
uint32_t      RF_GetLastPacketDID(void);
uint32_t      RF_GetLastPacketSeq(void);
uint8_t       RF_GetLastPacketType(void);
uint32_t      RF_GetLastPacketAgeMs(void);
uint8_t       RF_GetLastPacketCrcRx(void);
uint8_t       RF_GetLastPacketCrcCalc(void);
uint32_t      RF_GetRxPacketCount(void);
uint32_t      RF_GetRxErrorCount(void);

#endif /* RF_H */
