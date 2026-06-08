/* ====================================================================
 * rf.c  —  RECEIVER  433 MHz OOK/ASK driver
 *
 * v6.2 — SYNC16 moved to file scope (was inside try_receive_frame)
 *
 *   The previous version defined SYNC16 with #define inside the body
 *   of try_receive_frame().  In C, a #define has no scope — it persists
 *   as a file-scope macro from that point forward and causes a
 *   "macro redefinition" warning with -Wall if the TU is processed more
 *   than once or if SYNC16 appears elsewhere.  Moved to file scope.
 *   No runtime change.
 *
 * Hardware: XY-MK-5V DATA → PB7 (GPIOB, GPIO_PIN_7)
 *   INPUT, no pull — per schematic SCH_Schematic1_2026-06-08.
 *   PB7 also wired to Ra-02 DIO0 via R4 (0Ω); mutually exclusive use.
 *
 * v6.1 — RF_Task() simplified (retained):
 *   Jumps directly into try_receive_frame() on pin activity rather than
 *   probing one bit first.  try_receive_frame() has its own bit-error
 *   recovery in the sync hunt loop.
 * ==================================================================== */

#include "rf.h"
#include "lora_parser.h"
#include "device_id.h"
#include "stm32f1xx_hal.h"
#include "main.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/* ── Board-specific ─────────────────────────────────────────────────── *
 * TIM3 clock derivation:                                                *
 *   SYSCLK = 64 MHz (HSI/2 → PLL×16)                                  *
 *   APB1   = 32 MHz (÷2); HAL doubles TIM clock when APB1 ≠ AHB       *
 *   TIM3 clock = 2 × 32 MHz = 64 MHz                                   *
 *   Prescaler 63 → 64 MHz / (63+1) = 1 MHz → 1 µs/count              */
#define RF_TIM3_PRESCALER   63u

/* ── Sync word — file scope (was inside try_receive_frame as #define) ─ *
 * Moved here to avoid compiler warning about macro defined in function  *
 * body persisting beyond function scope.                                */
#define RF_SYNC16   ((uint16_t)(((uint16_t)RF_SYNC_BYTE1 << 8u) | RF_SYNC_BYTE2))

/* ── External handles ───────────────────────────────────────────────── */
extern TIM_HandleTypeDef  htim3;
extern UART_HandleTypeDef huart1;

/* ── UART helper ────────────────────────────────────────────────────── */
static void uart_ln(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 500u);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2u, 500u);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  RF-ONLY WIRELESS DATA STORE
 * ═══════════════════════════════════════════════════════════════════════ */

static uint8_t  s_rfLevel      = 0u;
static uint8_t  s_rfWellDry    = 0u;
static uint32_t s_rfLastRxTick = 0u;
static bool     s_rfDataValid  = false;

/* ── Debug / visibility variables ───────────────────────────────────── */
static char     s_rfLastRawPacket[RF_MAX_PAYLOAD + 1u] = {0};
static uint32_t s_rfLastDid      = 0u;
static uint32_t s_rfLastSeq      = 0u;
static uint8_t  s_rfLastType     = 0u;
static uint8_t  s_rfLastCrcRx    = 0u;
static uint8_t  s_rfLastCrcCalc  = 0u;
static uint32_t s_rfLastPacketMs = 0u;
static bool     s_rfPacketSeen   = false;

/* ── Duplicate-sequence filter ──────────────────────────────────────── */
static uint32_t s_rfDupLastSeq = 0u;
static bool     s_rfDupSeqInit = false;

/* ── Counters ───────────────────────────────────────────────────────── */
static uint32_t s_rxPackets = 0u;
static uint32_t s_rxErrors  = 0u;

/* ── Hardware ready flag ────────────────────────────────────────────── */
static bool s_hwOk = false;

/* ═══════════════════════════════════════════════════════════════════════
 *  CRC-8  poly 0x07, init 0x00
 * ═══════════════════════════════════════════════════════════════════════ */
uint8_t RF_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00u;

    while (len--)
    {
        crc ^= *data++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80u) ? ((crc << 1u) ^ 0x07u) : (crc << 1u);
    }

    return crc;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  LOW-LEVEL PIN / TIMER HELPERS
 * ═══════════════════════════════════════════════════════════════════════ */

static inline uint8_t rf_pin(void)
{
    /* XY-MK-5V DATA is wired to RF_connector_Pin (PC13, GPIOC).       *
     * RF_DATA_Pin (PB7) is used by lora.c for DIO0 — not touched here.*/
    return (HAL_GPIO_ReadPin(RF_connector_GPIO_Port, RF_connector_Pin) == GPIO_PIN_SET)
           ? 1u : 0u;
}

static inline void tim_rst(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0u);
}

static inline uint32_t tim_us(void)
{
    return __HAL_TIM_GET_COUNTER(&htim3);
}

static bool wait_low(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (rf_pin() != 0u)
    {
        if ((HAL_GetTick() - t0) >= timeout_ms) return false;
    }
    return true;
}

static bool wait_high(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (rf_pin() != 1u)
    {
        if ((HAL_GetTick() - t0) >= timeout_ms) return false;
    }
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  READ ONE RF BIT
 *
 *  HIGH 100–550 µs = bit 1
 *  HIGH 600–1100 µs = bit 0
 *  Returns 1 / 0 on success, -1 on timeout or unrecognised width.
 * ═══════════════════════════════════════════════════════════════════════ */
static int8_t read_bit(void)
{
    if (!wait_high(3u))
        return -1;

    tim_rst();

    while (rf_pin() == 1u)
    {
        if (tim_us() > (RF_THRESH_ZERO_MAX_US + 200u))
            return -1;
    }

    uint32_t w = tim_us();

    if (w >= RF_THRESH_ONE_MIN_US  && w <= RF_THRESH_ONE_MAX_US)  return 1;
    if (w >= RF_THRESH_ZERO_MIN_US && w <= RF_THRESH_ZERO_MAX_US) return 0;
    return -1;
}

static bool read_byte(uint8_t *out)
{
    uint8_t b = 0u;
    for (int8_t i = 7; i >= 0; i--)
    {
        int8_t bit = read_bit();
        if (bit < 0) return false;
        b |= (uint8_t)((uint8_t)bit << (uint8_t)i);
    }
    *out = b;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  TRY RECEIVE ONE COMPLETE RF FRAME
 *
 *  Frame: preamble (5×0xAA) + sync (0x2D 0xD4) + len + payload + CRC-8
 *
 *  Sync hunt error recovery (v6.1):
 *    One bad read_bit() resets good_bits and retries once.
 *    Two consecutive bad bits → abandon attempt.
 * ═══════════════════════════════════════════════════════════════════════ */
static bool try_receive_frame(void)
{
    if (!wait_low(3u))
        return false;

    uint16_t sreg      = 0u;
    uint32_t bits      = 0u;
    uint32_t good_bits = 0u;
    bool     found     = false;

    /* ── Phase 1: preamble + sync hunt ─────────────────────────────── */
    while (bits < RF_SYNC_HUNT_MAX_BITS)
    {
        int8_t b = read_bit();

        if (b < 0)
        {
            good_bits = 0u;
            b = read_bit();
            if (b < 0) return false;
        }

        bits++;
        good_bits++;

        sreg = (uint16_t)((sreg << 1u) | (uint8_t)b);

        if (sreg == RF_SYNC16 && good_bits >= RF_MIN_VALID_PREAMBLE_BITS)
        {
            found = true;
            break;
        }
    }

    if (!found) return false;

    /* ── Phase 2: length byte ──────────────────────────────────────── */
    uint8_t len = 0u;
    if (!read_byte(&len)) return false;

    if (len == 0u || len > RF_MAX_PAYLOAD)
    {
        s_rxErrors++;
        uart_ln("[RF RX] invalid length byte");
        return false;
    }

    /* ── Phase 3: payload bytes ────────────────────────────────────── */
    char payload[RF_MAX_PAYLOAD + 1u];
    for (uint8_t i = 0u; i < len; i++)
    {
        if (!read_byte((uint8_t *)&payload[i])) return false;
    }
    payload[len] = '\0';

    /* ── Phase 4: CRC byte ─────────────────────────────────────────── */
    uint8_t rx_crc = 0u;
    if (!read_byte(&rx_crc)) return false;

    uint8_t calc_crc = RF_CRC8((const uint8_t *)payload, len);

    strncpy(s_rfLastRawPacket, payload, RF_MAX_PAYLOAD);
    s_rfLastRawPacket[RF_MAX_PAYLOAD] = '\0';
    s_rfLastCrcRx    = rx_crc;
    s_rfLastCrcCalc  = calc_crc;
    s_rfLastPacketMs = HAL_GetTick();
    s_rfPacketSeen   = true;

    if (rx_crc != calc_crc)
    {
        s_rxErrors++;
        char dbg[160];
        snprintf(dbg, sizeof(dbg),
                 "[RF RX] CRC FAIL  raw=\"%s\"  rx=0x%02X  calc=0x%02X",
                 s_rfLastRawPacket, (unsigned)rx_crc, (unsigned)calc_crc);
        uart_ln(dbg);
        return false;
    }

    /* ── Phase 5: parse payload ────────────────────────────────────── */
    ParsedPacket_t p;
    if (!LoRa_ParsePacket(payload, &p))
    {
        s_rxErrors++;
        char dbg[160];
        snprintf(dbg, sizeof(dbg), "[RF RX] PARSE FAIL  raw=\"%s\"",
                 s_rfLastRawPacket);
        uart_ln(dbg);
        return false;
    }

    s_rxPackets++;
    s_rfLastType = (uint8_t)p.type;
    s_rfLastDid  = p.did;
    s_rfLastSeq  = p.seq;

    {
        char dbg[200];
        snprintf(dbg, sizeof(dbg),
                 "[RF RX] OK  #%lu  type=%d  DID=%08lX  SEQ=%08lX  \"%s\"",
                 (unsigned long)s_rxPackets, (int)p.type,
                 (unsigned long)p.did, (unsigned long)p.seq,
                 s_rfLastRawPacket);
        uart_ln(dbg);
    }

    /* ── Phase 6: accept TANKLEVEL data ────────────────────────────── */
    if (p.type == PKT_TYPE_TANKLEVEL)
    {
        if (s_rfDupSeqInit && p.seq == s_rfDupLastSeq)
        {
            uart_ln("[RF RX] duplicate seq — discarded");
            return false;
        }

        s_rfDupLastSeq = p.seq;
        s_rfDupSeqInit = true;

        s_rfLevel      = p.level;
        s_rfWellDry    = p.well_dry;
        s_rfLastRxTick = HAL_GetTick();
        s_rfDataValid  = true;

        char dbg[200];
        snprintf(dbg, sizeof(dbg),
                 "[RF RX] ACCEPTED  Level=%u%%  WD=%u  Seq=%08lX  DID=%08lX",
                 (unsigned)s_rfLevel, (unsigned)s_rfWellDry,
                 (unsigned long)s_rfLastSeq, (unsigned long)s_rfLastDid);
        uart_ln(dbg);
        return true;
    }

    /* ── HELLO ─────────────────────────────────────────────────────── */
    if (p.type == PKT_TYPE_HELLO)
    {
        char dbg[160];
        snprintf(dbg, sizeof(dbg), "[RF RX] HELLO  DID=%08lX  raw=\"%s\"",
                 (unsigned long)s_rfLastDid, s_rfLastRawPacket);
        uart_ln(dbg);
        s_rfLastRxTick = HAL_GetTick();
        return true;
    }

    uart_ln("[RF RX] unknown packet type — ignored");
    return false;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  PUBLIC DATA ACCESSORS
 * ═══════════════════════════════════════════════════════════════════════ */

bool RF_IsWirelessDataValid(void)
{
    if (!s_rfDataValid) return false;

    if ((HAL_GetTick() - s_rfLastRxTick) > RF_WIRELESS_TIMEOUT_MS)
    {
        s_rfDataValid = false;
        uart_ln("[RF RX] wireless data timed out — falling back to local ADC");
        return false;
    }

    return true;
}

uint8_t  RF_GetWirelessTankLevel (void) { return s_rfLevel;        }
uint8_t  RF_GetWirelessWellDry   (void) { return s_rfWellDry;      }

bool          RF_HasReceivedPacket    (void) { return s_rfPacketSeen;    }
const char*   RF_GetLastRawPacket     (void) { return s_rfLastRawPacket; }
uint32_t      RF_GetLastPacketDID     (void) { return s_rfLastDid;       }
uint32_t      RF_GetLastPacketSeq     (void) { return s_rfLastSeq;       }
uint8_t       RF_GetLastPacketType    (void) { return s_rfLastType;      }
uint8_t       RF_GetLastPacketCrcRx   (void) { return s_rfLastCrcRx;     }
uint8_t       RF_GetLastPacketCrcCalc (void) { return s_rfLastCrcCalc;   }
uint32_t      RF_GetRxPacketCount     (void) { return s_rxPackets;       }
uint32_t      RF_GetRxErrorCount      (void) { return s_rxErrors;        }

uint32_t RF_GetLastPacketAgeMs(void)
{
    if (!s_rfPacketSeen) return 0xFFFFFFFFUL;
    return HAL_GetTick() - s_rfLastPacketMs;
}

void RF_ClearWirelessData(void)
{
    s_rfLevel      = 0u;
    s_rfWellDry    = 0u;
    s_rfDataValid  = false;
    s_rfLastRxTick = 0u;

    s_rfDupSeqInit = false;
    s_rfDupLastSeq = 0u;

    s_rfLastRawPacket[0] = '\0';
    s_rfLastDid          = 0u;
    s_rfLastSeq          = 0u;
    s_rfLastType         = 0u;
    s_rfLastCrcRx        = 0u;
    s_rfLastCrcCalc      = 0u;
    s_rfLastPacketMs     = 0u;
    s_rfPacketSeen       = false;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  RF_Init
 * ═══════════════════════════════════════════════════════════════════════ */
void RF_Init(void)
{
    uart_ln("[RF RX] Init...");
    uart_ln("[RF RX] DATA pin: PC13 (GPIOC GPIO_PIN_13 = RF_connector_Pin)");

    HAL_TIM_Base_Stop(&htim3);
    __HAL_TIM_SET_PRESCALER(&htim3, RF_TIM3_PRESCALER);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 0xFFFFu);
    htim3.Instance->EGR |= TIM_EGR_UG;   /* force prescaler reload */
    HAL_TIM_Base_Start(&htim3);

    RF_ClearWirelessData();

    s_rxPackets = 0u;
    s_rxErrors  = 0u;
    s_hwOk      = true;

    uart_ln("[RF RX] TIM3 → 1 MHz (prescaler=63, TIM3_CLK=64MHz)");
    uart_ln("[RF RX] Init complete — 433 MHz OOK RX ready");
    uart_ln("[RF RX] Waiting for @HI:<DID># and @TL:...# packets");
}

/* ═══════════════════════════════════════════════════════════════════════
 *  RF_Task  — call from main loop as frequently as possible
 * ═══════════════════════════════════════════════════════════════════════ */
void RF_Task(void)
{
    if (!s_hwOk) return;

    /* Data expiry — fires even if application stops calling the getter */
    if (s_rfDataValid &&
        (HAL_GetTick() - s_rfLastRxTick) > RF_WIRELESS_TIMEOUT_MS)
    {
        s_rfDataValid = false;
        uart_ln("[RF RX] RF data expired (task check)");
    }

    /* Activity gate: wait up to 2 ms for pin to go HIGH.
     * Prevents busy-looping on an idle (LOW) channel.
     * 2 ms << 1200 µs bit period so no preamble edge is missed.       */
    if (rf_pin() == 0u)
    {
        uint32_t t0 = HAL_GetTick();
        while (rf_pin() == 0u)
        {
            if ((HAL_GetTick() - t0) >= 2u) return;
        }
    }

    (void)try_receive_frame();
}
