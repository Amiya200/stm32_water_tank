/* ====================================================================
 * rf.c  —  RECEIVER  433 MHz OOK/ASK driver
 *
 * Compile this file into the RECEIVER project (LORA_RECEIVER_NODE).
 *
 * Hardware: XY-MK-5V / MX-RM-5V (or equivalent) superheterodyne
 *           433 MHz receiver module.
 *   VCC  → 5 V  (superheterodyne modules need 5 V for best sensitivity)
 *   GND  → GND
 *   DATA → RF_DATA_Pin  ← GPIO_MODE_INPUT, no pull (already correct)
 *
 * How to call from main.c (RX):
 *
 *   // In startup, after TIM3 and GPIO inits:
 *   RF_Init();
 *
 *   // Inside the main while(1) loop — call it every iteration:
 *   RF_Task();
 *
 *   // Integrate into adc.c as LoRa fallback (see rf.h for snippet).
 *
 * ── Decode strategy ─────────────────────────────────────────────────
 *
 *   RF_Task() is called from the main loop every ~10 ms.
 *
 *   Non-blocking path (typical, returns in <5 ms):
 *     Check if pin has gone HIGH.  If not — return immediately.
 *     If yes, wait for the line to settle LOW (end of the current HIGH
 *     phase, up to 2 ms), then attempt one "probe" bit measurement.
 *     If the probe pulse is not in valid range — return (noise).
 *
 *   Blocking path (valid preamble detected, blocks up to ~600 ms):
 *     Call try_receive_frame() which reads all preamble bits, hunts
 *     for the sync word, then reads length + payload + CRC.
 *     Validates CRC, parses with LoRa_ParsePacket(), stores result.
 *
 *   A 600 ms block is acceptable because:
 *     • The TX sends 3 repetitions — if one is missed, 2 remain.
 *     • LoRa remains the primary channel; RF data is checked only when
 *       LoRa_IsWirelessDataValid() is false.
 *     • Motor FSM operates on a seconds timescale.
 *
 * ── Noise immunity ──────────────────────────────────────────────────
 *
 *   Cheap superheterodyne receivers over-amplify noise when no carrier
 *   is present, producing random HIGH/LOW output.  Two guards handle
 *   this:
 *
 *   1. Probe measurement: the very first bit pulse is measured before
 *      committing to a full frame decode.  Random noise produces pulse
 *      widths outside the 100–1 100 µs valid window — these are
 *      discarded with a cheap sub-5 ms cost.
 *
 *   2. Preamble gating: RF_MIN_VALID_PREAMBLE_BITS (16) consecutive
 *      valid bits must be received before a sync match is accepted.
 *      Random noise is unlikely to produce 16 consecutive valid pulses.
 *
 * ── Timer notes ─────────────────────────────────────────────────────
 *
 *   RF_Init() reconfigures TIM3 to 1 MHz (1 µs / count).
 *   Prescaler 63 is correct for a 64 MHz TIM3 clock; adjust
 *   RF_TIM3_PRESCALER if your board frequency differs.
 * ==================================================================== */

#include "rf.h"
#include "lora_parser.h"
#include "device_id.h"
#include "stm32f1xx_hal.h"
#include "main.h"          // ← add this
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/* ── Board-specific ─────────────────────────────────────────────────── */
#define RF_TIM3_PRESCALER   63u     /* 64 MHz / 64 = 1 MHz               */

extern TIM_HandleTypeDef  htim3;
extern UART_HandleTypeDef huart1;

/* ── UART helper ────────────────────────────────────────────────────── */
static void uart_ln(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s,   (uint16_t)strlen(s), 500u);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2u,               500u);
}

/* ── Wireless data store ────────────────────────────────────────────── */
static uint8_t  s_level      = 0u;
static uint8_t  s_wellDry    = 0u;
static uint32_t s_lastRxTick = 0u;
static bool     s_dataValid  = false;

static uint32_t s_lastSeq    = 0u;
static bool     s_seqInit    = false;

static uint32_t s_rxPackets  = 0u;   /* accepted good packets            */
static uint32_t s_rxErrors   = 0u;   /* CRC or length errors             */

static bool     s_hwOk       = false;

/* ── CRC-8  (poly 0x07, init 0x00) ─────────────────────────────────── */
uint8_t RF_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00u;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80u) ? ((crc << 1u) ^ 0x07u) : (crc << 1u);
    }
    return crc;
}

static inline uint8_t rf_pin(void)
{
    if (HAL_GPIO_ReadPin(RF_DATA_GPIO_Port, RF_DATA_Pin) == GPIO_PIN_SET)
        return 1u;
    return 0u;
}

static inline void   tim_rst (void) { __HAL_TIM_SET_COUNTER(&htim3, 0u); }
static inline uint32_t tim_us(void) { return __HAL_TIM_GET_COUNTER(&htim3); }

/* ── wait_low/wait_high — spin with ms-granularity timeout ─────────── *
 * Uses HAL_GetTick() so SysTick remains unblocked.                     *
 * ──────────────────────────────────────────────────────────────────── */
static bool wait_low (uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (rf_pin() != 0u)
        if ((HAL_GetTick() - t0) >= timeout_ms) return false;
    return true;
}

static bool wait_high(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (rf_pin() != 1u)
        if ((HAL_GetTick() - t0) >= timeout_ms) return false;
    return true;
}

/* ── read_bit ───────────────────────────────────────────────────────── *
 *                                                                       *
 * Precondition : pin is currently LOW.                                  *
 * Postcondition: pin is LOW again (end of this bit's LOW phase).       *
 *                                                                       *
 * Algorithm:                                                            *
 *   1. Wait for rising edge (timeout 3 ms > 2 × bit_period).           *
 *   2. Measure HIGH pulse width using TIM3 (µs precision).             *
 *   3. Classify: 100–550 µs → '1',  600–1 100 µs → '0',  else error.  *
 *                                                                       *
 * Returns 0 or 1 on success, -1 on timeout or invalid pulse width.    *
 * ──────────────────────────────────────────────────────────────────── */
static int8_t read_bit(void)
{
    /* 1. Rising edge */
    if (!wait_high(3u)) return -1;

    /* 2. Measure HIGH pulse; bail out if it exceeds maximum valid width */
    tim_rst();
    while (rf_pin() == 1u) {
        if (tim_us() > (RF_THRESH_ZERO_MAX_US + 200u)) return -1;
    }
    uint32_t w = tim_us();

    /* 3. Classify */
    if (w >= RF_THRESH_ONE_MIN_US  && w <= RF_THRESH_ONE_MAX_US)  return 1;
    if (w >= RF_THRESH_ZERO_MIN_US && w <= RF_THRESH_ZERO_MAX_US) return 0;
    return -1;
}

/* ── read_byte ──────────────────────────────────────────────────────── *
 * Reads 8 bits MSB-first via read_bit().                               *
 * Precondition/Postcondition: same as read_bit() — pin is LOW.        *
 * ──────────────────────────────────────────────────────────────────── */
static bool read_byte(uint8_t *out)
{
    uint8_t b = 0u;
    for (int8_t i = 7; i >= 0; i--) {
        int8_t bit = read_bit();
        if (bit < 0) return false;
        b |= (uint8_t)((uint8_t)bit << (uint8_t)i);
    }
    *out = b;
    return true;
}

/* ── try_receive_frame ──────────────────────────────────────────────── *
 *                                                                       *
 * Attempts to decode one complete RF frame.  This function is          *
 * BLOCKING for up to ~600 ms while a packet is in flight.             *
 *                                                                       *
 * Phase 1 — Sync hunt                                                  *
 *   Shift bits into a 16-bit register.  When the register equals the  *
 *   sync word (0x2DD4) AND at least RF_MIN_VALID_PREAMBLE_BITS valid  *
 *   bits have been received, declare sync found.                       *
 *                                                                       *
 * Phase 2 — Data                                                        *
 *   Read length byte → payload bytes → CRC byte.                      *
 *   Validate CRC.  On failure: increment error counter and return.     *
 *                                                                       *
 * Phase 3 — Parse & store                                              *
 *   Call LoRa_ParsePacket() on the payload.  Accept only              *
 *   PKT_TYPE_TANKLEVEL; duplicate sequence numbers are discarded.      *
 *                                                                       *
 * Returns true if a fresh tank-level packet was accepted.             *
 * ──────────────────────────────────────────────────────────────────── */
static bool try_receive_frame(void)
{
    /* First synchronise to a LOW state so read_bit() is well-defined.  */
    if (!wait_low(3u)) return false;   /* pin stuck HIGH → noise, abort  */

    /* ── Phase 1: preamble + sync hunt ─────────────────────────────── */
#define SYNC16  ((uint16_t)((RF_SYNC_BYTE1 << 8u) | RF_SYNC_BYTE2))   /* 0x2DD4 */

    uint16_t sreg       = 0u;
    uint32_t bits       = 0u;   /* total bits shifted in                 */
    uint32_t good_bits  = 0u;   /* consecutive valid bits (reset on err) */
    bool     sync_found = false;

    while (bits < RF_SYNC_HUNT_MAX_BITS) {
        int8_t b = read_bit();
        if (b < 0) {
            /* Lost signal mid-preamble.  Reset good-bit streak but     */
            /* keep trying — short noise glitches happen.               */
            good_bits = 0u;
            /* After two consecutive failures give up entirely.         */
            b = read_bit();
            if (b < 0) return false;
        }

        bits++;
        good_bits++;
        sreg = (uint16_t)((sreg << 1u) | (uint8_t)b);

        if (sreg == SYNC16 && good_bits >= RF_MIN_VALID_PREAMBLE_BITS) {
            sync_found = true;
            break;
        }
    }

    if (!sync_found) return false;

    /* ── Phase 2: length ────────────────────────────────────────────── */
    uint8_t len = 0u;
    if (!read_byte(&len))                 return false;
    if (len == 0u || len > RF_MAX_PAYLOAD) return false;

    /* ── Phase 2: payload ───────────────────────────────────────────── */
    char payload[RF_MAX_PAYLOAD + 1u];
    for (uint8_t i = 0u; i < len; i++) {
        if (!read_byte((uint8_t *)&payload[i])) return false;
    }
    payload[len] = '\0';

    /* ── Phase 2: CRC ───────────────────────────────────────────────── */
    uint8_t rx_crc = 0u;
    if (!read_byte(&rx_crc)) return false;

    uint8_t calc_crc = RF_CRC8((const uint8_t *)payload, len);
    if (rx_crc != calc_crc) {
        s_rxErrors++;
        char dbg[88];
        snprintf(dbg, sizeof(dbg),
                 "[RF RX] CRC error  rx=0x%02X  calc=0x%02X  raw=\"%s\"",
                 (unsigned)rx_crc, (unsigned)calc_crc, payload);
        uart_ln(dbg);
        return false;
    }

    /* ── Phase 3: parse ─────────────────────────────────────────────── */
    ParsedPacket_t p;
    if (!LoRa_ParsePacket(payload, &p)) {
        s_rxErrors++;
        uart_ln("[RF RX] parse failed (CRC ok)");
        return false;
    }

    s_rxPackets++;

    {
        char dbg[100];
        snprintf(dbg, sizeof(dbg),
                 "[RF RX] rx#%lu  type=%d  DID=%08lX  \"%s\"",
                 (unsigned long)s_rxPackets,
                 (int)p.type,
                 (unsigned long)p.did,
                 payload);
        uart_ln(dbg);
    }

    /* ── Phase 3: accept tank-level data ──────────────────────────── */
    if (p.type == PKT_TYPE_TANKLEVEL) {

        /* Duplicate-sequence filter */
        if (s_seqInit && p.seq == s_lastSeq) {
            uart_ln("[RF RX] duplicate seq — discarded");
            return false;
        }
        s_lastSeq = p.seq;
        s_seqInit = true;

        s_level      = p.level;
        s_wellDry    = p.well_dry;
        s_lastRxTick = HAL_GetTick();
        s_dataValid  = true;

        char dbg[80];
        snprintf(dbg, sizeof(dbg),
                 "[RF RX] DATA accepted: Level=%u%%  WD=%u  Seq=%08lX",
                 (unsigned)p.level, (unsigned)p.well_dry,
                 (unsigned long)p.seq);
        uart_ln(dbg);
        return true;
    }

    if (p.type == PKT_TYPE_HELLO) {
        char dbg[60];
        snprintf(dbg, sizeof(dbg),
                 "[RF RX] HELLO from DID=%08lX (simplex — no ACK)",
                 (unsigned long)p.did);
        uart_ln(dbg);
    }

    return false;   /* HELLO or unknown type — no data stored            */
}

/* ── Public data accessors ──────────────────────────────────────────── */

bool RF_IsWirelessDataValid(void)
{
    if (!s_dataValid) return false;

    if ((HAL_GetTick() - s_lastRxTick) > RF_WIRELESS_TIMEOUT_MS) {
        s_dataValid = false;
        uart_ln("[RF RX] wireless data TIMED OUT — falling back to local ADC");
        return false;
    }
    return true;
}

uint8_t RF_GetWirelessTankLevel(void) { return s_level;   }
uint8_t RF_GetWirelessWellDry  (void) { return s_wellDry; }

void RF_ClearWirelessData(void)
{
    s_level      = 0u;
    s_wellDry    = 0u;
    s_dataValid  = false;
    s_lastRxTick = 0u;
    s_seqInit    = false;
    s_lastSeq    = 0u;
}

/* ── RF_Init ────────────────────────────────────────────────────────── */
void RF_Init(void)
{
    uart_ln("[RF RX] Init...");

    HAL_TIM_Base_Stop(&htim3);
    __HAL_TIM_SET_PRESCALER(&htim3, RF_TIM3_PRESCALER);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 0xFFFFu);
    htim3.Instance->EGR |= TIM_EGR_UG;     /* force prescaler update     */
    HAL_TIM_Base_Start(&htim3);

    RF_ClearWirelessData();
    s_rxPackets = 0u;
    s_rxErrors  = 0u;
    s_hwOk      = true;

    uart_ln("[RF RX] TIM3 reconfigured to 1 MHz (1 µs/count)");
    uart_ln("[RF RX] Init complete — 433 MHz OOK RX ready");
}

/* ── RF_Task  (call from main loop every iteration) ─────────────────── *
 *                                                                       *
 * Execution paths:                                                      *
 *                                                                       *
 *  A) No signal — returns in <6 ms                                     *
 *     • Pin is LOW and stays LOW for the 5 ms observation window.      *
 *     • The module is idle, nothing to decode.                          *
 *                                                                       *
 *  B) Noise burst — returns in <10 ms                                  *
 *     • Pin briefly goes HIGH then LOW again.                          *
 *     • wait_low(2 ms) succeeds; read_bit() returns a pulse outside    *
 *       the valid window; try_receive_frame() exits early.             *
 *                                                                       *
 *  C) Real preamble — blocks up to ~600 ms                             *
 *     • Pin shows repeated valid-width pulses.                         *
 *     • try_receive_frame() succeeds and stores data.                  *
 *     • Returns after the last bit of the first repetition is read;    *
 *       remaining repetitions from the TX are ignored (dup-seq filter  *
 *       in try_receive_frame will discard them on subsequent RF_Task   *
 *       calls anyway).                                                  *
 * ──────────────────────────────────────────────────────────────────── */
void RF_Task(void)
{
    if (!s_hwOk) return;

    /* ── Step 1: quick idle check ──────────────────────────────────── *
     * If pin is LOW and stays LOW for 5 ms → no transmitter active.   */
    if (rf_pin() == 0u) {
        uint32_t t0 = HAL_GetTick();
        while (rf_pin() == 0u) {
            if ((HAL_GetTick() - t0) >= 5u) {
                /* Periodic data-expiry maintenance while idle           */
                if (s_dataValid &&
                    (HAL_GetTick() - s_lastRxTick) > RF_WIRELESS_TIMEOUT_MS) {
                    s_dataValid = false;
                    uart_ln("[RF RX] data expired (idle check)");
                }
                return;
            }
        }
        /* Pin just went HIGH — fall through to decode attempt          */
    }

    /* ── Step 2: pin is HIGH — wait for it to go LOW ───────────────── *
     * We need to be in a clean LOW state before read_bit().            *
     * If pin stays HIGH >2 ms (abnormal), treat as noise and bail.    */
    if (!wait_low(2u)) return;

    /* ── Step 3: validate one probe bit from this LOW state ─────────── *
     * If the very next bit pulse is invalid → noise, return cheaply.  */
    int8_t probe = read_bit();
    if (probe < 0) return;

    /* ── Step 4: valid pulse seen — attempt full frame decode ────────── *
     * The probe bit was consumed.  try_receive_frame() starts its own  *
     * sync hunt from a fresh LOW state; the remaining 39+ preamble     *
     * bits are more than enough to find the sync word.                 *
     *                                                                   *
     * Note: we do NOT feed the probe bit into try_receive_frame to     *
     * avoid buffering complexity.  One lost bit from a 40-bit preamble *
     * is inconsequential.                                              */
    (void)probe;
    (void)try_receive_frame();

    /* Stale-data check on every RF_Task pass */
    if (s_dataValid &&
        (HAL_GetTick() - s_lastRxTick) > RF_WIRELESS_TIMEOUT_MS) {
        s_dataValid = false;
        uart_ln("[RF RX] data expired");
    }
}
