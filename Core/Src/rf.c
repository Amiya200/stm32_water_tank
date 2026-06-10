/* ====================================================================
 * rf.c  —  RECEIVER  433 MHz OOK/ASK driver
 *
 * v6.7 — Superregenerative-receiver (XY-MK-5V) hardening
 *
 * WHY: the red modules are SUPERHETERODYNE (clean, squelched DATA out).
 *      The green XY-MK-5V is SUPERREGENERATIVE — while the carrier is
 *      off it ramps AGC to maximum and sprays the DATA line with noise:
 *      sub-100µs spikes plus AGC "pumping" during the 900µs carrier-off
 *      half of every '1' bit. The v6.6 edge-timed decoder trusted every
 *      edge, so that noise destroyed the bit stream (read_bit() locked
 *      onto spikes -> -1; the "two bad bits -> abort" sync hunt bailed
 *      before ever seeing 0x2DD4) -> s_rxPackets stuck at 0.
 *
 * FOUR RX-only changes (TX firmware and rf.h wire format UNCHANGED):
 *   1. Glitch-confirmed rising edge in read_bit(): a real carrier pulse
 *      holds HIGH >= RF_GLITCH_CONFIRM_US; noise spikes don't, so they
 *      are rejected before they are ever measured.
 *   2. Single-threshold bit classification: removes the old 550-600µs
 *      dead zone. Superregen STRETCHES pulses, so anything landing in
 *      that gap used to be thrown away. Now HIGH < RF_BIT_SPLIT_US => 1,
 *      else => 0, with min/max sanity bounds.
 *   3. Noise-tolerant sync hunt: a bad bit resets the clean-run AND the
 *      shift register (noise can't be stitched into a false sync) instead
 *      of aborting; bounded by RF_SYNC_HUNT_MAX_ATTEMPTS.
 *   4. Glitch-confirmed activity gate in RF_Task(): idle superregen noise
 *      no longer launches a full frame decode every loop iteration.
 *
 * ── v6.6 history (read_bit timing fix — still in force) ──────────────
 *   read_bit() uses ONE free-running TIM3 counter. It records the tick
 *   at the rising edge (t_rise) and measures the HIGH width to the
 *   falling edge (t_fall). No second timer reset between detect and
 *   measure — that double-reset was the original s_rxPackets = 0 bug.
 *
 * Pin: PD1 (RF_connector_Pin = GPIO_PIN_1, RF_connector_GPIO_Port = GPIOD)
 *   PD01 AFIO remap MANDATORY in MX_GPIO_Init() before HAL_GPIO_Init().
 *   No pull resistor — XY-MK-5V drives DATA actively (a pull would
 *   distort the module's AGC envelope).
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

/* ── TIM3: 64 MHz / (63+1) = 1 MHz -> 1 µs/count ──────────────────── */
#define RF_TIM3_PRESCALER    63u

/* ── Activity gate: how long RF_Task() waits for a preamble edge ────── *
 * 5000 µs = just over 4 bit periods. Long enough to catch a preamble   *
 * HIGH with certainty; short enough to keep the function non-blocking.  */
#define RF_IDLE_GATE_US      5000u

/* ── read_bit() inter-bit timeout ───────────────────────────────────── *
 * Maximum time to wait for the LOW->HIGH transition between bits.       *
 * 2 full bit periods (2400µs) to tolerate TX jitter.                   */
#define RF_BIT_EDGE_TIMEOUT_US   2400u

/* ── Sync word ──────────────────────────────────────────────────────── */
#define RF_SYNC16  ((uint16_t)(((uint16_t)RF_SYNC_BYTE1 << 8u) | RF_SYNC_BYTE2))

/* ── Superregen (XY-MK-5V) hardening — RX decode tuning ─────────────── *
 * Superregen receivers spray noise on DATA while the carrier is off and *
 * pump their AGC, so the raw line is full of sub-100µs spikes. These    *
 * constants let the decoder reject that noise. All tunable.             *
 *                                                                       *
 *   RF_GLITCH_CONFIRM_US : raise (e.g. 80-100) to reject more noise,    *
 *                          lower if real '1' pulses get rejected.       *
 *   RF_BIT_SPLIT_US      : midpoint between a 300µs '1' and 900µs '0'.  *
 *   RF_BIT_WIDTH_MIN/MAX : hard sanity bounds on the HIGH pulse.        */
#define RF_GLITCH_CONFIRM_US        60u   /* HIGH must persist this long to be a real edge */
#define RF_BIT_WIDTH_MIN_US        150u   /* HIGH shorter than this = noise, reject        */
#define RF_BIT_WIDTH_MAX_US       1300u   /* HIGH longer than this  = error, reject        */
#define RF_BIT_SPLIT_US            600u   /* HIGH < split => '1' (short), >= split => '0'  */
#define RF_SYNC_BAD_RUN_MAX         10u   /* consecutive bad bits before abandoning hunt   */
#define RF_SYNC_HUNT_MAX_ATTEMPTS  400u   /* total read_bit() calls per hunt (time bound)  */

extern TIM_HandleTypeDef  htim3;
extern UART_HandleTypeDef huart1;

static void uart_ln(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s,      (uint16_t)strlen(s), 500u);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2u,                  500u);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  MODULE STATE
 * ═══════════════════════════════════════════════════════════════════════ */
static uint8_t  s_rfLevel      = 0u;
static uint8_t  s_rfWellDry    = 0u;
static uint32_t s_rfLastRxTick = 0u;
static bool     s_rfDataValid  = false;

static char     s_rfLastRawPacket[RF_MAX_PAYLOAD + 1u] = {0};
static uint32_t s_rfLastDid      = 0u;
static uint32_t s_rfLastSeq      = 0u;
static uint8_t  s_rfLastType     = 0u;
static uint8_t  s_rfLastCrcRx    = 0u;
static uint8_t  s_rfLastCrcCalc  = 0u;
static uint32_t s_rfLastPacketMs = 0u;
static bool     s_rfPacketSeen   = false;

static uint32_t s_rfDupLastSeq  = 0u;
static bool     s_rfDupSeqInit  = false;

static uint32_t s_rxPackets = 0u;
static uint32_t s_rxErrors  = 0u;
static bool     s_hwOk      = false;

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
 *  PIN READER
 *  Reads PD1. PD01 remap must be active in AFIO or always returns 0.
 * ═══════════════════════════════════════════════════════════════════════ */
static inline uint8_t rf_pin(void)
{
    return (HAL_GPIO_ReadPin(RF_connector_GPIO_Port, RF_connector_Pin)
            == GPIO_PIN_SET) ? 1u : 0u;
}

/* ── TIM3 counter helpers ───────────────────────────────────────────── */
static inline void     tim_start(void) { __HAL_TIM_SET_COUNTER(&htim3, 0u); }
static inline uint32_t tim_now(void)   { return __HAL_TIM_GET_COUNTER(&htim3); }

/* ═══════════════════════════════════════════════════════════════════════
 *  READ ONE RF BIT  — v6.7 (glitch-confirmed edge + single threshold)
 *
 *  Encoding (from rf.h):
 *    bit '1': HIGH 300µs + LOW 900µs   (total 1200µs)
 *    bit '0': HIGH 900µs + LOW 300µs   (total 1200µs)
 *
 *  Sequence (all on ONE free-running TIM3 counter):
 *    1. If HIGH on entry, wait out the current pulse.
 *    2. Hunt for a CONFIRMED rising edge — the line must hold HIGH for
 *       >= RF_GLITCH_CONFIRM_US. Superregen idle/AGC noise is only brief
 *       spikes, so it is rejected here and never measured.
 *    3. Measure HIGH width from t_rise to the falling edge (t_fall).
 *    4. Classify by a single threshold (no dead zone):
 *         w <  RF_BIT_SPLIT_US -> 1   (short pulse)
 *         w >= RF_BIT_SPLIT_US -> 0   (long  pulse)
 *       with min/max sanity bounds.
 *
 *  Returns: 1, 0, or -1 (timeout / out-of-range / noise).
 * ═══════════════════════════════════════════════════════════════════════ */
static int8_t read_bit(void)
{
    uint32_t t_rise = 0u, t_fall = 0u, w = 0u, tc = 0u;
    bool     glitch = false;

    /* Step 1: if HIGH on entry, wait out the current pulse first. */
    if (rf_pin() == 1u)
    {
        tim_start();
        while (rf_pin() == 1u)
            if (tim_now() > RF_BIT_WIDTH_MAX_US) return -1;
    }

    /* Step 2: hunt for a CONFIRMED rising edge.
     *         A genuine carrier pulse stays HIGH >= RF_GLITCH_CONFIRM_US;
     *         superregen idle noise produces only brief spikes that are
     *         rejected here. Timer keeps running across glitches; the
     *         overall wait is bounded by RF_BIT_EDGE_TIMEOUT_US.        */
    tim_start();
    for (;;)
    {
        if (tim_now() > RF_BIT_EDGE_TIMEOUT_US) return -1;   /* no edge */
        if (rf_pin() == 0u) continue;

        t_rise = tim_now();                 /* candidate rising edge      */
        tc     = t_rise;
        glitch = false;
        while ((tim_now() - tc) < RF_GLITCH_CONFIRM_US)
            if (rf_pin() == 0u) { glitch = true; break; }
        if (!glitch) break;                 /* confirmed real rising edge */
        /* else: glitch — keep hunting                                    */
    }

    /* Step 3: measure HIGH width to the falling edge. */
    while (rf_pin() == 1u)
        if ((tim_now() - t_rise) > RF_BIT_WIDTH_MAX_US) return -1;
    t_fall = tim_now();

    /* Step 4: single-threshold classification (no dead zone). */
    w = t_fall - t_rise;
    if (w < RF_BIT_WIDTH_MIN_US) return -1;
    return (w < RF_BIT_SPLIT_US) ? (int8_t)1 : (int8_t)0;
}

/* ── read_byte: MSB first, 8 bits ───────────────────────────────────── */
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
 *  Called when RF_Task() detects a confirmed preamble edge on PD1.
 *  Pin may already be HIGH on entry.
 *
 *  Frame: [5×0xAA preamble] [0x2D sync1] [0xD4 sync2] [len] [payload] [CRC8]
 * ═══════════════════════════════════════════════════════════════════════ */
static bool try_receive_frame(void)
{
    uint16_t sreg      = 0u;
    uint32_t good_bits = 0u;
    uint32_t bad_run   = 0u;
    uint32_t attempts  = 0u;
    bool     found     = false;

    /* ── Phase 1: noise-tolerant sync hunt ──────────────────────────── *
     * A bad bit resets the clean-run AND the shift register (so noise   *
     * can't be concatenated into a false sync) instead of aborting.     *
     * Only a clean RF_SYNC16 built from >= RF_MIN_VALID_PREAMBLE_BITS    *
     * consecutive good bits is accepted. Attempts are bounded for time. */
    while (attempts < RF_SYNC_HUNT_MAX_ATTEMPTS)
    {
        attempts++;
        int8_t b = read_bit();

        if (b < 0)
        {
            good_bits = 0u;
            sreg      = 0u;
            if (++bad_run >= RF_SYNC_BAD_RUN_MAX) return false;
            continue;                       /* don't shift noise into sreg */
        }
        bad_run = 0u;

        sreg = (uint16_t)((sreg << 1u) | (uint8_t)b);
        good_bits++;

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
        uart_ln("[RF RX] ERR: bad length");
        return false;
    }

    /* ── Phase 3: payload ──────────────────────────────────────────── */
    char payload[RF_MAX_PAYLOAD + 1u];
    for (uint8_t i = 0u; i < len; i++)
    {
        if (!read_byte((uint8_t *)&payload[i])) return false;
    }
    payload[len] = '\0';

    /* ── Phase 4: CRC ──────────────────────────────────────────────── */
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
                 "[RF RX] CRC FAIL raw=\"%s\" rx=0x%02X calc=0x%02X",
                 s_rfLastRawPacket, (unsigned)rx_crc, (unsigned)calc_crc);
        uart_ln(dbg);
        return false;
    }

    /* ── Phase 5: parse ────────────────────────────────────────────── */
    ParsedPacket_t p;
    if (!LoRa_ParsePacket(payload, &p))
    {
        s_rxErrors++;
        char dbg[160];
        snprintf(dbg, sizeof(dbg), "[RF RX] PARSE FAIL raw=\"%s\"",
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
                 "[RF RX] OK #%lu type=%d DID=%08lX SEQ=%08lX \"%s\"",
                 (unsigned long)s_rxPackets, (int)p.type,
                 (unsigned long)p.did, (unsigned long)p.seq,
                 s_rfLastRawPacket);
        uart_ln(dbg);
    }

    /* ── Phase 6: TANKLEVEL ─────────────────────────────────────────── */
    if (p.type == PKT_TYPE_TANKLEVEL)
    {
        if (s_rfDupSeqInit && p.seq == s_rfDupLastSeq)
        {
            uart_ln("[RF RX] dup seq — discarded");
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
                 "[RF RX] ACCEPTED Level=%u%% WD=%u Seq=%08lX DID=%08lX",
                 (unsigned)s_rfLevel, (unsigned)s_rfWellDry,
                 (unsigned long)s_rfLastSeq, (unsigned long)s_rfLastDid);
        uart_ln(dbg);
        return true;
    }

    /* ── HELLO ─────────────────────────────────────────────────────── */
    if (p.type == PKT_TYPE_HELLO)
    {
        char dbg[160];
        snprintf(dbg, sizeof(dbg),
                 "[RF RX] HELLO DID=%08lX raw=\"%s\"",
                 (unsigned long)s_rfLastDid, s_rfLastRawPacket);
        uart_ln(dbg);
        s_rfLastRxTick = HAL_GetTick();
        return true;
    }

    uart_ln("[RF RX] unknown type");
    return false;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  PUBLIC ACCESSORS
 * ═══════════════════════════════════════════════════════════════════════ */
bool RF_IsWirelessDataValid(void)
{
    if (!s_rfDataValid) return false;
    if ((HAL_GetTick() - s_rfLastRxTick) > RF_WIRELESS_TIMEOUT_MS)
    {
        s_rfDataValid = false;
        uart_ln("[RF RX] timeout — local ADC fallback");
        return false;
    }
    return true;
}

uint8_t       RF_GetWirelessTankLevel (void) { return s_rfLevel;        }
uint8_t       RF_GetWirelessWellDry   (void) { return s_rfWellDry;      }
bool          RF_HasReceivedPacket    (void) { return s_rfPacketSeen;   }
const char   *RF_GetLastRawPacket     (void) { return s_rfLastRawPacket;}
uint32_t      RF_GetLastPacketDID     (void) { return s_rfLastDid;      }
uint32_t      RF_GetLastPacketSeq     (void) { return s_rfLastSeq;      }
uint8_t       RF_GetLastPacketType    (void) { return s_rfLastType;     }
uint8_t       RF_GetLastPacketCrcRx   (void) { return s_rfLastCrcRx;    }
uint8_t       RF_GetLastPacketCrcCalc (void) { return s_rfLastCrcCalc;  }
uint32_t      RF_GetRxPacketCount     (void) { return s_rxPackets;      }
uint32_t      RF_GetRxErrorCount      (void) { return s_rxErrors;       }

uint32_t RF_GetLastPacketAgeMs(void)
{
    if (!s_rfPacketSeen) return 0xFFFFFFFFUL;
    return HAL_GetTick() - s_rfLastPacketMs;
}

void RF_ClearWirelessData(void)
{
    s_rfLevel            = 0u;
    s_rfWellDry          = 0u;
    s_rfDataValid        = false;
    s_rfLastRxTick       = 0u;
    s_rfDupSeqInit       = false;
    s_rfDupLastSeq       = 0u;
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
    uart_ln("[RF RX] Init v6.7 (superregen-hardened)...");
    uart_ln("[RF RX] Pin     : PD1 (RF_connector_Pin = GPIO_PIN_1, GPIOD)");
    uart_ln("[RF RX] Remap   : __HAL_AFIO_REMAP_PD01_ENABLE() in MX_GPIO_Init");
    uart_ln("[RF RX] No pull : INPUT no pull (XY-MK-5V drives actively)");

    HAL_TIM_Base_Stop(&htim3);
    __HAL_TIM_SET_PRESCALER(&htim3, RF_TIM3_PRESCALER);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 0xFFFFu);
    htim3.Instance->EGR |= TIM_EGR_UG;   /* force prescaler load */
    HAL_TIM_Base_Start(&htim3);

    RF_ClearWirelessData();
    s_rxPackets = 0u;
    s_rxErrors  = 0u;
    s_hwOk      = true;

    uart_ln("[RF RX] TIM3    : 1 MHz (prescaler=63)");
    uart_ln("[RF RX] Gate    : 5000 us idle gate, glitch-confirmed");
    uart_ln("[RF RX] Decode  : glitch-confirmed edges + single threshold");
    uart_ln("[RF RX] READY   : main loop must have NO HAL_Delay in RF433 mode");
}

/* ═══════════════════════════════════════════════════════════════════════
 *  RF_Task
 *
 *  Non-blocking idle path: waits up to RF_IDLE_GATE_US for a CONFIRMED
 *  rising edge on PD1. Returns immediately if none. Glitch-confirm stops
 *  superregen idle noise from launching a full frame decode every loop.
 *
 *  Active path: confirmed edge -> try_receive_frame() decodes the frame
 *  (~200ms for a full packet). Main loop blocks here — unavoidable for
 *  bit-bang OOK.
 *
 *  CALLER MUST NOT call HAL_Delay() between RF_Task() invocations when
 *  g_wireless_mode == WIRELESS_MODE_RF433.
 * ═══════════════════════════════════════════════════════════════════════ */
void RF_Task(void)
{
    if (!s_hwOk) return;

    /* Data expiry */
    if (s_rfDataValid &&
        (HAL_GetTick() - s_rfLastRxTick) > RF_WIRELESS_TIMEOUT_MS)
    {
        s_rfDataValid = false;
        uart_ln("[RF RX] data expired");
    }

    /* Activity gate — wait up to RF_IDLE_GATE_US for a CONFIRMED HIGH.
     * A real carrier holds HIGH >= RF_GLITCH_CONFIRM_US; superregen idle
     * noise spikes don't, so they don't trigger a frame decode. If pin
     * is already HIGH on entry (mid-preamble) it is confirmed and we go
     * straight to decode.                                              */
    tim_start();
    for (;;)
    {
        if (tim_now() > RF_IDLE_GATE_US) return;     /* idle, no activity */
        if (rf_pin() == 0u) continue;

        uint32_t tc = tim_now();
        bool glitch = false;
        while ((tim_now() - tc) < RF_GLITCH_CONFIRM_US)
            if (rf_pin() == 0u) { glitch = true; break; }
        if (!glitch) break;                          /* real carrier */
    }

    /* Confirmed carrier — decode frame */
    (void)try_receive_frame();
}
