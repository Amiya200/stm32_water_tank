/* ====================================================================
 * rf.c  —  RECEIVER  433 MHz OOK/ASK driver
 *
 * v6.6 — read_bit() timing fixed (critical bug)
 *
 * ROOT CAUSE of s_rxPackets = 0:
 *   read_bit() called wait_high_us() which internally calls tim_rst()
 *   and spins until pin goes HIGH.  Then read_bit() called tim_rst()
 *   AGAIN before measuring the pulse.  This second reset wiped out any
 *   elapsed time since the rising edge, so all measurements started
 *   near zero.  A 300µs bit-1 pulse measured as < 100µs → returns -1.
 *   Every single bit returned -1 → sync hunt never completed → packets
 *   never received → s_rxPackets stays 0 forever.
 *
 * FIX — read_bit() now works in three strictly ordered steps:
 *   Step 1: Wait for LOW  (end of previous bit's LOW phase, or idle).
 *           Timeout 2400µs (2 full bit periods).
 *           Skipped if pin is already LOW on entry.
 *   Step 2: Wait for HIGH (rising edge — start of this bit's pulse).
 *           Timeout 2400µs.
 *           Timer is NOT reset here; timer runs from before step 2.
 *   Step 3: Record the tick at the exact moment HIGH is detected,
 *           then spin until pin goes LOW, measuring the HIGH duration.
 *           This gives a clean measurement from rising to falling edge.
 *
 *   Both wait loops and the measurement loop share the SAME timer
 *   counter.  The counter runs continuously; only the start-of-measure
 *   snapshot (t_rise) is recorded at the rising edge instant.
 *
 * Pin: PD1 (RF_connector_Pin = GPIO_PIN_1, RF_connector_GPIO_Port = GPIOD)
 *   Schematic: CN1 screw terminal → RF_CONNECTOR net → MCU pin PD1.
 *   PD01 AFIO remap MANDATORY in MX_GPIO_Init() before HAL_GPIO_Init().
 *   No pull resistor — XY-MK-5V drives DATA actively.
 *
 * Other fixes carried from v6.5:
 *   - TIM3-based activity gate (no HAL_GetTick for µs windows)
 *   - No HAL_Delay(10) in main loop (see rx_main.c)
 *   - No stale wait_low() at try_receive_frame() entry
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

/* ── TIM3: 64 MHz / (63+1) = 1 MHz → 1 µs/count ───────────────────── */
#define RF_TIM3_PRESCALER    63u

/* ── Activity gate: how long RF_Task() waits for a preamble edge ────── *
 * 5000 µs = just over 4 bit periods.  Long enough to catch a preamble  *
 * HIGH with certainty; short enough to keep the function non-blocking.  */
#define RF_IDLE_GATE_US      5000u

/* ── read_bit() inter-bit timeout ───────────────────────────────────── *
 * Maximum time to wait for the LOW→HIGH transition between bits.        *
 * Set to 2 full bit periods (2400µs) to tolerate TX jitter.            */
#define RF_BIT_EDGE_TIMEOUT_US   2400u

/* ── Sync word ──────────────────────────────────────────────────────── */
#define RF_SYNC16  ((uint16_t)(((uint16_t)RF_SYNC_BYTE1 << 8u) | RF_SYNC_BYTE2))

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
 *  Reads PD1.  PD01 remap must be active in AFIO or always returns 0.
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
 *  READ ONE RF BIT  — v6.6 corrected
 *
 *  Encoding reminder (from rf.h):
 *    bit '1': HIGH 300µs  + LOW 900µs  (total 1200µs)
 *    bit '0': HIGH 900µs  + LOW 300µs  (total 1200µs)
 *
 *  Correct measurement sequence:
 *    1. If pin is HIGH on entry (we're inside a pulse), wait for it
 *       to go LOW first.  This handles the case where RF_Task() entered
 *       try_receive_frame() mid-preamble with pin already HIGH.
 *    2. Wait for pin to go HIGH (rising edge = start of next pulse).
 *       Record tim_now() at the exact instant pin goes HIGH (t_rise).
 *    3. Wait for pin to go LOW (falling edge = end of pulse).
 *       Record tim_now() (t_fall).
 *    4. pulse_width = t_fall - t_rise.
 *       100–550µs → bit 1.
 *       600–1100µs → bit 0.
 *
 *  All steps use the SAME free-running TIM3 counter.  No second reset
 *  between steps — that was the v6.4/6.5 bug.
 *
 *  Returns: 1, 0, or -1 (timeout / bad width).
 * ═══════════════════════════════════════════════════════════════════════ */
static int8_t read_bit(void)
{
    /* Step 1: if already HIGH, wait for LOW (end of current pulse).
     *         Timeout: ZERO_MAX + 300µs guard (1400µs).               */
    if (rf_pin() == 1u)
    {
        tim_start();
        while (rf_pin() == 1u)
        {
            if (tim_now() > (RF_THRESH_ZERO_MAX_US + 300u))
                return -1;  /* stuck HIGH — line error or noise burst */
        }
    }

    /* Step 2: wait for rising edge (pin goes HIGH).
     *         Timeout: RF_BIT_EDGE_TIMEOUT_US (2400µs).               */
    tim_start();
    while (rf_pin() == 0u)
    {
        if (tim_now() > RF_BIT_EDGE_TIMEOUT_US)
            return -1;  /* no rising edge — gap between packets or noise */
    }
    uint32_t t_rise = tim_now();   /* exact tick when pin went HIGH */

    /* Step 3: wait for falling edge (pin goes LOW).
     *         Timeout: ZERO_MAX + 300µs guard (1400µs) from t_rise.  */
    while (rf_pin() == 1u)
    {
        if ((tim_now() - t_rise) > (RF_THRESH_ZERO_MAX_US + 300u))
            return -1;  /* pulse too long */
    }
    uint32_t t_fall = tim_now();

    /* Step 4: classify pulse width */
    uint32_t w = t_fall - t_rise;

    if (w >= RF_THRESH_ONE_MIN_US  && w <= RF_THRESH_ONE_MAX_US)  return 1;
    if (w >= RF_THRESH_ZERO_MIN_US && w <= RF_THRESH_ZERO_MAX_US) return 0;
    return -1;  /* width outside both windows — noise or timing drift */
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
 *  Called when RF_Task() detects the preamble rising edge on PD1.
 *  Pin may already be HIGH on entry.
 *
 *  Frame: [5×0xAA preamble] [0x2D sync1] [0xD4 sync2] [len] [payload] [CRC8]
 *
 *  Phase 1 (sync hunt):
 *    read_bit() handles HIGH-at-entry correctly (Step 1 waits for LOW).
 *    Bits are shifted into sreg; match against RF_SYNC16 (0x2DD4).
 *    Requires RF_MIN_VALID_PREAMBLE_BITS clean bits before accepting sync
 *    to reject spurious glitch triggers.
 * ═══════════════════════════════════════════════════════════════════════ */
static bool try_receive_frame(void)
{
    uint16_t sreg      = 0u;
    uint32_t bits      = 0u;
    uint32_t good_bits = 0u;
    bool     found     = false;

    /* ── Phase 1: sync hunt ─────────────────────────────────────────── */
    while (bits < RF_SYNC_HUNT_MAX_BITS)
    {
        int8_t b = read_bit();

        if (b < 0)
        {
            good_bits = 0u;
            b = read_bit();           /* one retry on bad bit */
            if (b < 0) return false;  /* two bad in a row → abandon */
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
    uart_ln("[RF RX] Init v6.6...");
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
    uart_ln("[RF RX] Gate    : 5000 us idle gate (TIM3)");
    uart_ln("[RF RX] READY   : main loop must have NO HAL_Delay in RF433 mode");
}

/* ═══════════════════════════════════════════════════════════════════════
 *  RF_Task
 *
 *  Non-blocking idle path: waits up to RF_IDLE_GATE_US (5ms) for a
 *  rising edge on PD1.  Returns immediately if none detected.
 *
 *  Active path: rising edge detected → try_receive_frame() decodes
 *  the complete frame (~200ms for full packet).  Main loop blocks here
 *  — unavoidable for bit-bang OOK.
 *
 *  CALLER MUST NOT call HAL_Delay() between RF_Task() invocations
 *  when g_wireless_mode == WIRELESS_MODE_RF433.
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

    /* Activity gate — TIM3 based for µs precision.
     * If pin already HIGH: skip gate and decode immediately.
     * If pin LOW: wait up to RF_IDLE_GATE_US for a rising edge.       */
    if (rf_pin() == 0u)
    {
        tim_start();
        while (rf_pin() == 0u)
        {
            if (tim_now() > RF_IDLE_GATE_US)
                return;   /* idle — no activity */
        }
        /* Rising edge detected within gate window */
    }

    /* Pin is HIGH (or just went HIGH) — decode frame */
    (void)try_receive_frame();
}
