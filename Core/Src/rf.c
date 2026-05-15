/* ====================================================================
 * rf.c  —  RECEIVER  433 MHz OOK/ASK driver
 *
 * Compile this file into the RECEIVER project (LORA_RECEIVER_NODE).
 *
 * Hardware: XY-MK-5V / MX-RM-5V or equivalent superheterodyne
 *           433 MHz receiver module.
 *
 *   VCC  → 5 V
 *   GND  → GND
 *   DATA → RF_DATA_Pin  ← GPIO_MODE_INPUT, no pull
 *
 * How to call from main.c:
 *
 *   RF_Init();
 *
 *   while (1)
 *   {
 *       RF_Task();
 *   }
 *
 * This receiver stores RF data separately from LoRa and UART:
 *
 *   s_rfLevel
 *   s_rfWellDry
 *   s_rfDataValid
 *   s_rfLastRawPacket
 *   s_rfLastDid
 *   s_rfLastSeq
 *   s_rfLastType
 *
 * UART is used only for debug printing.
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

/* ── Board-specific ─────────────────────────────────────────────────── */
#define RF_TIM3_PRESCALER   63u     /* 64 MHz / 64 = 1 MHz */

/* ── External handles ───────────────────────────────────────────────── */
extern TIM_HandleTypeDef  htim3;
extern UART_HandleTypeDef huart1;

/* ── UART helper: debug only ────────────────────────────────────────── */
static void uart_ln(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 500u);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2u, 500u);
}

/* ═══════════════════════════════════════════════════════════════════════
 * RF-ONLY WIRELESS DATA STORE
 * These variables are separate from LoRa and UART.
 * UART only prints debug logs.
 * ═══════════════════════════════════════════════════════════════════════ */

static uint8_t  s_rfLevel      = 0u;
static uint8_t  s_rfWellDry    = 0u;
static uint32_t s_rfLastRxTick = 0u;
static bool     s_rfDataValid  = false;

/*
 * RF-only packet visibility/debug variables.
 * These help you see exactly what RF transmitter sent.
 */
static char     s_rfLastRawPacket[RF_MAX_PAYLOAD + 1u] = {0};
static uint32_t s_rfLastDid      = 0u;
static uint32_t s_rfLastSeq      = 0u;
static uint8_t  s_rfLastType     = 0u;
static uint8_t  s_rfLastCrcRx    = 0u;
static uint8_t  s_rfLastCrcCalc  = 0u;
static uint32_t s_rfLastPacketMs = 0u;
static bool     s_rfPacketSeen   = false;

/* Duplicate filter — RF-only */
static uint32_t s_rfDupLastSeq = 0u;
static bool     s_rfDupSeqInit = false;

/* Receiver counters */
static uint32_t s_rxPackets = 0u;   /* accepted good parsed packets */
static uint32_t s_rxErrors  = 0u;   /* CRC, length, parse errors */

/* RF hardware init flag */
static bool s_hwOk = false;

/* ═══════════════════════════════════════════════════════════════════════
 * CRC-8  poly 0x07, init 0x00
 * ═══════════════════════════════════════════════════════════════════════ */

uint8_t RF_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00u;

    while (len--)
    {
        crc ^= *data++;

        for (int i = 0; i < 8; i++)
        {
            crc = (crc & 0x80u) ? ((crc << 1u) ^ 0x07u) : (crc << 1u);
        }
    }

    return crc;
}

/* ═══════════════════════════════════════════════════════════════════════
 * LOW LEVEL PIN / TIMER HELPERS
 * ═══════════════════════════════════════════════════════════════════════ */

static inline uint8_t rf_pin(void)
{
    if (HAL_GPIO_ReadPin(RF_DATA_GPIO_Port, RF_DATA_Pin) == GPIO_PIN_SET)
        return 1u;

    return 0u;
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
        if ((HAL_GetTick() - t0) >= timeout_ms)
            return false;
    }

    return true;
}

static bool wait_high(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    while (rf_pin() != 1u)
    {
        if ((HAL_GetTick() - t0) >= timeout_ms)
            return false;
    }

    return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 * READ ONE RF BIT
 *
 * Logic:
 *   HIGH pulse 100–550 us   = bit 1
 *   HIGH pulse 600–1100 us  = bit 0
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

    if (w >= RF_THRESH_ONE_MIN_US && w <= RF_THRESH_ONE_MAX_US)
        return 1;

    if (w >= RF_THRESH_ZERO_MIN_US && w <= RF_THRESH_ZERO_MAX_US)
        return 0;

    return -1;
}

static bool read_byte(uint8_t *out)
{
    uint8_t b = 0u;

    for (int8_t i = 7; i >= 0; i--)
    {
        int8_t bit = read_bit();

        if (bit < 0)
            return false;

        b |= (uint8_t)((uint8_t)bit << (uint8_t)i);
    }

    *out = b;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 * TRY RECEIVE ONE COMPLETE RF FRAME
 *
 * Frame format:
 *   preamble + sync + len + payload + CRC
 *
 * Payload example:
 *   @HI:70A2EE7E#
 *   @TL:000,WD:1,SQ:00000003,DID:70A2EE7E#
 *
 * The RF receiver stores raw and parsed packet information separately.
 * ═══════════════════════════════════════════════════════════════════════ */

static bool try_receive_frame(void)
{
    if (!wait_low(3u))
        return false;

#define SYNC16  ((uint16_t)((RF_SYNC_BYTE1 << 8u) | RF_SYNC_BYTE2))

    uint16_t sreg       = 0u;
    uint32_t bits       = 0u;
    uint32_t good_bits  = 0u;
    bool     sync_found = false;

    /* ── Phase 1: preamble + sync hunt ─────────────────────────────── */
    while (bits < RF_SYNC_HUNT_MAX_BITS)
    {
        int8_t b = read_bit();

        if (b < 0)
        {
            good_bits = 0u;

            b = read_bit();
            if (b < 0)
                return false;
        }

        bits++;
        good_bits++;

        sreg = (uint16_t)((sreg << 1u) | (uint8_t)b);

        if (sreg == SYNC16 && good_bits >= RF_MIN_VALID_PREAMBLE_BITS)
        {
            sync_found = true;
            break;
        }
    }

    if (!sync_found)
        return false;

    /* ── Phase 2: length ────────────────────────────────────────────── */
    uint8_t len = 0u;

    if (!read_byte(&len))
        return false;

    if (len == 0u || len > RF_MAX_PAYLOAD)
    {
        s_rxErrors++;
        uart_ln("[RF RX PACKET] invalid length");
        return false;
    }

    /* ── Phase 3: payload ───────────────────────────────────────────── */
    char payload[RF_MAX_PAYLOAD + 1u];

    for (uint8_t i = 0u; i < len; i++)
    {
        if (!read_byte((uint8_t *)&payload[i]))
            return false;
    }

    payload[len] = '\0';

    /* ── Phase 4: CRC ───────────────────────────────────────────────── */
    uint8_t rx_crc = 0u;

    if (!read_byte(&rx_crc))
        return false;

    uint8_t calc_crc = RF_CRC8((const uint8_t *)payload, len);

    /*
     * Store raw RF packet even if CRC fails.
     * This is RF-only debug data.
     */
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
                 "[RF RX PACKET] CRC_FAIL RAW=\"%s\" RXCRC=0x%02X CALC=0x%02X",
                 s_rfLastRawPacket,
                 (unsigned)s_rfLastCrcRx,
                 (unsigned)s_rfLastCrcCalc);

        uart_ln(dbg);
        return false;
    }

    /* ── Phase 5: parse payload ─────────────────────────────────────── */
    ParsedPacket_t p;

    if (!LoRa_ParsePacket(payload, &p))
    {
        s_rxErrors++;

        char dbg[160];
        snprintf(dbg, sizeof(dbg),
                 "[RF RX PACKET] PARSE_FAIL RAW=\"%s\"",
                 s_rfLastRawPacket);

        uart_ln(dbg);
        return false;
    }

    s_rxPackets++;

    /*
     * Store parsed RF packet metadata separately.
     */
    s_rfLastType = (uint8_t)p.type;
    s_rfLastDid  = p.did;
    s_rfLastSeq  = p.seq;

    {
        char dbg[180];
        snprintf(dbg, sizeof(dbg),
                 "[RF RX PACKET] OK RX#%lu TYPE=%d DID=%08lX SEQ=%08lX RAW=\"%s\"",
                 (unsigned long)s_rxPackets,
                 (int)p.type,
                 (unsigned long)p.did,
                 (unsigned long)p.seq,
                 s_rfLastRawPacket);

        uart_ln(dbg);
    }

    /* ── Phase 6: accept tank-level data ────────────────────────────── */
    if (p.type == PKT_TYPE_TANKLEVEL)
    {
        /*
         * Duplicate sequence filter — RF-only.
         * TX sends same packet multiple times, so duplicate seq is normal.
         */
        if (s_rfDupSeqInit && p.seq == s_rfDupLastSeq)
        {
            uart_ln("[RF RX PACKET] DUPLICATE_SEQ discarded");
            return false;
        }

        s_rfDupLastSeq = p.seq;
        s_rfDupSeqInit = true;

        /*
         * Store RF transmitter data in RF-only variables.
         */
        s_rfLevel      = p.level;
        s_rfWellDry    = p.well_dry;
        s_rfLastRxTick = HAL_GetTick();
        s_rfDataValid  = true;

        char dbg[180];
        snprintf(dbg, sizeof(dbg),
                 "[RF RX DATA] ACCEPTED Level=%u%% WD=%u Seq=%08lX DID=%08lX RAW=\"%s\"",
                 (unsigned)s_rfLevel,
                 (unsigned)s_rfWellDry,
                 (unsigned long)s_rfLastSeq,
                 (unsigned long)s_rfLastDid,
                 s_rfLastRawPacket);

        uart_ln(dbg);

        return true;
    }

    /* ── HELLO packet debug ─────────────────────────────────────────── */
    if (p.type == PKT_TYPE_HELLO)
    {
        char dbg[160];
        snprintf(dbg, sizeof(dbg),
                 "[RF RX HELLO] DID=%08lX RAW=\"%s\"",
                 (unsigned long)s_rfLastDid,
                 s_rfLastRawPacket);

        uart_ln(dbg);

        return true;
    }

    uart_ln("[RF RX PACKET] unknown packet type");
    return false;
}

/* ═══════════════════════════════════════════════════════════════════════
 * PUBLIC RF DATA ACCESSORS
 * These are RF-only accessors.
 * ═══════════════════════════════════════════════════════════════════════ */

bool RF_IsWirelessDataValid(void)
{
    if (!s_rfDataValid)
        return false;

    if ((HAL_GetTick() - s_rfLastRxTick) > RF_WIRELESS_TIMEOUT_MS)
    {
        s_rfDataValid = false;
        uart_ln("[RF RX] RF wireless data TIMED OUT — falling back to local ADC");
        return false;
    }

    return true;
}

uint8_t RF_GetWirelessTankLevel(void)
{
    return s_rfLevel;
}

uint8_t RF_GetWirelessWellDry(void)
{
    return s_rfWellDry;
}

/* RF-only debug/status functions */

bool RF_HasReceivedPacket(void)
{
    return s_rfPacketSeen;
}

const char* RF_GetLastRawPacket(void)
{
    return s_rfLastRawPacket;
}

uint32_t RF_GetLastPacketDID(void)
{
    return s_rfLastDid;
}

uint32_t RF_GetLastPacketSeq(void)
{
    return s_rfLastSeq;
}

uint8_t RF_GetLastPacketType(void)
{
    return s_rfLastType;
}

uint32_t RF_GetLastPacketAgeMs(void)
{
    if (!s_rfPacketSeen)
        return 0xFFFFFFFFUL;

    return HAL_GetTick() - s_rfLastPacketMs;
}

uint8_t RF_GetLastPacketCrcRx(void)
{
    return s_rfLastCrcRx;
}

uint8_t RF_GetLastPacketCrcCalc(void)
{
    return s_rfLastCrcCalc;
}

uint32_t RF_GetRxPacketCount(void)
{
    return s_rxPackets;
}

uint32_t RF_GetRxErrorCount(void)
{
    return s_rxErrors;
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
 * RF INIT
 * ═══════════════════════════════════════════════════════════════════════ */

void RF_Init(void)
{
    uart_ln("[RF RX] Init...");

    HAL_TIM_Base_Stop(&htim3);

    __HAL_TIM_SET_PRESCALER(&htim3, RF_TIM3_PRESCALER);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 0xFFFFu);

    htim3.Instance->EGR |= TIM_EGR_UG;

    HAL_TIM_Base_Start(&htim3);

    RF_ClearWirelessData();

    s_rxPackets = 0u;
    s_rxErrors  = 0u;
    s_hwOk      = true;

    uart_ln("[RF RX] TIM3 reconfigured to 1 MHz (1 us/count)");
    uart_ln("[RF RX] Init complete — 433 MHz OOK RX ready");
}

/* ═══════════════════════════════════════════════════════════════════════
 * RF TASK
 *
 * Call from main loop continuously.
 * ═══════════════════════════════════════════════════════════════════════ */

void RF_Task(void)
{
    if (!s_hwOk)
        return;

    /*
     * Step 1:
     * If RF pin is LOW and stays LOW for 5 ms, no RF packet is active.
     */
    if (rf_pin() == 0u)
    {
        uint32_t t0 = HAL_GetTick();

        while (rf_pin() == 0u)
        {
            if ((HAL_GetTick() - t0) >= 5u)
            {
                /*
                 * Periodic RF data expiry check while idle.
                 */
                if (s_rfDataValid &&
                    (HAL_GetTick() - s_rfLastRxTick) > RF_WIRELESS_TIMEOUT_MS)
                {
                    s_rfDataValid = false;
                    uart_ln("[RF RX] RF data expired (idle check)");
                }

                return;
            }
        }

        /*
         * Pin just went HIGH.
         * Fall through to decode attempt.
         */
    }

    /*
     * Step 2:
     * Pin is HIGH. Wait for clean LOW before reading bit stream.
     */
    if (!wait_low(2u))
        return;

    /*
     * Step 3:
     * Probe one bit. If invalid, treat it as noise.
     */
    int8_t probe = read_bit();

    if (probe < 0)
        return;

    /*
     * Step 4:
     * Valid pulse seen. Try full RF frame decode.
     *
     * The probe bit is consumed. Remaining preamble bits are enough.
     */
    (void)probe;
    (void)try_receive_frame();

    /*
     * Stale-data check after decode attempt.
     */
    if (s_rfDataValid &&
        (HAL_GetTick() - s_rfLastRxTick) > RF_WIRELESS_TIMEOUT_MS)
    {
        s_rfDataValid = false;
        uart_ln("[RF RX] RF data expired");
    }
}
