/* ====================================================================
 * lora.c  —  RECEIVER  (motor-controller node)
 *
 * NODE TYPE: LORA_RECEIVER_NODE
 *
 * What changed vs the previous version:
 *
 *  1. parse_data_packet() now extracts the WD (well-dry) field that
 *     the transmitter embeds in every data packet.
 *     Packet format: @TL:080,WD:0,SN:00003#
 *     WD is OPTIONAL for backward compatibility:
 *       • If ",WD:" is present  → parsed and stored in g_wirelessWellDry
 *       • If ",WD:" is absent   → g_wirelessWellDry defaults to 0 (OK)
 *
 *  2. g_wirelessWellDry added to module state.
 *     Exposed via LoRa_GetWirelessWellDry().
 *     adc.c uses this to inject CH5 (dry-run sensor) when wireless
 *     data is valid, so the motor-protection FSM reacts to a remote
 *     well-dry alarm even without a physical wire to the well.
 *
 *  3. g_loraNewPacketFlag set on every successfully parsed data
 *     packet.  main.c clears it and sets g_screenUpdatePending so the
 *     LCD refreshes immediately rather than waiting for the 400 ms
 *     blink timer.
 *
 *  4. g_loraConnected lifecycle:
 *       • Set   1  on first valid data packet OR @HI# handshake reply
 *       • Cleared 0  when WIRELESS_TIMEOUT_MS elapses without a packet
 *         (detected inside LoRa_IsWirelessDataValid(), called each
 *          loop iteration via the watchdog at the top of LoRa_Task())
 *       • LED reflects state: GREEN-STEADY when connected,
 *                             RED-BLINK   when timeout/disconnected
 * ==================================================================== */

#include "lora.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "led.h"

extern SPI_HandleTypeDef hspi1;

/* ════════════════════════════════════════════════════════════════════
 *  MODULE STATE
 * ════════════════════════════════════════════════════════════════════ */
uint8_t  loraMode        = LORA_MODE_RECEIVER;
uint8_t  rxBuffer_l[64]  = {0};
uint32_t rxPacketCount_l = 0;

/* Wireless timeout — if no packet arrives within this window the link
 * is considered lost and g_loraConnected / g_wirelessDataValid reset  */
#define WIRELESS_TIMEOUT_MS   60000UL
#define ACK_TX_TIMEOUT_MS       300UL

/* ── Wireless data received from transmitter ─────────────────────── */
static uint8_t  g_wirelessTankLevel  = 0;   /* 0–100 %              */
static uint8_t  g_wirelessWellDry    = 0;   /* 0=OK  1=DRY alarm    */
static uint32_t g_wirelessLastRxTick = 0;   /* HAL_GetTick() stamp  */
static bool     g_wirelessDataValid  = false;

/* ── Public flags / counters ─────────────────────────────────────── */
uint8_t  g_loraConnected     = 0;   /* 0=down  1=up — read by screen/adc */
bool     g_loraNewPacketFlag = false; /* set each time a data pkt parsed  */

/* TX stat stubs — keeps the linker happy on the RX project */
uint32_t g_lora_tx_ok    = 0;
uint32_t g_lora_tx_retry = 0;
uint32_t g_lora_tx_fail  = 0;

/* ════════════════════════════════════════════════════════════════════
 *  PUBLIC GETTERS
 * ════════════════════════════════════════════════════════════════════ */

/* Returns last received tank level (0–100 %).
 * Only meaningful while LoRa_IsWirelessDataValid() == true.          */
uint8_t LoRa_GetWirelessTankLevel(void) { return g_wirelessTankLevel; }

/* Returns last received well-dry flag (0=OK, 1=DRY alarm).
 * adc.c uses this to synthesise CH5 when wireless data is valid.
 * Only meaningful while LoRa_IsWirelessDataValid() == true.          */
uint8_t LoRa_GetWirelessWellDry(void)   { return g_wirelessWellDry; }

/* Returns true while a valid packet arrived within WIRELESS_TIMEOUT_MS.
 * Side-effects on timeout:
 *   • g_wirelessDataValid  cleared
 *   • g_loraConnected      cleared
 *   • LED set to RED-BLINK to alert the user                         */
bool LoRa_IsWirelessDataValid(void)
{
    if (!g_wirelessDataValid) return false;

    if ((HAL_GetTick() - g_wirelessLastRxTick) > WIRELESS_TIMEOUT_MS)
    {
        /* ── Link timed out ────────────────────────────────────────── */
        g_wirelessDataValid = false;
        g_wirelessWellDry   = 0;    /* safe default — don't keep old alarm */

        if (g_loraConnected)
        {
            g_loraConnected = 0;
            /* Slow RED blink so the operator knows the TX is offline */
            LED_SetIntent(LED_COLOR_RED, LED_MODE_BLINK, 500);
        }
        return false;
    }
    return true;
}

/* ════════════════════════════════════════════════════════════════════
 *  NSS / FREQUENCY
 * ════════════════════════════════════════════════════════════════════ */
#define NSS_LOW()  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET)
#define NSS_HIGH() HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET)
#define LORA_FREQUENCY_HZ  433000000UL

/* ════════════════════════════════════════════════════════════════════
 *  SPI REGISTER ACCESS  (public — same on both nodes)
 * ════════════════════════════════════════════════════════════════════ */
void LoRa_WriteReg(uint8_t addr, uint8_t data)
{
    uint8_t buf[2] = { (uint8_t)(addr | 0x80), data };
    NSS_LOW(); HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY); NSS_HIGH();
}

uint8_t LoRa_ReadReg(uint8_t addr)
{
    uint8_t tx = addr & 0x7F, rx = 0;
    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive (&hspi1, &rx, 1, HAL_MAX_DELAY);
    NSS_HIGH();
    return rx;
}

void LoRa_WriteBuffer(uint8_t addr, const uint8_t *buffer, uint8_t size)
{
    uint8_t a = addr | 0x80;
    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &a,               1,    HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)buffer, size, HAL_MAX_DELAY);
    NSS_HIGH();
}

void LoRa_ReadBuffer(uint8_t addr, uint8_t *buffer, uint8_t size)
{
    uint8_t a = addr & 0x7F;
    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &a,     1,    HAL_MAX_DELAY);
    HAL_SPI_Receive (&hspi1, buffer, size, HAL_MAX_DELAY);
    NSS_HIGH();
}

/* ════════════════════════════════════════════════════════════════════
 *  INTERNAL HELPERS  (all static)
 * ════════════════════════════════════════════════════════════════════ */
static void LoRa_Reset(void)
{
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
}

static void LoRa_EnterRxContinuous(void)
{
    LoRa_WriteReg(0x12, 0xFF);   /* clear all IRQ flags */
    LoRa_WriteReg(0x01, 0x85);   /* RX-continuous mode  */
}

/* Poll DIO0 and read a pending packet from the FIFO.
 * Returns 0 if no packet is ready, otherwise the payload length.
 * rssi_out receives the packet RSSI in dBm.                          */
static uint8_t LoRa_ReceivePacket(uint8_t *buffer, int16_t *rssi_out)
{
    /* DIO0 goes high when RxDone IRQ is set */
    if (HAL_GPIO_ReadPin(LORA_DIO0_PORT, LORA_DIO0_PIN) == GPIO_PIN_RESET)
        return 0;

    uint8_t irq = LoRa_ReadReg(0x12);

    /* Discard packet with CRC error */
    if (irq & 0x20) { LoRa_WriteReg(0x12, 0xFF); return 0; }

    uint8_t len = LoRa_ReadReg(0x13);
    if (len == 0 || len > 63) { LoRa_WriteReg(0x12, 0xFF); return 0; }

    uint8_t fifoAddr = LoRa_ReadReg(0x10);
    LoRa_WriteReg(0x0D, fifoAddr);
    LoRa_ReadBuffer(0x00, buffer, len);

    if (rssi_out) *rssi_out = -157 + (int16_t)LoRa_ReadReg(0x1A);
    LoRa_WriteReg(0x12, 0xFF);   /* clear all IRQ flags */
    return len;
}

/* Send a short reply then immediately return to RX-Continuous mode.
 * Used for ACK and @OK# responses.                                   */
static void LoRa_SendReply(const char *pkt)
{
    uint8_t len = (uint8_t)strlen(pkt);
    if (len == 0) return;

    LoRa_WriteReg(0x01, 0x81);   /* standby */
    HAL_Delay(1);
    LoRa_WriteReg(0x0D, 0x00);
    LoRa_WriteBuffer(0x00, (const uint8_t *)pkt, len);
    LoRa_WriteReg(0x22, len);
    LoRa_WriteReg(0x12, 0xFF);
    LoRa_WriteReg(0x01, 0x83);   /* TX */

    uint32_t t0 = HAL_GetTick();
    while (!(LoRa_ReadReg(0x12) & 0x08))
        if ((HAL_GetTick() - t0) > ACK_TX_TIMEOUT_MS) break;

    LoRa_WriteReg(0x12, 0x08);   /* clear TX-done flag */
    LoRa_EnterRxContinuous();    /* back to RX */
}

static void LoRa_SendACK(uint32_t seq)
{
    char ack[24];
    snprintf(ack, sizeof(ack), "@ACK:%05lu#", (unsigned long)seq);
    LoRa_SendReply(ack);
}

/* ── Parse @TL:<level>[,WD:<wd>],SN:<seq># ─────────────────────────
 *
 *  Transmitter packet format (as built by LoRa_BuildPacket in TX):
 *    @TL:080,WD:0,SN:00003#
 *
 *  The WD field is parsed if present; if absent (older firmware on TX)
 *  *wd_out is left at 0 (safe default — no dry-run alarm).
 *
 *  Returns true only when level, SN and terminating '#' are all valid.
 * ──────────────────────────────────────────────────────────────────── */
static bool parse_data_packet(const char *pkt,
                               uint8_t   *level_out,
                               uint8_t   *wd_out,
                               uint32_t  *seq_out)
{
    /* Must start with @TL: */
    if (pkt[0] != '@' || pkt[1] != 'T' || pkt[2] != 'L' || pkt[3] != ':')
        return false;

    /* ── Parse tank level (0–100) ───────────────────────────────── */
    const char *p = pkt + 4;
    if (*p < '0' || *p > '9') return false;

    uint16_t val = 0;
    while (*p >= '0' && *p <= '9')
    {
        val = (uint16_t)(val * 10u + (uint16_t)(*p - '0'));
        if (val > 100) return false;  /* sanity: level can't exceed 100 */
        p++;
    }
    *level_out = (uint8_t)val;

    /* ── Parse optional WD (well-dry) field ─────────────────────── */
    *wd_out = 0;    /* safe default when TX firmware doesn't include WD */
    if (strncmp(p, ",WD:", 4) == 0)
    {
        p += 4;
        if (*p >= '0' && *p <= '9')
        {
            *wd_out = (uint8_t)(*p - '0');
            p++;
        }
    }

    /* ── Parse mandatory SN (sequence number) ───────────────────── */
    const char *sn = strstr(p, ",SN:");
    if (!sn) return false;
    sn += 4;

    if (*sn < '0' || *sn > '9') return false;
    uint32_t s = 0;
    while (*sn >= '0' && *sn <= '9')
    {
        s = s * 10u + (uint32_t)(*sn - '0');
        sn++;
    }
    if (*sn != '#') return false;   /* packet must end with '#' */

    *seq_out = s;
    return true;
}

/* ════════════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ════════════════════════════════════════════════════════════════════ */

/* Configure LoRa in RX-Continuous mode.
 * Must be called once during system init (before the main loop).     */
void LoRa_Init(void)
{
    LoRa_Reset();

    /* Read version register as a basic SPI self-test; result unused  */
    uint8_t ver = LoRa_ReadReg(0x42);
    (void)ver;

    LoRa_WriteReg(0x01, 0x80);   /* LoRa sleep */
    HAL_Delay(10);

    /* Set carrier frequency to 433 MHz */
    uint64_t frf = ((uint64_t)LORA_FREQUENCY_HZ << 19) / 32000000ULL;
    LoRa_WriteReg(0x06, (uint8_t)(frf >> 16));
    LoRa_WriteReg(0x07, (uint8_t)(frf >>  8));
    LoRa_WriteReg(0x08, (uint8_t)(frf      ));

    LoRa_WriteReg(0x0E, 0x00);   /* TX FIFO base addr */
    LoRa_WriteReg(0x0F, 0x00);   /* RX FIFO base addr */
    LoRa_WriteReg(0x09, 0x8F);   /* PA_BOOST max power */
    LoRa_WriteReg(0x0C, 0x23);   /* LNA max gain + boost */
    LoRa_WriteReg(0x4D, 0x87);   /* PA DAGC on */

    /* ── Modem config — MUST match transmitter lora.c exactly ────── */
    LoRa_WriteReg(0x1D, 0x72);   /* BW=125 kHz | CR=4/5 | ExplicitHdr */
    LoRa_WriteReg(0x1E, 0x74);   /* SF=7 | CRC=ON                      */
    LoRa_WriteReg(0x26, 0x04);   /* LowDataRateOptimize=OFF             */
    LoRa_WriteReg(0x39, 0x12);   /* SyncWord = 0x12 (public network)    */

    /* Start receiving immediately */
    LoRa_EnterRxContinuous();

    /* Purple steady = initialised, waiting for first packet */
    LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_STEADY, 1);
}

/* ── LoRa_Task — call every ~10 ms (non-blocking) ──────────────────
 *
 * Checks whether the 60 s watchdog has fired (link timeout), then
 * polls DIO0.  If a packet is waiting in the FIFO it is read,
 * classified, and:
 *   • @HI#       → reply @OK#  (handshake — no state change)
 *   • valid data → update g_wirelessTankLevel, g_wirelessWellDry,
 *                  g_wirelessLastRxTick, g_wirelessDataValid,
 *                  g_loraConnected, g_loraNewPacketFlag
 *                  then send ACK
 *   • unknown    → silently discarded
 *
 * All fields that adc.c and model_handle.c rely on are valid by
 * the time LoRa_Task() returns.
 * ──────────────────────────────────────────────────────────────────── */
void LoRa_Task(void)
{
    if (loraMode != LORA_MODE_RECEIVER) return;

    /* ── Watchdog: check timeout once per loop ──────────────────── */
    /* LoRa_IsWirelessDataValid() has the side-effect of clearing     */
    /* g_loraConnected and switching LED to RED-BLINK on timeout.     */
    if (g_loraConnected || g_wirelessDataValid)
        LoRa_IsWirelessDataValid();

    /* ── Poll for incoming packet ───────────────────────────────── */
    int16_t rssi = 0;
    uint8_t len  = LoRa_ReceivePacket(rxBuffer_l, &rssi);
    if (len == 0) return;   /* nothing to do */

    rxBuffer_l[len] = '\0';
    rxPacketCount_l++;

    /* ── HELLO handshake — TX probing for a live receiver ──────── */
    if (strcmp((char *)rxBuffer_l, "@HI#") == 0)
    {
        LoRa_SendReply("@OK#");
        /* Don't set g_loraConnected here; wait for a real data packet */
        return;
    }

    /* ── Data packet: @TL:080,WD:0,SN:00003# ───────────────────── */
    uint8_t  level = 0;
    uint8_t  wd    = 0;
    uint32_t seq   = 0;

    if (parse_data_packet((char *)rxBuffer_l, &level, &wd, &seq))
    {
        /* ── Store wireless values ────────────────────────────── */
        g_wirelessTankLevel  = level;
        g_wirelessWellDry    = wd;
        g_wirelessLastRxTick = HAL_GetTick();
        g_wirelessDataValid  = true;

        /* ── Update connection state ──────────────────────────── */
        if (!g_loraConnected)
        {
            g_loraConnected = 1;
            /* Blue blink briefly when first connecting;            */
            /* will settle to steady via LED_Task if supported.     */
            LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 120);
        }
        else
        {
            /* Heartbeat blink on every packet while connected */
            LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 120);
        }

        /* ── Notify main.c to refresh screen immediately ─────── */
        g_loraNewPacketFlag = true;

        /* ── ACK back to transmitter ──────────────────────────── */
        LoRa_SendACK(seq);
    }
    /* Unknown / malformed packets are silently discarded             */
}
