/* ====================================================================
 * lora.c  —  RECEIVER  (motor-controller node)
 *
 * Packet protocol (both nodes must match exactly):
 *
 *   TX → RX   @HI#                          startup hello
 *   RX → TX   @OK#                          hello reply
 *   TX → RX   @TL:<3d>,SN:<5d>#            data  e.g. @TL:080,SN:00003#
 *   RX → TX   @ACK:<5d>#                   ack   e.g. @ACK:00003#
 *
 * Connection state
 *   g_loraConnected = 1  when a valid data packet arrived recently
 *   g_loraConnected = 0  after WIRELESS_TIMEOUT_MS (60 s) of silence
 *                        OR immediately on power-up until first packet
 *
 * Auto-reconnect
 *   If the transmitter goes offline and comes back, the first valid
 *   @TL:...,SN:...# packet immediately sets g_loraConnected = 1 again.
 *   No manual reset, no reboot needed on either side.
 *
 * Modem settings  (MUST match transmitter lora.c exactly)
 *   Frequency  : 433 MHz
 *   BW         : 125 kHz
 *   SF         : 7
 *   CR         : 4/5
 *   CRC        : ON  (0x1E = 0x74)
 *   Sync word  : 0x12 (public)
 *   Header     : Explicit
 * ==================================================================== */

#include "lora.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "led.h"

/* ── UART helpers — implemented in main.c ──────────────────────────── */
extern void UART_Print  (const char *s);
//extern void UART_PrintLn(const char *s);

/* ── SPI handle — configured in main.c / MX_SPI1_Init() ────────────── */
extern SPI_HandleTypeDef hspi1;

/* ════════════════════════════════════════════════════════════════════
 *  MODULE-LEVEL STATE
 * ════════════════════════════════════════════════════════════════════ */

/* Runtime mode flag */
uint8_t  loraMode        = LORA_MODE_RECEIVER;

/* Shared RX buffer and packet counter (extern'd in lora.h) */
uint8_t  rxBuffer_l[64]  = {0};
uint32_t rxPacketCount_l = 0;

/* Wireless tank level state */
#define WIRELESS_TIMEOUT_MS   60000UL   /* silence threshold → disconnect      */
#define ACK_TX_TIMEOUT_MS       300UL   /* max wait for TxDone when sending ACK */

static uint8_t  g_wirelessTankLevel  = 0;
static uint32_t g_wirelessLastRxTick = 0;
static bool     g_wirelessDataValid  = false;

/* Shared connection flag (extern'd in lora.h; same variable on TX side) */
uint8_t g_loraConnected = 0;

/* TX statistics stubs — defined on TX side only; externs keep linker happy */
uint32_t g_lora_tx_ok    = 0;
uint32_t g_lora_tx_retry = 0;
uint32_t g_lora_tx_fail  = 0;

/* ════════════════════════════════════════════════════════════════════
 *  PUBLIC GETTERS
 * ════════════════════════════════════════════════════════════════════ */

uint8_t LoRa_GetWirelessTankLevel(void)
{
    return g_wirelessTankLevel;
}

bool LoRa_IsWirelessDataValid(void)
{
    if (!g_wirelessDataValid) return false;

    if ((HAL_GetTick() - g_wirelessLastRxTick) > WIRELESS_TIMEOUT_MS)
    {
        g_wirelessDataValid = false;
        if (g_loraConnected)
        {
            g_loraConnected = 0;
//            UART_PrintLn("[LORA-RX] Link TIMEOUT — no packet for 60 s  connected=0");
            LED_SetIntent(LED_COLOR_RED, LED_MODE_BLINK, 500);
        }
        return false;
    }
    return true;
}

/* ════════════════════════════════════════════════════════════════════
 *  NSS / FREQUENCY CONSTANTS
 * ════════════════════════════════════════════════════════════════════ */

#define NSS_LOW()  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET)
#define NSS_HIGH() HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET)

#define LORA_FREQUENCY_HZ   433000000UL

/* ════════════════════════════════════════════════════════════════════
 *  SPI REGISTER ACCESS  (public — declared in lora.h)
 * ════════════════════════════════════════════════════════════════════ */

void LoRa_WriteReg(uint8_t addr, uint8_t data)
{
    uint8_t buf[2] = { (uint8_t)(addr | 0x80), data };
    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
    NSS_HIGH();
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
 *  INTERNAL HELPERS  (all static — not visible outside this file)
 * ════════════════════════════════════════════════════════════════════ */

/* ── Hardware reset pulse ───────────────────────────────────────────── */
static void LoRa_Reset(void)
{
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
}

/* ── Enter RX-Continuous mode ───────────────────────────────────────── */
static void LoRa_EnterRxContinuous(void)
{
    LoRa_WriteReg(0x12, 0xFF);   /* clear all IRQ flags  */
    LoRa_WriteReg(0x01, 0x85);   /* mode = RX_CONT       */
}

/* ── Poll DIO0 and read one packet (non-blocking) ───────────────────── */
static uint8_t LoRa_ReceivePacket(uint8_t *buffer, int16_t *rssi)
{
    /* DIO0 = RxDone.  Low → nothing yet. */
    if (HAL_GPIO_ReadPin(LORA_DIO0_PORT, LORA_DIO0_PIN) == GPIO_PIN_RESET)
        return 0;

    uint8_t irq = LoRa_ReadReg(0x12);

    /* CRC error — clear and bail */
    if (irq & 0x20)
    {
        LoRa_WriteReg(0x12, 0xFF);
//        UART_PrintLn("[LORA-RX] CRC error — packet discarded");
        return 0;
    }

    /* Payload length sanity */
    uint8_t len = LoRa_ReadReg(0x13);
    if (len == 0 || len > 63)
    {
        LoRa_WriteReg(0x12, 0xFF);
        return 0;
    }

    /* Read payload from FIFO */
    uint8_t fifoAddr = LoRa_ReadReg(0x10);   /* RegFifoRxCurrentAddr */
    LoRa_WriteReg(0x0D, fifoAddr);
    LoRa_ReadBuffer(0x00, buffer, len);

    /* RSSI */
    *rssi = -157 + (int16_t)LoRa_ReadReg(0x1A);

    LoRa_WriteReg(0x12, 0xFF);   /* clear all IRQ flags */
    return len;
}

/* ── Send a short reply packet then return to RX-Continuous ─────────── *
 * Used for both @OK# (hello reply) and @ACK:<seq># (data ACK).         *
 * The entire TX+return-to-RX takes < 150 ms at SF7/BW125.              *
 * ──────────────────────────────────────────────────────────────────── */
static void LoRa_SendReply(const char *pkt)
{
    uint8_t len = (uint8_t)strlen(pkt);
    if (len == 0) return;

    /* 1. Standby */
    LoRa_WriteReg(0x01, 0x81);
    HAL_Delay(1);

    /* 2. Load FIFO */
    LoRa_WriteReg(0x0D, 0x00);
    LoRa_WriteBuffer(0x00, (const uint8_t *)pkt, len);
    LoRa_WriteReg(0x22, len);    /* RegPayloadLength  */
    LoRa_WriteReg(0x12, 0xFF);   /* clear IRQ flags   */

    /* 3. TX mode */
    LoRa_WriteReg(0x01, 0x83);

    /* 4. Poll TxDone (bit 3) */
    uint32_t t0 = HAL_GetTick();
    while (!(LoRa_ReadReg(0x12) & 0x08))
    {
        if ((HAL_GetTick() - t0) > ACK_TX_TIMEOUT_MS) break;
    }
    LoRa_WriteReg(0x12, 0x08);   /* clear TxDone flag */

    /* 5. Back to RX-Continuous */
    LoRa_EnterRxContinuous();
}

/* ── Build and send @ACK:<5d># ──────────────────────────────────────── */
static void LoRa_SendACK(uint32_t seq)
{
    char ack[24];
    snprintf(ack, sizeof(ack), "@ACK:%05lu#", (unsigned long)seq);

    char log[48];
    snprintf(log, sizeof(log), "[LORA-RX] TX ACK \"%s\"", ack);
//    UART_PrintLn(log);

    LoRa_SendReply(ack);
}

/* ── Parse @TL:<level>[,<anything>],SN:<seq># ───────────────────────── *
 *                                                                        *
 * Accepts any extra comma-separated fields between TL and SN            *
 * (e.g. WD:0) so the transmitter can evolve its packet freely.          *
 * ──────────────────────────────────────────────────────────────────── */
static bool parse_data_packet(const char *pkt,
                               uint8_t   *level_out,
                               uint32_t  *seq_out)
{
    /* Must start with @TL: */
    if (pkt[0] != '@' || pkt[1] != 'T' || pkt[2] != 'L' || pkt[3] != ':')
        return false;

    /* Parse level digits (0-100) */
    const char *p   = pkt + 4;
    uint16_t    val = 0;
    if (*p < '0' || *p > '9') return false;

    while (*p >= '0' && *p <= '9')
    {
        val = (uint16_t)(val * 10u + (uint16_t)(*p - '0'));
        if (val > 100) return false;
        p++;
    }
    *level_out = (uint8_t)val;

    /* Find ",SN:" anywhere after the level field */
    const char *sn = strstr(p, ",SN:");
    if (!sn) return false;
    sn += 4;   /* skip ",SN:" */

    if (*sn < '0' || *sn > '9') return false;

    uint32_t s = 0;
    while (*sn >= '0' && *sn <= '9')
    {
        s = s * 10u + (uint32_t)(*sn - '0');
        sn++;
    }

    if (*sn != '#') return false;   /* terminator required */
    *seq_out = s;
    return true;
}

/* ════════════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ════════════════════════════════════════════════════════════════════ */

/* ── LoRa_Init — call once in main() after all HAL/GPIO inits ────────── */
void LoRa_Init(void)
{
    char buf[72];

    LoRa_Reset();
//    UART_PrintLn("[LORA-RX] Init: starting SX127x …");

    /* Verify SPI comms — version register must read 0x12 */
    uint8_t ver = LoRa_ReadReg(0x42);
    snprintf(buf, sizeof(buf),
             "[LORA-RX] Init: chip version=0x%02X (expect 0x12)", ver);
//    UART_PrintLn(buf);
    if (ver != 0x12)
//        UART_PrintLn("[LORA-RX] WARNING: unexpected version — check SPI/NSS!");

    /* LoRa sleep mode (required before changing most registers) */
    LoRa_WriteReg(0x01, 0x80);
    HAL_Delay(10);

    /* Frequency: 433 MHz */
    uint64_t frf = ((uint64_t)LORA_FREQUENCY_HZ << 19) / 32000000ULL;
    LoRa_WriteReg(0x06, (uint8_t)(frf >> 16));
    LoRa_WriteReg(0x07, (uint8_t)(frf >>  8));
    LoRa_WriteReg(0x08, (uint8_t)(frf      ));

    /* FIFO base addresses */
    LoRa_WriteReg(0x0E, 0x00);   /* RegFifoTxBaseAddr = 0  */
    LoRa_WriteReg(0x0F, 0x00);   /* RegFifoRxBaseAddr = 0  */

    /* RF front-end */
    LoRa_WriteReg(0x09, 0x8F);   /* PA_BOOST, MaxPower=7, OutputPower=15 */
    LoRa_WriteReg(0x0C, 0x23);   /* LNA: highest gain, LNA boost ON       */
    LoRa_WriteReg(0x4D, 0x87);   /* RegPaDac: PA DAGC ON                  */

    /* ── Modem config — MUST match transmitter exactly ─────────────── */
    LoRa_WriteReg(0x1D, 0x72);   /* BW=125 kHz | CR=4/5 | ExplicitHeader  */
    LoRa_WriteReg(0x1E, 0x74);   /* SF=7 | CRC=ON                          */
    LoRa_WriteReg(0x26, 0x04);   /* LowDataRateOptimize=OFF                 */
    LoRa_WriteReg(0x39, 0x12);   /* SyncWord = 0x12 (public LoRa)           */

//    UART_PrintLn("[LORA-RX] Init: BW=125kHz SF=7 CR=4/5 CRC=ON sync=0x12");

    /* Start listening */
    LoRa_EnterRxContinuous();

//    UART_PrintLn("[LORA-RX] Init: RX-Continuous — waiting for transmitter …");
    LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_STEADY, 1);
}

/* ── LoRa_Task — call every loop iteration (every ~10 ms) ────────────── *
 *                                                                         *
 * Non-blocking.  Returns immediately when no packet is waiting.          *
 * On a valid @TL:...,SN:...# packet:                                     *
 *   1. Updates g_wirelessTankLevel and connection state.                 *
 *   2. Sends @ACK:<seq># (RX briefly becomes TX ~100 ms, then reverts).  *
 * On @HI# hello: replies @OK#.                                           *
 * Connection auto-drops after 60 s of silence; auto-restores on the      *
 * very next valid packet, with no reboot needed on either side.          *
 * ──────────────────────────────────────────────────────────────────── */
void LoRa_Task(void)
{
    if (loraMode != LORA_MODE_RECEIVER) return;

    /* ── Periodic connection timeout check ──────────────────────────── */
    /* Call even when no packet arrives so the 60 s watchdog ticks.     */
    if (g_loraConnected)
        LoRa_IsWirelessDataValid();   /* clears g_loraConnected on timeout */

    /* ── Poll for incoming packet ───────────────────────────────────── */
    int16_t rssi = 0;
    uint8_t len  = LoRa_ReceivePacket(rxBuffer_l, &rssi);

    if (len == 0) return;            /* nothing this cycle — done */

    rxBuffer_l[len] = '\0';          /* NUL-terminate for string operations */
    rxPacketCount_l++;

    char log[80];
    snprintf(log, sizeof(log),
             "[LORA-RX] pkt #%lu  \"%s\"  RSSI=%d dBm  (%u B)",
             (unsigned long)rxPacketCount_l,
             (char *)rxBuffer_l, (int)rssi, len);
//    UART_PrintLn(log);

    /* ── HELLO handshake ────────────────────────────────────────────── */
    if (strcmp((char *)rxBuffer_l, "@HI#") == 0)
    {
//        UART_PrintLn("[LORA-RX] Got @HI# — sending @OK#");
        LoRa_SendReply("@OK#");
        return;
    }

    /* ── Data packet ────────────────────────────────────────────────── */
    uint8_t  level = 0;
    uint32_t seq   = 0;

    if (parse_data_packet((char *)rxBuffer_l, &level, &seq))
    {
        bool wasDisconnected = (g_loraConnected == 0);

        g_wirelessTankLevel  = level;
        g_wirelessLastRxTick = HAL_GetTick();
        g_wirelessDataValid  = true;
        g_loraConnected      = 1;     /* ← link confirmed (or restored)    */

        if (wasDisconnected)
//            UART_PrintLn("[LORA-RX] Link RESTORED — transmitter is back online");

        snprintf(log, sizeof(log),
                 "[LORA-RX] Data OK: level=%u%%  seq=%05lu  connected=1",
                 level, (unsigned long)seq);
//        UART_PrintLn(log);

        LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 120);

        /* Send ACK — RX briefly becomes TX then returns to RX-CONT */
        LoRa_SendACK(seq);
    }
    else
    {
        snprintf(log, sizeof(log),
                 "[LORA-RX] Unknown / malformed packet ignored: \"%s\"",
                 (char *)rxBuffer_l);
//        UART_PrintLn(log);
    }
}
