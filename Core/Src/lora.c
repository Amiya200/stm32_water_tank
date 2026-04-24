/* ====================================================================
 * lora.c  —  RECEIVER  (motor-controller node)
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

#define WIRELESS_TIMEOUT_MS   60000UL
#define ACK_TX_TIMEOUT_MS       300UL

static uint8_t  g_wirelessTankLevel  = 0;
static uint32_t g_wirelessLastRxTick = 0;
static bool     g_wirelessDataValid  = false;

uint8_t  g_loraConnected = 0;

/* TX stat stubs — keeps linker happy on RX project */
uint32_t g_lora_tx_ok    = 0;
uint32_t g_lora_tx_retry = 0;
uint32_t g_lora_tx_fail  = 0;

/* ════════════════════════════════════════════════════════════════════
 *  PUBLIC GETTERS
 * ════════════════════════════════════════════════════════════════════ */
uint8_t LoRa_GetWirelessTankLevel(void) { return g_wirelessTankLevel; }

bool LoRa_IsWirelessDataValid(void)
{
    if (!g_wirelessDataValid) return false;
    if ((HAL_GetTick() - g_wirelessLastRxTick) > WIRELESS_TIMEOUT_MS)
    {
        g_wirelessDataValid = false;
        if (g_loraConnected)
        {
            g_loraConnected = 0;
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
 *  SPI REGISTER ACCESS  (public)
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
    LoRa_WriteReg(0x12, 0xFF);
    LoRa_WriteReg(0x01, 0x85);
}

static uint8_t LoRa_ReceivePacket(uint8_t *buffer, int16_t *rssi)
{
    if (HAL_GPIO_ReadPin(LORA_DIO0_PORT, LORA_DIO0_PIN) == GPIO_PIN_RESET)
        return 0;

    uint8_t irq = LoRa_ReadReg(0x12);
    if (irq & 0x20) { LoRa_WriteReg(0x12, 0xFF); return 0; }  /* CRC error */

    uint8_t len = LoRa_ReadReg(0x13);
    if (len == 0 || len > 63) { LoRa_WriteReg(0x12, 0xFF); return 0; }

    uint8_t fifoAddr = LoRa_ReadReg(0x10);
    LoRa_WriteReg(0x0D, fifoAddr);
    LoRa_ReadBuffer(0x00, buffer, len);

    *rssi = -157 + (int16_t)LoRa_ReadReg(0x1A);
    LoRa_WriteReg(0x12, 0xFF);
    return len;
}

/* ── Send a short reply then return to RX-Continuous ───────────────── */
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

    LoRa_WriteReg(0x12, 0x08);
    LoRa_EnterRxContinuous();
}

static void LoRa_SendACK(uint32_t seq)
{
    char ack[24];
    snprintf(ack, sizeof(ack), "@ACK:%05lu#", (unsigned long)seq);
    LoRa_SendReply(ack);
}

/* ── Parse @TL:<level>[,...],SN:<seq># ─────────────────────────────── */
static bool parse_data_packet(const char *pkt,
                               uint8_t   *level_out,
                               uint32_t  *seq_out)
{
    if (pkt[0] != '@' || pkt[1] != 'T' || pkt[2] != 'L' || pkt[3] != ':')
        return false;

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

    const char *sn = strstr(p, ",SN:");
    if (!sn) return false;
    sn += 4;

    if (*sn < '0' || *sn > '9') return false;
    uint32_t s = 0;
    while (*sn >= '0' && *sn <= '9') { s = s * 10u + (uint32_t)(*sn - '0'); sn++; }
    if (*sn != '#') return false;
    *seq_out = s;
    return true;
}

/* ════════════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ════════════════════════════════════════════════════════════════════ */
void LoRa_Init(void)
{
    LoRa_Reset();

    uint8_t ver = LoRa_ReadReg(0x42);
    (void)ver;   /* SPI self-check; result unused without UART */

    LoRa_WriteReg(0x01, 0x80);   /* LoRa sleep */
    HAL_Delay(10);

    uint64_t frf = ((uint64_t)LORA_FREQUENCY_HZ << 19) / 32000000ULL;
    LoRa_WriteReg(0x06, (uint8_t)(frf >> 16));
    LoRa_WriteReg(0x07, (uint8_t)(frf >>  8));
    LoRa_WriteReg(0x08, (uint8_t)(frf      ));

    LoRa_WriteReg(0x0E, 0x00);   /* TX FIFO base */
    LoRa_WriteReg(0x0F, 0x00);   /* RX FIFO base */
    LoRa_WriteReg(0x09, 0x8F);   /* PA_BOOST max */
    LoRa_WriteReg(0x0C, 0x23);   /* LNA boost    */
    LoRa_WriteReg(0x4D, 0x87);   /* PA DAGC      */

    /* Modem config — MUST match transmitter exactly */
    LoRa_WriteReg(0x1D, 0x72);   /* BW=125 kHz | CR=4/5 | ExplicitHdr */
    LoRa_WriteReg(0x1E, 0x74);   /* SF=7 | CRC=ON                      */
    LoRa_WriteReg(0x26, 0x04);   /* LowDataRateOptimize=OFF             */
    LoRa_WriteReg(0x39, 0x12);   /* SyncWord = 0x12 (public)            */

    LoRa_EnterRxContinuous();
    LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_STEADY, 1);
}

void LoRa_Task(void)
{
    if (loraMode != LORA_MODE_RECEIVER) return;

    /* Watchdog — clears g_loraConnected on 60 s silence */
    if (g_loraConnected) LoRa_IsWirelessDataValid();

    int16_t rssi = 0;
    uint8_t len  = LoRa_ReceivePacket(rxBuffer_l, &rssi);
    if (len == 0) return;

    rxBuffer_l[len] = '\0';
    rxPacketCount_l++;

    /* HELLO handshake */
    if (strcmp((char *)rxBuffer_l, "@HI#") == 0)
    {
        LoRa_SendReply("@OK#");
        return;
    }

    /* Data packet */
    uint8_t  level = 0;
    uint32_t seq   = 0;

    if (parse_data_packet((char *)rxBuffer_l, &level, &seq))
    {
        g_wirelessTankLevel  = level;
        g_wirelessLastRxTick = HAL_GetTick();
        g_wirelessDataValid  = true;
        g_loraConnected      = 1;

        LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 120);
        LoRa_SendACK(seq);
    }
    /* Unknown / malformed packets silently ignored */
}
