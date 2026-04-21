/* ====================================================================
 * lora.c  —  RECEIVER (motor-controller node)
 *
 * What changed vs the old version:
 *
 * 1. LoRa_Task() now parses incoming @TL:<percent># packets and stores
 *    the level in g_wirelessTankLevel with a timestamp.
 *
 * 2. Two new public functions expose the data to adc.c:
 *      LoRa_GetWirelessTankLevel()  — returns 0-100
 *      LoRa_IsWirelessDataValid()   — false if > 60 s since last packet
 *
 * 3. A lightweight string parser (parse_tank_level) is used instead
 *    of sscanf to keep flash/RAM usage minimal.
 *
 * Packet format expected from transmitter:
 *      @TL:<percent>#    e.g.  @TL:80#   @TL:0#   @TL:100#
 * ==================================================================== */

#include "lora.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "led.h"

/* ── Mode / globals ─────────────────────────────────────────────────── */
uint8_t  loraMode        = LORA_MODE_RECEIVER;
uint8_t  rxBuffer_l[64]  = {0};
uint32_t rxPacketCount_l = 0;

extern SPI_HandleTypeDef hspi1;

/* ── Wireless data state ────────────────────────────────────────────── */
#define WIRELESS_TIMEOUT_MS  60000UL   /* 60 s without a packet → invalid */

static uint8_t  g_wirelessTankLevel  = 0;
static uint32_t g_wirelessLastRxTick = 0;
static bool     g_wirelessDataValid  = false;

/* ── Public getters used by adc.c ───────────────────────────────────── */
uint8_t LoRa_GetWirelessTankLevel(void) { return g_wirelessTankLevel; }

bool LoRa_IsWirelessDataValid(void)
{
    if (!g_wirelessDataValid) return false;
    /* Auto-invalidate after timeout */
    if ((HAL_GetTick() - g_wirelessLastRxTick) > WIRELESS_TIMEOUT_MS)
    {
        g_wirelessDataValid = false;
        return false;
    }
    return true;
}

/* ── NSS macros ─────────────────────────────────────────────────────── */
#define NSS_LOW()  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET)
#define NSS_HIGH() HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET)

#define LORA_FREQUENCY 433000000UL

/* ── SPI helpers ────────────────────────────────────────────────────── */
void LoRa_WriteReg(uint8_t addr, uint8_t data)
{
    uint8_t buf[2] = { addr | 0x80, data };
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
    HAL_SPI_Transmit(&hspi1, &a,           1,    HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)buffer, size, HAL_MAX_DELAY);
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

/* ── Hardware reset ─────────────────────────────────────────────────── */
static void LoRa_Reset(void)
{
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
}

/* ── Init (RX continuous mode) ──────────────────────────────────────── */
void LoRa_Init(void)
{
    LoRa_Reset();

    LoRa_WriteReg(0x01, 0x80);            /* LoRa + Sleep              */
    HAL_Delay(10);

    uint64_t frf = ((uint64_t)LORA_FREQUENCY << 19) / 32000000ULL;
    LoRa_WriteReg(0x06, (uint8_t)(frf >> 16));
    LoRa_WriteReg(0x07, (uint8_t)(frf >>  8));
    LoRa_WriteReg(0x08, (uint8_t)(frf));

    LoRa_WriteReg(0x0E, 0x00);            /* TX FIFO base              */
    LoRa_WriteReg(0x0F, 0x00);            /* RX FIFO base              */
    LoRa_WriteReg(0x09, 0x8F);            /* PA config                 */
    LoRa_WriteReg(0x0C, 0x23);            /* LNA boost                 */
    LoRa_WriteReg(0x4D, 0x87);            /* PA DAGC                   */

    /* Modem config — must match transmitter exactly */
    LoRa_WriteReg(0x1D, 0x72);            /* BW125, CR4/5, explicit hdr*/
    LoRa_WriteReg(0x1E, 0x70);            /* SF7, CRC OFF              */
    LoRa_WriteReg(0x26, 0x04);            /* Low DR optimise OFF       */
    LoRa_WriteReg(0x39, 0x12);            /* Public sync word          */

    LoRa_WriteReg(0x12, 0xFF);            /* Clear IRQ flags           */
    LoRa_WriteReg(0x01, 0x85);            /* RX Continuous mode        */

    LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_STEADY, 1);
}

/* ── Receive a packet (non-blocking, polls DIO0) ────────────────────── */
uint8_t LoRa_ReceivePacket(uint8_t *buffer, int16_t *rssi)
{
    /* DIO0 goes high when RxDone IRQ fires */
    if (HAL_GPIO_ReadPin(LORA_DIO0_PORT, LORA_DIO0_PIN) == GPIO_PIN_RESET)
        return 0;

    uint8_t irq = LoRa_ReadReg(0x12);

    /* CRC error — discard */
    if (irq & 0x20)
    {
        LoRa_WriteReg(0x12, 0xFF);
        return 0;
    }

    uint8_t len      = LoRa_ReadReg(0x13);          /* Payload length  */
    uint8_t fifoAddr = LoRa_ReadReg(0x10);           /* Current RX addr */
    LoRa_WriteReg(0x0D, fifoAddr);
    LoRa_ReadBuffer(0x00, buffer, len);

    int16_t raw = (int16_t)LoRa_ReadReg(0x1A);
    *rssi = -157 + raw;

    LoRa_WriteReg(0x12, 0xFF);                       /* Clear IRQs      */
    return len;
}

/* ── Lightweight @TL:<level># parser ────────────────────────────────
 *
 * Returns the parsed level (0-100) on success, or 0xFF on any error.
 * Avoids sscanf to minimise code-size impact.
 * ──────────────────────────────────────────────────────────────────── */
static uint8_t parse_tank_level(const char *pkt)
{
    /* Expect exactly: @TL:<digits># */
    if (pkt[0] != '@' || pkt[1] != 'T' || pkt[2] != 'L' || pkt[3] != ':')
        return 0xFF;

    const char *p   = pkt + 4;
    uint16_t    val = 0;

    while (*p >= '0' && *p <= '9')
    {
        val = (uint16_t)(val * 10u + (uint16_t)(*p - '0'));
        if (val > 100) return 0xFF;   /* out of range */
        p++;
    }

    if (*p != '#') return 0xFF;       /* missing terminator */
    return (uint8_t)val;
}

/* ── Task — call every loop iteration from main() ───────────────────
 *
 * Polls for a received packet, parses it, and updates the wireless
 * tank level.  Must be called frequently (every 10–50 ms) so that
 * DIO0 is sampled quickly after a packet arrives.
 * ──────────────────────────────────────────────────────────────────── */
void LoRa_Task(void)
{
    if (loraMode != LORA_MODE_RECEIVER) return;

    int16_t rssi = 0;
    uint8_t len  = LoRa_ReceivePacket(rxBuffer_l, &rssi);

    if (len == 0) return;             /* No packet this cycle          */

    rxBuffer_l[len] = '\0';           /* NUL-terminate for parsing     */
    rxPacketCount_l++;

    uint8_t level = parse_tank_level((char*)rxBuffer_l);

    if (level != 0xFF)                /* Valid @TL:<n># packet         */
    {
        g_wirelessTankLevel  = level;
        g_wirelessLastRxTick = HAL_GetTick();
        g_wirelessDataValid  = true;
    }
    /* Unknown/malformed packets are silently ignored                  */

    LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 120);
}
