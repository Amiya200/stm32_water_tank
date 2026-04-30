/* ====================================================================
 * lora.c  —  RECEIVER LoRa driver
 *
 * FINAL DID-BASED PROTOCOL
 *
 * RX accepts:
 *   @TL:060,WD:0,DID:A1B2C3D4#
 *
 * RX replies:
 *   @ACK:A1B2C3D4#
 *
 * Pairing behaviour:
 *   - If pairing mode is ON:
 *       RX reads DID from normal water-level packet.
 *       RX stores DID in EEPROM using PairedDev_Add().
 *       RX sends ACK.
 *
 *   - If pairing mode is OFF:
 *       RX accepts only paired DID.
 *       Unknown DID gets @REJECT#.
 *
 * No SN / serial number is used anywhere.
 * ==================================================================== */

#include "lora.h"
#include "device_id.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "led.h"

extern SPI_HandleTypeDef hspi1;

/* Public globals */
uint8_t  loraMode        = LORA_MODE_RECEIVER;
uint8_t  rxBuffer_l[96]  = {0};
uint32_t rxPacketCount_l = 0;

uint8_t  g_loraConnected     = 0;
bool     g_loraNewPacketFlag = false;

uint32_t g_lora_tx_ok        = 0;
uint32_t g_lora_tx_retry     = 0;
uint32_t g_lora_tx_fail      = 0;

/* Private constants */
#define LORA_BUF_SIZE        96U
#define WIRELESS_TIMEOUT_MS  60000UL
#define ACK_TX_TIMEOUT_MS      300UL
#define PAIRING_TIMEOUT_MS   30000UL

/* Wireless data state */
static uint8_t  g_wirelessTankLevel  = 0;
static uint8_t  g_wirelessWellDry    = 0;
static uint32_t g_wirelessLastRxTick = 0;
static bool     g_wirelessDataValid  = false;

/* Pairing state */
static bool     s_pairingMode     = false;
static uint32_t s_pairingDeadline = 0;
static bool     s_pairingDone     = false;
static uint32_t s_lastPairedDID   = 0;

#define NSS_LOW()   HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET)
#define NSS_HIGH()  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET)

/* ====================================================================
 * SPI PRIMITIVES
 * ==================================================================== */

void LoRa_WriteReg(uint8_t addr, uint8_t data)
{
    uint8_t buf[2] = { (uint8_t)(addr | 0x80), data };

    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
    NSS_HIGH();
}

uint8_t LoRa_ReadReg(uint8_t addr)
{
    uint8_t tx = (uint8_t)(addr & 0x7F);
    uint8_t rx = 0;

    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &rx, 1, HAL_MAX_DELAY);
    NSS_HIGH();

    return rx;
}

void LoRa_WriteBuffer(uint8_t addr, const uint8_t *buffer, uint8_t size)
{
    uint8_t a = (uint8_t)(addr | 0x80);

    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &a, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)buffer, size, HAL_MAX_DELAY);
    NSS_HIGH();
}

void LoRa_ReadBuffer(uint8_t addr, uint8_t *buffer, uint8_t size)
{
    uint8_t a = (uint8_t)(addr & 0x7F);

    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &a, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buffer, size, HAL_MAX_DELAY);
    NSS_HIGH();
}

/* ====================================================================
 * INTERNAL HELPERS
 * ==================================================================== */

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

static uint8_t LoRa_ReceivePacket(uint8_t *buffer, int16_t *rssi_out)
{
    if (HAL_GPIO_ReadPin(LORA_DIO0_PORT, LORA_DIO0_PIN) == GPIO_PIN_RESET)
        return 0;

    uint8_t irq = LoRa_ReadReg(0x12);
    LoRa_WriteReg(0x12, 0xFF);

    if (irq & 0x20)
        return 0;

    if ((irq & 0x40) == 0)
        return 0;

    uint8_t len = LoRa_ReadReg(0x13);

    if (len == 0 || len >= (LORA_BUF_SIZE - 1))
        return 0;

    uint8_t fifoAddr = LoRa_ReadReg(0x10);
    LoRa_WriteReg(0x0D, fifoAddr);
    LoRa_ReadBuffer(0x00, buffer, len);
    buffer[len] = '\0';

    if (rssi_out != NULL)
        *rssi_out = (int16_t)(-157 + (int16_t)LoRa_ReadReg(0x1A));

    return len;
}

static void LoRa_SendReply(const char *pkt)
{
    if (pkt == NULL)
        return;

    uint8_t len = (uint8_t)strlen(pkt);

    if (len == 0 || len >= LORA_BUF_SIZE)
        return;

    LoRa_WriteReg(0x01, 0x81);
    HAL_Delay(1);

    LoRa_WriteReg(0x0D, 0x00);
    LoRa_WriteBuffer(0x00, (const uint8_t *)pkt, len);
    LoRa_WriteReg(0x22, len);
    LoRa_WriteReg(0x12, 0xFF);
    LoRa_WriteReg(0x01, 0x83);

    uint32_t t0 = HAL_GetTick();

    while ((LoRa_ReadReg(0x12) & 0x08) == 0)
    {
        if ((HAL_GetTick() - t0) > ACK_TX_TIMEOUT_MS)
            break;
    }

    LoRa_WriteReg(0x12, 0x08);
    LoRa_EnterRxContinuous();
}

static void LoRa_SendACK(uint32_t did)
{
    char ack[24];

    snprintf(ack,
             sizeof(ack),
             "@ACK:%08lX#",
             (unsigned long)did);

    LoRa_SendReply(ack);
}

static bool parse_hex8(const char *p, uint32_t *out)
{
    uint32_t value = 0;

    if (p == NULL || out == NULL)
        return false;

    for (int i = 0; i < 8; i++)
    {
        char c = p[i];
        uint8_t n;

        if      (c >= '0' && c <= '9') n = (uint8_t)(c - '0');
        else if (c >= 'A' && c <= 'F') n = (uint8_t)(c - 'A' + 10);
        else if (c >= 'a' && c <= 'f') n = (uint8_t)(c - 'a' + 10);
        else return false;

        value = (value << 4) | n;
    }

    *out = value;
    return true;
}

/* Optional legacy hello support */
static bool parse_hello_packet(const char *pkt, uint32_t *did_out)
{
    if (pkt == NULL || did_out == NULL)
        return false;

    *did_out = 0;

    if (strncmp(pkt, "@HI:", 4) != 0)
        return false;

    const char *p = pkt + 4;

    if (strlen(p) < 9 || p[8] != '#')
        return false;

    if (!parse_hex8(p, did_out))
        return false;

    if (*did_out == 0 || *did_out == 0xFFFFFFFFUL)
        return false;

    return true;
}

/* Final parser:
 *   @TL:060,WD:0,DID:A1B2C3D4#
 */
static bool parse_data_packet(const char *pkt,
                              uint8_t *level_out,
                              uint8_t *wd_out,
                              uint32_t *did_out)
{
    const char *p;
    uint16_t level = 0;
    uint8_t wd = 0;
    uint32_t did = 0;

    if (pkt == NULL || level_out == NULL || wd_out == NULL || did_out == NULL)
        return false;

    *level_out = 0;
    *wd_out = 0;
    *did_out = 0;

    if (strncmp(pkt, "@TL:", 4) != 0)
        return false;

    p = pkt + 4;

    if (*p < '0' || *p > '9')
        return false;

    while (*p >= '0' && *p <= '9')
    {
        level = (uint16_t)((level * 10U) + (uint16_t)(*p - '0'));

        if (level > 100U)
            return false;

        p++;
    }

    if (strncmp(p, ",WD:", 4) != 0)
        return false;

    p += 4;

    if (*p != '0' && *p != '1')
        return false;

    wd = (uint8_t)(*p - '0');
    p++;

    if (strncmp(p, ",DID:", 5) != 0)
        return false;

    p += 5;

    if (!parse_hex8(p, &did))
        return false;

    if (p[8] != '#')
        return false;

    if (did == 0 || did == 0xFFFFFFFFUL)
        return false;

    *level_out = (uint8_t)level;
    *wd_out = wd;
    *did_out = did;

    return true;
}

static void accept_wireless_data(uint8_t level, uint8_t wd)
{
    g_wirelessTankLevel  = level;
    g_wirelessWellDry    = wd;
    g_wirelessLastRxTick = HAL_GetTick();
    g_wirelessDataValid  = true;
    g_loraConnected      = 1;
    g_loraNewPacketFlag  = true;
}

static void pair_device_from_did(uint32_t did)
{
    if (did == 0 || did == 0xFFFFFFFFUL)
        return;

    /* Use Clear if you want only one transmitter at a time.
     * Comment this line if you want multiple paired transmitters.
     */
    PairedDev_Clear();

    PairedDev_Add(did);

    s_lastPairedDID = did;
    s_pairingDone   = true;
    s_pairingMode   = false;
}

/* ====================================================================
 * PUBLIC PAIRING API
 * ==================================================================== */

void LoRa_EnterPairingMode(void)
{
    s_pairingMode     = true;
    s_pairingDone     = false;
    s_lastPairedDID   = 0;
    s_pairingDeadline = HAL_GetTick() + PAIRING_TIMEOUT_MS;
}

void LoRa_ExitPairingMode(void)
{
    s_pairingMode = false;
}

bool LoRa_IsPairingMode(void)
{
    return s_pairingMode;
}

bool LoRa_IsPairingComplete(void)
{
    return s_pairingDone;
}

uint32_t LoRa_GetLastPairedDID(void)
{
    return s_lastPairedDID;
}

/* ====================================================================
 * PUBLIC WIRELESS DATA API
 * ==================================================================== */

uint8_t LoRa_GetWirelessTankLevel(void)
{
    return g_wirelessTankLevel;
}

uint8_t LoRa_GetWirelessWellDry(void)
{
    return g_wirelessWellDry;
}

bool LoRa_IsWirelessDataValid(void)
{
    if (!g_wirelessDataValid)
        return false;

    if ((HAL_GetTick() - g_wirelessLastRxTick) > WIRELESS_TIMEOUT_MS)
    {
        g_wirelessDataValid = false;
        g_wirelessWellDry = 0;

        if (g_loraConnected)
        {
            g_loraConnected = 0;
            LED_SetIntent(LED_COLOR_RED, LED_MODE_BLINK, 500);
        }

        return false;
    }

    return true;
}

/* ====================================================================
 * INIT + TASK
 * ==================================================================== */

void LoRa_Init(void)
{
    DeviceID_Init();
    PairedDev_Init();

    LoRa_Reset();

    (void)LoRa_ReadReg(0x42);

    LoRa_WriteReg(0x01, 0x80);
    HAL_Delay(10);

    /* Frequency: 433 MHz */
    uint64_t frf = ((uint64_t)433000000ULL << 19) / 32000000ULL;
    LoRa_WriteReg(0x06, (uint8_t)(frf >> 16));
    LoRa_WriteReg(0x07, (uint8_t)(frf >> 8));
    LoRa_WriteReg(0x08, (uint8_t)(frf));

    LoRa_WriteReg(0x0E, 0x00);
    LoRa_WriteReg(0x0F, 0x00);

    LoRa_WriteReg(0x09, 0x8F);
    LoRa_WriteReg(0x0C, 0x23);
    LoRa_WriteReg(0x4D, 0x87);

    LoRa_WriteReg(0x1D, 0x72);
    LoRa_WriteReg(0x1E, 0x74);  /* SF7 + CRC ON */
    LoRa_WriteReg(0x26, 0x04);
    LoRa_WriteReg(0x39, 0x12);

    LoRa_EnterRxContinuous();

    LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_STEADY, 1);
}

void LoRa_Task(void)
{
    if (loraMode != LORA_MODE_RECEIVER)
        return;

    if (g_loraConnected || g_wirelessDataValid)
        (void)LoRa_IsWirelessDataValid();

    if (s_pairingMode && HAL_GetTick() >= s_pairingDeadline)
        s_pairingMode = false;

    int16_t rssi = 0;
    uint8_t len = LoRa_ReceivePacket(rxBuffer_l, &rssi);

    if (len == 0)
        return;

    rxBuffer_l[len] = '\0';
    rxPacketCount_l++;

    /* Optional HELLO handling */
    uint32_t helloDID = 0;
    if (parse_hello_packet((char *)rxBuffer_l, &helloDID))
    {
        if (s_pairingMode)
        {
            pair_device_from_did(helloDID);
            LoRa_SendACK(helloDID);
            LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 120);
        }
        else if (PairedDev_IsAllowed(helloDID))
        {
            LoRa_SendACK(helloDID);
        }
        else
        {
            LoRa_SendReply("@REJECT#");
        }

        return;
    }

    /* Main DATA packet handling */
    uint8_t level = 0;
    uint8_t wd    = 0;
    uint32_t did  = 0;

    if (!parse_data_packet((char *)rxBuffer_l, &level, &wd, &did))
        return;

    /* Pair directly from water-level packet */
    if (s_pairingMode)
    {
        pair_device_from_did(did);
        accept_wireless_data(level, wd);
        LoRa_SendACK(did);
        LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 120);
        return;
    }

    /* Normal mode: validate paired transmitter */
    if (!PairedDev_IsAllowed(did))
    {
        LoRa_SendReply("@REJECT#");
        return;
    }

    accept_wireless_data(level, wd);
    LoRa_SendACK(did);

    LED_SetIntent(LED_COLOR_GREEN, LED_MODE_STEADY, 1);

    (void)rssi;
}
