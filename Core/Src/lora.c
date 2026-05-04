/* ====================================================================
 * lora.c  —  RECEIVER LoRa driver — WITH SEQUENCE NUMBER SUPPORT
 *
 * IMPROVEMENTS:
 *  • UART debugging for every packet received
 *  • LED feedback for connection status
 *  • Auto-reconnection logic
 *  • Better pairing flow
 *  • Packet statistics
 *  • NEW: Sequence number tracking and validation
 * ==================================================================== */

#include "lora.h"
#include "device_id.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "led.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

/* UART helper */
static void UART_Debug(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 1000);
}

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
#define WIRELESS_TIMEOUT_MS  60000UL  /* 60s timeout - can be adjusted */
#define ACK_TX_TIMEOUT_MS      300UL
#define PAIRING_TIMEOUT_MS   30000UL

/* Wireless data state */
static uint8_t  g_wirelessTankLevel  = 0;
static uint8_t  g_wirelessWellDry    = 0;
static uint32_t g_wirelessLastRxTick = 0;
static bool     g_wirelessDataValid  = false;

/* NEW: Sequence number tracking */
static uint32_t g_lastSequenceNum     = 0;
static bool     g_sequenceInitialized = false;
static uint32_t g_packetsLost         = 0;
static uint32_t g_duplicatePackets    = 0;
static uint32_t g_outOfOrderPackets   = 0;

/* Pairing state */
static bool     s_pairingMode     = false;
static uint32_t s_pairingDeadline = 0;
static bool     s_pairingDone     = false;
static uint32_t s_lastPairedDID   = 0;

/* Statistics */
static uint32_t s_totalPacketsRx     = 0;
static uint32_t s_validDataPackets   = 0;
static uint32_t s_invalidPackets     = 0;
static uint32_t s_rejectedPackets    = 0;
static uint32_t s_lastStatsReport    = 0;

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
    snprintf(ack, sizeof(ack), "@ACK:%08lX#", (unsigned long)did);

    char dbg[64];
    snprintf(dbg, sizeof(dbg), "[LORA RX] Sending ACK: %s", ack);
    UART_Debug(dbg);

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

/**
 * @brief Parse data packet with sequence number
 *
 * NEW FORMAT: @TL:060,WD:0,SQ:00001234,DID:A1B2C3D4#
 *
 * @param pkt       Input packet string
 * @param level_out Output: tank level 0-100%
 * @param wd_out    Output: well dry flag
 * @param seq_out   Output: sequence number
 * @param did_out   Output: device ID
 * @return true if parsing successful
 */
static bool parse_data_packet(const char *pkt,
                              uint8_t *level_out,
                              uint8_t *wd_out,
                              uint32_t *seq_out,
                              uint32_t *did_out)
{
    const char *p;
    uint16_t level = 0;
    uint8_t wd = 0;
    uint32_t seq = 0;
    uint32_t did = 0;

    if (pkt == NULL || level_out == NULL || wd_out == NULL ||
        seq_out == NULL || did_out == NULL)
        return false;

    *level_out = 0;
    *wd_out = 0;
    *seq_out = 0;
    *did_out = 0;

    /* Parse @TL: */
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

    /* Parse ,WD: */
    if (strncmp(p, ",WD:", 4) != 0)
        return false;

    p += 4;

    if (*p != '0' && *p != '1')
        return false;

    wd = (uint8_t)(*p - '0');
    p++;

    /* NEW: Parse ,SQ: */
    if (strncmp(p, ",SQ:", 4) != 0)
        return false;

    p += 4;

    if (!parse_hex8(p, &seq))
        return false;

    p += 8;

    /* Parse ,DID: */
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
    *seq_out = seq;
    *did_out = did;

    return true;
}

/**
 * @brief Validate and update sequence number
 *
 * Detects:
 *  - Packet loss (gaps in sequence)
 *  - Duplicate packets (same sequence received twice)
 *  - Out-of-order packets (sequence older than last received)
 *
 * @param newSeq New sequence number from received packet
 * @return true if sequence is valid (new and in order)
 */
static bool validate_sequence(uint32_t newSeq)
{
    char dbg[120];

    /* First packet - initialize */
    if (!g_sequenceInitialized)
    {
        g_lastSequenceNum = newSeq;
        g_sequenceInitialized = true;
        UART_Debug("[LORA RX] Sequence initialized");
        return true;
    }

    /* Check for duplicate */
    if (newSeq == g_lastSequenceNum)
    {
        g_duplicatePackets++;
        snprintf(dbg, sizeof(dbg),
                 "[LORA RX] !!! DUPLICATE packet - SEQ:%08lX (total duplicates: %lu)",
                 (unsigned long)newSeq, (unsigned long)g_duplicatePackets);
        UART_Debug(dbg);
        return false;  /* Reject duplicate */
    }

    /* Check for out-of-order (older packet) */
    if (newSeq < g_lastSequenceNum)
    {
        g_outOfOrderPackets++;
        snprintf(dbg, sizeof(dbg),
                 "[LORA RX] !!! OUT-OF-ORDER packet - SEQ:%08lX < Last:%08lX (total OOO: %lu)",
                 (unsigned long)newSeq, (unsigned long)g_lastSequenceNum,
                 (unsigned long)g_outOfOrderPackets);
        UART_Debug(dbg);
        return false;  /* Reject old packet */
    }

    /* Check for packet loss */
    uint32_t expected = g_lastSequenceNum + 1;
    if (newSeq > expected)
    {
        uint32_t lost = newSeq - expected;
        g_packetsLost += lost;
        snprintf(dbg, sizeof(dbg),
                 "[LORA RX] !!! PACKET LOSS detected - Gap: %lu packets (total lost: %lu)",
                 (unsigned long)lost, (unsigned long)g_packetsLost);
        UART_Debug(dbg);
        /* Still accept the packet, just note the loss */
    }

    /* Update last sequence */
    g_lastSequenceNum = newSeq;
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

    char dbg[80];
    snprintf(dbg, sizeof(dbg), "[LORA RX] Pairing device DID:%08lX", (unsigned long)did);
    UART_Debug(dbg);

    /* Clear previous pairings if you want only one TX at a time */
    PairedDev_Clear();

    PairedDev_Add(did);

    s_lastPairedDID = did;
    s_pairingDone   = true;
    s_pairingMode   = false;

    /* NEW: Reset sequence tracking on new pairing */
    g_sequenceInitialized = false;
    g_lastSequenceNum = 0;

    snprintf(dbg, sizeof(dbg), "[LORA RX] *** PAIRING COMPLETE *** DID:%08lX", (unsigned long)did);
    UART_Debug(dbg);
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

    UART_Debug("[LORA RX] *** PAIRING MODE ACTIVE - Waiting for TX ***");
    LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 200);
}

void LoRa_ExitPairingMode(void)
{
    s_pairingMode = false;
    UART_Debug("[LORA RX] Pairing mode exited");
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

/**
 * @brief Get last received sequence number
 */
uint32_t LoRa_GetLastSequence(void)
{
    return g_lastSequenceNum;
}

/**
 * @brief Get packet loss statistics
 */
uint32_t LoRa_GetPacketsLost(void)
{
    return g_packetsLost;
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
            UART_Debug("[LORA RX] !!! CONNECTION LOST - TIMEOUT !!!");
            LED_SetIntent(LED_COLOR_RED, LED_MODE_BLINK, 500);

            /* Reset sequence tracking on disconnect */
            g_sequenceInitialized = false;
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
    char dbg[120];

    UART_Debug("\r\n=========================================");
    UART_Debug("  HELONIX - RECEIVER LORA INIT");
    UART_Debug("=========================================");

    DeviceID_Init();
    PairedDev_Init();

    uint32_t myDID = DeviceID_GetOwn();
    char myDIDstr[9];
    DeviceID_GetHex(myDIDstr);

    snprintf(dbg, sizeof(dbg), "[LORA RX] My DID: %s (0x%08lX)", myDIDstr, (unsigned long)myDID);
    UART_Debug(dbg);

    uint8_t pairedCount = PairedDev_Count();
    snprintf(dbg, sizeof(dbg), "[LORA RX] Paired devices: %u/%u", pairedCount, (uint8_t)MAX_PAIRED);
    UART_Debug(dbg);

    for (uint8_t i = 0; i < pairedCount; i++)
    {
        uint32_t did = PairedDev_Get(i);
        snprintf(dbg, sizeof(dbg), "[LORA RX]   Paired #%u: DID:%08lX", i+1, (unsigned long)did);
        UART_Debug(dbg);
    }

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

    /* NEW: Initialize sequence tracking */
    g_sequenceInitialized = false;
    g_lastSequenceNum = 0;
    g_packetsLost = 0;
    g_duplicatePackets = 0;
    g_outOfOrderPackets = 0;

    UART_Debug("[LORA RX] Hardware initialized - RX CONTINUOUS mode");
    UART_Debug("[LORA RX] Sequence tracking enabled");
    UART_Debug("[LORA RX] Waiting for packets...\r\n");

    LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_STEADY, 1);
}

void LoRa_Task(void)
{
    if (loraMode != LORA_MODE_RECEIVER)
        return;

    uint32_t now = HAL_GetTick();

    /* Check connection status */
    if (g_loraConnected || g_wirelessDataValid)
        (void)LoRa_IsWirelessDataValid();

    /* Check pairing timeout */
    if (s_pairingMode && now >= s_pairingDeadline)
    {
        s_pairingMode = false;
        UART_Debug("[LORA RX] Pairing timeout - no TX found");
    }

    /* Periodic statistics */
    if ((now - s_lastStatsReport) >= 30000)  /* Every 30 seconds */
    {
        s_lastStatsReport = now;
        char dbg[160];
        snprintf(dbg, sizeof(dbg),
                 "[LORA RX] Stats: Total=%lu Valid=%lu Invalid=%lu Rejected=%lu",
                 (unsigned long)s_totalPacketsRx,
                 (unsigned long)s_validDataPackets,
                 (unsigned long)s_invalidPackets,
                 (unsigned long)s_rejectedPackets);
        UART_Debug(dbg);

        snprintf(dbg, sizeof(dbg),
                 "[LORA RX]        Lost=%lu Duplicate=%lu OutOfOrder=%lu LastSeq=%08lX",
                 (unsigned long)g_packetsLost,
                 (unsigned long)g_duplicatePackets,
                 (unsigned long)g_outOfOrderPackets,
                 (unsigned long)g_lastSequenceNum);
        UART_Debug(dbg);
    }

    int16_t rssi = 0;
    uint8_t len = LoRa_ReceivePacket(rxBuffer_l, &rssi);

    if (len == 0)
        return;

    rxBuffer_l[len] = '\0';
    rxPacketCount_l++;
    s_totalPacketsRx++;

    char dbg[160];
    snprintf(dbg, sizeof(dbg), "\r\n[LORA RX] <<< Packet #%lu | Len=%u | RSSI=%d dBm >>>",
             (unsigned long)rxPacketCount_l, len, rssi);
    UART_Debug(dbg);
    snprintf(dbg, sizeof(dbg), "[LORA RX] Raw: \"%s\"", (char*)rxBuffer_l);
    UART_Debug(dbg);

    /* Try HELLO packet first */
    uint32_t helloDID = 0;
    if (parse_hello_packet((char *)rxBuffer_l, &helloDID))
    {
        snprintf(dbg, sizeof(dbg), "[LORA RX] HELLO packet from DID:%08lX", (unsigned long)helloDID);
        UART_Debug(dbg);

        if (s_pairingMode)
        {
            UART_Debug("[LORA RX] *** PAIRING MODE - Accepting HELLO ***");
            pair_device_from_did(helloDID);
            LoRa_SendACK(helloDID);
            LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 120);
            s_validDataPackets++;
        }
        else if (PairedDev_IsAllowed(helloDID))
        {
            UART_Debug("[LORA RX] Known device - sending ACK");
            LoRa_SendACK(helloDID);
            s_validDataPackets++;
        }
        else
        {
            UART_Debug("[LORA RX] !!! REJECTED - Unknown device (not paired) !!!");
            LoRa_SendReply("@REJECT#");
            s_rejectedPackets++;
        }

        return;
    }

    /* Try DATA packet with sequence number */
    uint8_t level = 0;
    uint8_t wd    = 0;
    uint32_t seq  = 0;
    uint32_t did  = 0;

    if (!parse_data_packet((char *)rxBuffer_l, &level, &wd, &seq, &did))
    {
        snprintf(dbg, sizeof(dbg), "[LORA RX] !!! PARSE FAILED - Invalid packet format !!!");
        UART_Debug(dbg);
        s_invalidPackets++;
        return;
    }

    snprintf(dbg, sizeof(dbg), "[LORA RX] DATA: TL=%u%% WD=%u SEQ:%08lX DID:%08lX",
             level, wd, (unsigned long)seq, (unsigned long)did);
    UART_Debug(dbg);

    /* Pairing mode - accept any device */
    if (s_pairingMode)
    {
        UART_Debug("[LORA RX] *** PAIRING MODE - Accepting DATA packet ***");
        pair_device_from_did(did);

        /* Validate sequence (will auto-initialize on first packet) */
        (void)validate_sequence(seq);

        accept_wireless_data(level, wd);
        LoRa_SendACK(did);
        LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 120);
        s_validDataPackets++;
        return;
    }

    /* Normal mode - validate paired device */
    if (!PairedDev_IsAllowed(did))
    {
        snprintf(dbg, sizeof(dbg), "[LORA RX] !!! REJECTED - DID:%08lX not paired !!!", (unsigned long)did);
        UART_Debug(dbg);
        LoRa_SendReply("@REJECT#");
        s_rejectedPackets++;
        return;
    }

    /* Validate sequence number */
    if (!validate_sequence(seq))
    {
        /* Duplicate or out-of-order - still send ACK but don't update data */
        UART_Debug("[LORA RX] Sending ACK for duplicate/OOO packet (not updating data)");
        LoRa_SendACK(did);
        s_invalidPackets++;
        return;
    }

    /* Accept data */
    UART_Debug("[LORA RX] *** DATA ACCEPTED - Updating tank level ***");
    accept_wireless_data(level, wd);
    LoRa_SendACK(did);
    s_validDataPackets++;

    if (!g_loraConnected)
    {
        UART_Debug("[LORA RX] *** CONNECTION ESTABLISHED ***");
    }

    LED_SetIntent(LED_COLOR_GREEN, LED_MODE_STEADY, 1);
}
