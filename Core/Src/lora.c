#include "lora.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;

// LoRa Modes
#define LORA_MODE_TRANSMITTER 1
#define LORA_MODE_RECEIVER    2
#define LORA_MODE_TRANCEIVER  3 // Both Transmitter and Receiver

// NSS Pin Control
#define NSS_LOW()   HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET)
#define NSS_HIGH()  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET)

// Constants
#define LORA_FREQUENCY 433000000UL // Frequency in Hz
#define LORA_TIMEOUT   2000        // Timeout for TxDone in milliseconds
#define LORA_PING_MSG  "PING"
#define LORA_ACK_MSG   "ACK"
#define LORA_HELLO_MSG "HELLO"
#define LORA_BUFFER_SIZE 64        // Debug buffer size

// Globals
static uint8_t z = 0;
uint8_t connectionStatus = 0; // 0 = lost, 1 = OK
uint8_t loraMode = LORA_MODE_TRANSMITTER;

uint8_t rxBuffer[64]; // LoRa RX
uint8_t txBuffer[64]; // LoRa TX

/* ---------------- Low-level SPI helpers ---------------- */
void LoRa_WriteReg(uint8_t addr, uint8_t data) {
    uint8_t buf[2] = { (uint8_t)(addr | 0x80), data };
    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
    NSS_HIGH();
}

uint8_t LoRa_ReadReg(uint8_t addr) {
    uint8_t tx = addr & 0x7F;
    uint8_t rx = 0;
    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &rx, 1, HAL_MAX_DELAY);
    NSS_HIGH();
    return rx;
}

void LoRa_WriteBuffer(uint8_t addr, const uint8_t *buffer, uint8_t size) {
    uint8_t a = addr | 0x80;
    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &a, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)buffer, size, HAL_MAX_DELAY);
    NSS_HIGH();
}

void LoRa_ReadBuffer(uint8_t addr, uint8_t *buffer, uint8_t size) {
    uint8_t a = addr & 0x7F;
    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &a, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buffer, size, HAL_MAX_DELAY);
    NSS_HIGH();
}

/* ---------------- Reset ---------------- */
void LoRa_Reset(void) {
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(2);
}

/* ---------------- Set frequency ---------------- */
void LoRa_SetFrequency(uint32_t freqHz) {
    uint64_t frf = ((uint64_t)freqHz << 19) / 32000000ULL;
    LoRa_WriteReg(0x06, (uint8_t)(frf >> 16));
    LoRa_WriteReg(0x07, (uint8_t)(frf >> 8));
    LoRa_WriteReg(0x08, (uint8_t)(frf >> 0));
}

/* ---------------- Init ---------------- */
void LoRa_Init(void) {
    LoRa_Reset();

    // Sleep → LoRa sleep
    LoRa_WriteReg(0x01, 0x00);
    HAL_Delay(2);
    LoRa_WriteReg(0x01, 0x80);
    HAL_Delay(2);

    // Frequency
    LoRa_SetFrequency(LORA_FREQUENCY);

    // PA config
    LoRa_WriteReg(0x09, 0x8F); // PA_BOOST
    LoRa_WriteReg(0x4D, 0x87); // RegPaDac

    // LNA
    LoRa_WriteReg(0x0C, 0x23);

    // Modem config
    LoRa_WriteReg(0x1D, 0x72);
    LoRa_WriteReg(0x1E, 0x74);
    LoRa_WriteReg(0x26, 0x04);

    // Preamble
    LoRa_WriteReg(0x20, 0x00);
    LoRa_WriteReg(0x21, 0x08);

    // SyncWord
    LoRa_WriteReg(0x39, 0x22);

    // DIO mapping
    LoRa_WriteReg(0x40, 0x00);

    // Clear IRQs
    LoRa_WriteReg(0x12, 0xFF);
}

/* ---------------- Mode control ---------------- */
void LoRa_SetStandby(void) { LoRa_WriteReg(0x01, 0x81); }
void LoRa_SetRxContinuous(void) { LoRa_WriteReg(0x01, 0x85); }
void LoRa_SetTx(void) { LoRa_WriteReg(0x01, 0x83); }

/* ---------------- Send packet ---------------- */
void LoRa_SendPacket(const uint8_t *buffer, uint8_t size) {
    LoRa_SetStandby();

    // FIFO reset
    LoRa_WriteReg(0x0E, 0x00);
    LoRa_WriteReg(0x0D, 0x00);

    // Write payload
    LoRa_WriteBuffer(0x00, buffer, size);
    LoRa_WriteReg(0x22, size);

    // Clear IRQs
    LoRa_WriteReg(0x12, 0xFF);

    LoRa_SetTx();

    // Wait for TxDone
    uint32_t start = HAL_GetTick();
    while (!(LoRa_ReadReg(0x12) & 0x08)) {
        if (HAL_GetTick() - start > LORA_TIMEOUT) {
            LoRa_WriteReg(0x12, 0xFF);
            LoRa_SetRxContinuous();
            return;
        }
    }

    LoRa_WriteReg(0x12, 0x08); // Clear TxDone
    LoRa_SetRxContinuous();
}

/* ---------------- Receive packet ---------------- */
uint8_t LoRa_ReceivePacket(uint8_t *buffer) {
    uint8_t irq = LoRa_ReadReg(0x12);
    if (irq & 0x40) { // RxDone
        if (irq & 0x20) { // CRC error
            LoRa_WriteReg(0x12, 0xFF);
            return 0;
        }
        uint8_t len = LoRa_ReadReg(0x13);
        uint8_t addr = LoRa_ReadReg(0x10);
        LoRa_WriteReg(0x0D, addr);
        LoRa_ReadBuffer(0x00, buffer, len);
        LoRa_WriteReg(0x12, 0xFF);
        return len;
    }
    return 0;
}

/* ---------------- LoRa Task (non-blocking) ---------------- */
void LoRa_Task(void) {
    static uint32_t lastPing = 0;
    char dbg[LORA_BUFFER_SIZE];

    // Verify chip
    uint8_t version = LoRa_ReadReg(0x42);
    if (version != 0x12) {
        snprintf(dbg, sizeof(dbg), "LoRa not found! RegVersion=0x%02X\r\n", version);
        Debug_Print(dbg);
        return;
    }

    switch (loraMode) {
    case LORA_MODE_TRANSMITTER:
        if (HAL_GetTick() - lastPing > 1000) { // every 1s
            uint8_t msg[] = "HELLO_TX";
            LoRa_SendPacket(msg, sizeof(msg) - 1);
            Debug_Print("TX: HELLO_TX\r\n");
            lastPing = HAL_GetTick();
        }
        break;

    case LORA_MODE_RECEIVER: {
        uint8_t len = LoRa_ReceivePacket(rxBuffer);
        if (len > 0) {
            rxBuffer[len] = '\0';
            snprintf(dbg, sizeof(dbg), "RX: %s\r\n", rxBuffer);
            Debug_Print(dbg);

            if (strcmp((char*)rxBuffer, LORA_PING_MSG) == 0) {
                uint8_t ack[] = LORA_ACK_MSG;
                LoRa_SendPacket(ack, sizeof(ack) - 1);
                Debug_Print("Sent ACK\r\n");
                connectionStatus = 1;
            } else if (strcmp((char*)rxBuffer, LORA_HELLO_MSG) == 0) {
                Debug_Print("HELLO received, Connection OK\r\n");
                connectionStatus = 1;
            }
        }
        break;
    }

    case LORA_MODE_TRANCEIVER: {
        uint8_t len = LoRa_ReceivePacket(rxBuffer);
        if (len > 0) {
            rxBuffer[len] = '\0';
            snprintf(dbg, sizeof(dbg), "RX: %s\r\n", rxBuffer);
            Debug_Print(dbg);

            if (strcmp((char*)rxBuffer, LORA_PING_MSG) == 0) {
                uint8_t ack[] = LORA_ACK_MSG;
                LoRa_SendPacket(ack, sizeof(ack) - 1);
                Debug_Print("Sent ACK\r\n");
            }
        }

        // Send PING every 500ms
        if (HAL_GetTick() - lastPing > 500) {
            uint8_t msg[] = LORA_PING_MSG;
            LoRa_SendPacket(msg, sizeof(msg) - 1);
            Debug_Print("Sent PING\r\n");
            lastPing = HAL_GetTick();
        }
        break;
    }

    default:
        Debug_Print("Invalid LoRa mode!\r\n");
        break;
    }
}
