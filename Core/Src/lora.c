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
/**
 * @brief Test LoRa connectivity as a transmitter by sending a test message.
 * @return uint8_t 1 if transmission successful, 0 if timeout or failure.
 */
uint8_t LoRa_TestConnectivity_Transmitter(void) {
    const uint8_t test_msg[] = "TEST";
    uint32_t startTick;

<<<<<<< HEAD
/* ---------------- LoRa Task (non-blocking) ---------------- */
=======
    LoRa_SetStandby(); // Ensure module is in standby before sending

    // Reset FIFO pointers
    LoRa_WriteReg(0x0E, 0x00);
    LoRa_WriteReg(0x0D, 0x00);

    // Write test message to FIFO
    LoRa_WriteBuffer(0x00, test_msg, sizeof(test_msg) - 1);
    LoRa_WriteReg(0x22, sizeof(test_msg) - 1);

    // Clear IRQ flags
    LoRa_WriteReg(0x12, 0xFF);

    // Set to TX mode
    LoRa_SetTx();

    // Wait for TxDone flag or timeout
    startTick = HAL_GetTick();
    while ((LoRa_ReadReg(0x12) & 0x08) == 0) {
        if ((HAL_GetTick() - startTick) > LORA_TIMEOUT) {
            // Timeout occurred
            LoRa_WriteReg(0x12, 0xFF); // Clear IRQs
            LoRa_SetRxContinuous(); // Return to RX mode
            return 0; // Transmission failed
        }
        HAL_Delay(1);
    }

    // Clear TxDone flag
    LoRa_WriteReg(0x12, 0x08);

    // Return to RX mode after transmission
    LoRa_SetRxContinuous();

    return 1; // Transmission successful
}
/* --- LoRa Task --- */
>>>>>>> 98979338ca7bfb3b77713f55d952a9e70c09570d
void LoRa_Task(void) {
    static uint32_t lastPing = 0;
    char dbg[LORA_BUFFER_SIZE];

    // Verify chip
    uint8_t version = LoRa_ReadReg(0x42);
    if (version != 0x12) {
<<<<<<< HEAD
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
=======
        z = 1;
        char errMsg[LORA_BUFFER_SIZE];
        sprintf(errMsg, "LoRa not found! RegVersion=0x%02X\r\n", version);
        Debug_Print(errMsg);
        HAL_Delay(2);
        return; // retry until chip responds
    }

    switch (loraMode) {
        case LORA_MODE_TRANSMITTER:
            // Transmitter logic
            Debug_Print("LoRa Mode: Transmitter\r\n");
            uint8_t tx_msg[] = "HELLO_TX";
            z = 5;
            LoRa_SendPacket(tx_msg, sizeof(tx_msg) - 1);
            Debug_Print("Sent: HELLO_TX\r\n");
            HAL_Delay(5); // Send every 2 seconds
            break;
>>>>>>> 98979338ca7bfb3b77713f55d952a9e70c09570d

    case LORA_MODE_RECEIVER: {
        uint8_t len = LoRa_ReceivePacket(rxBuffer);
        if (len > 0) {
            rxBuffer[len] = '\0';
            snprintf(dbg, sizeof(dbg), "RX: %s\r\n", rxBuffer);
            Debug_Print(dbg);

<<<<<<< HEAD
            z=123;
            if (strcmp((char*)rxBuffer, LORA_PING_MSG) == 0) {
                uint8_t ack[] = LORA_ACK_MSG;
                LoRa_SendPacket(ack, sizeof(ack) - 1);
                Debug_Print("Sent ACK\r\n");
                connectionStatus = 1;
            } else if (strcmp((char*)rxBuffer, LORA_HELLO_MSG) == 0) {
                Debug_Print("HELLO received, Connection OK\r\n");
                connectionStatus = 1;
=======
            // Step 1: Wait for "PING" from transmitter
            for (int i = 0; i < 40; i++) {   // ~1s timeout (40 x 25ms)
                uint8_t len = LoRa_ReceivePacket(rxBuffer);
                if (len > 0) {
                    rxBuffer[len] = '\0'; // null terminate
                    char dbg_rx[LORA_BUFFER_SIZE];
                    sprintf(dbg_rx, "Received: %s\r\n", rxBuffer);
                    Debug_Print(dbg_rx);

                    if (strncmp((char*)rxBuffer, LORA_PING_MSG, strlen(LORA_PING_MSG)) == 0) {
                        // Step 2: Reply with "ACK"
                        uint8_t ack_msg[] = LORA_ACK_MSG;
                        LoRa_SendPacket(ack_msg, sizeof(ack_msg) - 1);
                        Debug_Print("Sent: ACK\r\n");

                        connectionStatus = 1;
                        z = 6; // connection established
                        break;
                    }
                }
                HAL_Delay(5);
>>>>>>> 98979338ca7bfb3b77713f55d952a9e70c09570d
            }
        }
        break;
    }

<<<<<<< HEAD
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
=======
            // Step 3: Handle failed connection
            if (!connectionStatus) {
                Debug_Print("Connection failed. No PING received.\r\n");
                z = 7;
                HAL_Delay(5); // retry delay
            } else {
                // Step 4: Wait for HELLO after PING->ACK
                Debug_Print("Waiting for HELLO...\r\n");
                connectionStatus = 0; // reset until HELLO is confirmed

                for (int j = 0; j < 40; j++) {   // ~1s timeout for HELLO
                    uint8_t rx_len = LoRa_ReceivePacket(rxBuffer);
                    if (rx_len > 0) {
                        rxBuffer[rx_len] = '\0';
                        char dbg_rx2[LORA_BUFFER_SIZE];
                        sprintf(dbg_rx2, "Data Received: %s\r\n", rxBuffer);
                        Debug_Print(dbg_rx2);

                        if (strncmp((char*)rxBuffer, LORA_HELLO_MSG, strlen(LORA_HELLO_MSG)) == 0) {
                            Debug_Print("HELLO received -> Final Connection Established\r\n");
                            connectionStatus = 1;
                            z = 8; // Final established state
                            break;
                        }
                    }
                    HAL_Delay(2);
                }

                if (!connectionStatus) {
                    Debug_Print("HELLO not received after ACK.\r\n");
                    z = 9; // special error state for HELLO timeout
                }
>>>>>>> 98979338ca7bfb3b77713f55d952a9e70c09570d
            }
        }

<<<<<<< HEAD
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
=======
            HAL_Delay(5);
            break;

        case LORA_MODE_TRANCEIVER:
            // Transceiver logic (send and receive)
            Debug_Print("LoRa Mode: Transceiver\r\n");

            // Try to receive first
            uint8_t rx_len_tr = LoRa_ReceivePacket(rxBuffer);
            if (rx_len_tr > 0) {
                rxBuffer[rx_len_tr] = '\0'; // null terminate
                char dbg_rx_tr[LORA_BUFFER_SIZE];
                sprintf(dbg_rx_tr, "Received: %s\r\n", rxBuffer);
                Debug_Print(dbg_rx_tr);

                // If "PING" is received, send "ACK"
                if (strncmp((char*)rxBuffer, LORA_PING_MSG, strlen(LORA_PING_MSG)) == 0) {
                    uint8_t ack_msg[] = LORA_ACK_MSG;
                    LoRa_SendPacket(ack_msg, sizeof(ack_msg) - 1);
                    Debug_Print("Sent: ACK\r\n");
                }
            }

            // Then send a PING
            uint8_t tx_msg_tr[] = LORA_PING_MSG;
            LoRa_SendPacket(tx_msg_tr, sizeof(tx_msg_tr) - 1);
            Debug_Print("Sent: PING\r\n");

            // Wait for ACK (max 500 ms)
            connectionStatus = 0;
            for (int i = 0; i < 20; i++) {   // 20 x 25ms = 500ms
                uint8_t len = LoRa_ReceivePacket(rxBuffer);
                if (len > 0) {
                    rxBuffer[len] = '\0'; // null terminate
                    char dbg_ack[LORA_BUFFER_SIZE];
                    sprintf(dbg_ack, "Received ACK check: %s\r\n", rxBuffer);
                    Debug_Print(dbg_ack);

                    if (strncmp((char*)rxBuffer, LORA_ACK_MSG, strlen(LORA_ACK_MSG)) == 0) {
                        connectionStatus = 1;
                        z = 3;
                        break;
                    }
                }
                HAL_Delay(2);
            }

            if (!connectionStatus) {
                Debug_Print("Connection: LOST\r\n");
                z = 4;
            } else {
                Debug_Print("Connection: OK\r\n");
            }

            HAL_Delay(5); // Delay before next cycle in transceiver mode
            break;

        default:
            Debug_Print("Invalid LoRa Mode!\r\n");
            HAL_Delay(5);
            break;
>>>>>>> 98979338ca7bfb3b77713f55d952a9e70c09570d
    }
}
