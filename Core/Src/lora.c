#include "lora.h"
#include "stm32f1xx_hal.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

// LoRa Modes
#define LORA_MODE_TRANSMITTER 1
#define LORA_MODE_RECEIVER    2
#define LORA_MODE_TRANCEIVER  3 // Both Transmitter and Receiver


uint8_t rxBuffer[32];        // LoRa RX
uint8_t txBuffer[32];        // LoRa TX


// NSS Pin Control
#define NSS_LOW()   HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET)
#define NSS_HIGH()  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET)

// Constants
#define LORA_FREQUENCY 433000000 // Frequency in Hz
#define LORA_TIMEOUT 2000 // Timeout for TxDone in milliseconds
#define LORA_PING_MSG "PING"
#define LORA_ACK_MSG "ACK"
#define LORA_HELLO_MSG "HELLO"
#define LORA_BUFFER_SIZE 50 // Size for debug messages

static uint8_t z = 0;
uint8_t connectionStatus = 0; // 0 = lost, 1 = OK
uint8_t loraMode = LORA_MODE_TRANSMITTER; // Default to Transceiver mode




//uint8_t rxBuffer[256]; // Buffer for received data

/* --- low-level SPI helpers --- */
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

/* --- reset --- */
void LoRa_Reset(void) {
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
}

/* --- set frequency (Hz) --- */
void LoRa_SetFrequency(uint32_t freqHz) {
    /* FRF = freq * 2^19 / 32e6 */
    uint64_t frf = ((uint64_t)freqHz << 19) / 32000000ULL;
    LoRa_WriteReg(0x06, (uint8_t)(frf >> 16));
    LoRa_WriteReg(0x07, (uint8_t)(frf >> 8));
    LoRa_WriteReg(0x08, (uint8_t)(frf >> 0));
}

/* --- init with settings that match Arduino LoRa defaults --- */
void LoRa_Init(void) {
    LoRa_Reset();

    /* Sleep, then LoRa sleep mode */
    LoRa_WriteReg(0x01, 0x00);
    HAL_Delay(5);
    LoRa_WriteReg(0x01, 0x80);
    HAL_Delay(5);

    /* Frequency (433 MHz) */
    LoRa_SetFrequency(LORA_FREQUENCY);

    /* PA config: PA_BOOST, max power (common for SX1278 Ra-02) */
    LoRa_WriteReg(0x09, 0x8F); // PA_BOOST, max power

    /* Enable high-power PA if module supports it (optional) */
    LoRa_WriteReg(0x4D, 0x87); // RegPaDac (only on SX1276/78 family)

    /* LNA */
    LoRa_WriteReg(0x0C, 0x23);

    /* Modem config:
       RegModemConfig1 = 0x72 -> BW=125kHz, CR=4/5, Explicit header
       RegModemConfig2 = 0x74 -> SF7, CRC ON (Arduino default)
       RegModemConfig3 = 0x04 -> LowDataRateOptimize off, AGC Auto On
    */
    LoRa_WriteReg(0x1D, 0x72);
    LoRa_WriteReg(0x1E, 0x74);
    LoRa_WriteReg(0x26, 0x04);

    /* Preamble = 8 */
    LoRa_WriteReg(0x20, 0x00);
    LoRa_WriteReg(0x21, 0x08);

    /* SyncWord = 0x22 (matches LoRa.setSyncWord(0x22) on Arduino) */
    LoRa_WriteReg(0x39, 0x22);

    /* Map DIO0 = RxDone/TxDone as normal (we'll poll IRQs) */
    LoRa_WriteReg(0x40, 0x00);

    /* Clear IRQs */
    LoRa_WriteReg(0x12, 0xFF);
}

/**
  * @brief Sets the LoRa module to Standby mode.
  */
void LoRa_SetStandby(void) {
    LoRa_WriteReg(0x01, 0x81); // Standby mode
    HAL_Delay(2);
}

/**
  * @brief Sets the LoRa module to Continuous Receive mode.
  */
void LoRa_SetRxContinuous(void) {
    LoRa_WriteReg(0x01, 0x85); // Continuous RX mode
    HAL_Delay(2);
}

/**
  * @brief Sets the LoRa module to Transmit mode.
  */
void LoRa_SetTx(void) {
    LoRa_WriteReg(0x01, 0x83); // TX mode
    HAL_Delay(2);
}

/* --- send packet, poll TxDone, return to RX --- */
void LoRa_SendPacket(const uint8_t *buffer, uint8_t size) {
    LoRa_SetStandby(); // Go to Standby before TX

    /* Reset FIFO pointers */
    LoRa_WriteReg(0x0E, 0x00);
    LoRa_WriteReg(0x0D, 0x00);

    /* Write payload */
    LoRa_WriteBuffer(0x00, buffer, size);
    LoRa_WriteReg(0x22, size);

    /* Clear IRQs */
    LoRa_WriteReg(0x12, 0xFF);

    LoRa_SetTx(); // Enter TX

    /* Wait for TxDone */
    uint32_t t0 = HAL_GetTick();
    while ((LoRa_ReadReg(0x12) & 0x08) == 0) {
        if ((HAL_GetTick() - t0) > LORA_TIMEOUT) break; // timeout
        HAL_Delay(1);
    }

    /* Clear TxDone (if set) */
    LoRa_WriteReg(0x12, 0x08);

    LoRa_SetRxContinuous(); // Back to RX after TX
}

/* --- receive helper: returns length or 0 --- */
uint8_t LoRa_ReceivePacket(uint8_t *buffer) {
    uint8_t irq = LoRa_ReadReg(0x12);
    if (irq & 0x40) { // RxDone
        /* If CRC error bit (0x20) is set, ignore */
        if (irq & 0x20) {
            /* CRC error */
            LoRa_WriteReg(0x12, 0xFF);
            return 0;
        }
        uint8_t nb = LoRa_ReadReg(0x13);   // RegRxNbBytes
        uint8_t addr = LoRa_ReadReg(0x10); // RegFifoRxCurrentAddr
        LoRa_WriteReg(0x0D, addr);
        LoRa_ReadBuffer(0x00, buffer, nb);
        LoRa_WriteReg(0x12, 0xFF); // clear IRQs
        return nb;
    }
    return 0;
}

/* --- LoRa Task --- */
void LoRa_Task(void) {
    // Set the initial mode
    if (loraMode == LORA_MODE_RECEIVER || loraMode == LORA_MODE_TRANCEIVER) {
        LoRa_SetRxContinuous(); // Start in RX mode if receiver or transceiver
        Debug_Print("LoRa set to RX Continuous mode.\r\n");
    } else {
        LoRa_SetStandby(); // Otherwise, start in Standby
        Debug_Print("LoRa set to Standby mode.\r\n");
    }

    // === Verify LoRa chip ===
    uint8_t version = LoRa_ReadReg(0x42);  // SX1278 RegVersion
    if (version != 0x12) {
        z = 1;
        char errMsg[LORA_BUFFER_SIZE];
        sprintf(errMsg, "LoRa not found! RegVersion=0x%02X\r\n", version);
        Debug_Print(errMsg);
        HAL_Delay(2000);
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
            HAL_Delay(2000); // Send every 2 seconds
            break;

        case LORA_MODE_RECEIVER:
            Debug_Print("LoRa Mode: Receiver\r\n");
            connectionStatus = 0; // Reset connection status

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
                HAL_Delay(25);
            }

            // Step 3: Handle failed connection
            if (!connectionStatus) {
                Debug_Print("Connection failed. No PING received.\r\n");
                z = 7;
                HAL_Delay(1000); // retry delay
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
                    HAL_Delay(25);
                }

                if (!connectionStatus) {
                    Debug_Print("HELLO not received after ACK.\r\n");
                    z = 9; // special error state for HELLO timeout
                }
            }

            HAL_Delay(100);
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
                HAL_Delay(25);
            }

            if (!connectionStatus) {
                Debug_Print("Connection: LOST\r\n");
                z = 4;
            } else {
                Debug_Print("Connection: OK\r\n");
            }

            HAL_Delay(1000); // Delay before next cycle in transceiver mode
            break;

        default:
            Debug_Print("Invalid LoRa Mode!\r\n");
            HAL_Delay(1000);
            break;
    }
}
