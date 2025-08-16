#include "LoRa.h"
#include "main.h"

// Helper macros
#define NSS_LOW()    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET)
#define NSS_HIGH()   HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET)

// Reset the LoRa chip
void LoRa_Reset(void) {
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
}

// Write single register
void LoRa_WriteReg(uint8_t addr, uint8_t data) {
    uint8_t buf[2];
    buf[0] = addr | 0x80; // MSB=1 for write
    buf[1] = data;

    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
    NSS_HIGH();
}

// Read single register
uint8_t LoRa_ReadReg(uint8_t addr) {
    uint8_t tx = addr & 0x7F; // MSB=0 for read
    uint8_t rx = 0;

    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &rx, 1, HAL_MAX_DELAY);
    NSS_HIGH();

    return rx;
}

// Write multiple bytes
void LoRa_WriteBuffer(uint8_t addr, uint8_t *buffer, uint8_t size) {
    addr |= 0x80;

    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, buffer, size, HAL_MAX_DELAY);
    NSS_HIGH();
}

// Read multiple bytes
void LoRa_ReadBuffer(uint8_t addr, uint8_t *buffer, uint8_t size) {
    addr &= 0x7F;

    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buffer, size, HAL_MAX_DELAY);
    NSS_HIGH();
}

// LoRa Init (basic setup for 433MHz or 868/915MHz)
void LoRa_Init(void) {
    LoRa_Reset();

    // Put device in sleep mode
    LoRa_WriteReg(0x01, 0x00); // RegOpMode: Sleep, FSK mode
    HAL_Delay(10);

    // Switch to LoRa mode
    LoRa_WriteReg(0x01, 0x80); // RegOpMode: LoRa + Sleep
    HAL_Delay(10);

    // Frequency setup (example: 433 MHz)
    LoRa_WriteReg(0x06, 0x6C); // RegFrfMsb
    LoRa_WriteReg(0x07, 0x80); // RegFrfMid
    LoRa_WriteReg(0x08, 0x00); // RegFrfLsb

    // Power setup
    LoRa_WriteReg(0x09, 0x8F); // RegPaConfig: PA_BOOST, max power

    // LNA boost
    LoRa_WriteReg(0x0C, 0x23);

    // Bandwidth + coding rate + explicit header
    LoRa_WriteReg(0x1D, 0x72);

    // Spreading factor + CRC
    LoRa_WriteReg(0x1E, 0x74);

    // Preamble
    LoRa_WriteReg(0x20, 0x00);
    LoRa_WriteReg(0x21, 0x08);

    // Continuous RX mode
    LoRa_WriteReg(0x01, 0x85);
}

// Send packet
void LoRa_SendPacket(uint8_t *buffer, uint8_t size) {
    // Set to standby
    LoRa_WriteReg(0x01, 0x81);

    // Set FIFO address
    LoRa_WriteReg(0x0E, 0x00); // FifoTxBaseAddr
    LoRa_WriteReg(0x0D, 0x00); // FifoAddrPtr

    // Write data
    LoRa_WriteBuffer(0x00, buffer, size);

    // Payload length
    LoRa_WriteReg(0x22, size);

    // TX mode
    LoRa_WriteReg(0x01, 0x83);

    // Wait for DIO0 = TxDone
    while (HAL_GPIO_ReadPin(LORA_DIO0_PORT, LORA_DIO0_PIN) == GPIO_PIN_RESET);

    // Clear IRQ flags
    LoRa_WriteReg(0x12, 0xFF);
}

// Receive packet
uint8_t LoRa_ReceivePacket(uint8_t *buffer) {
    uint8_t size = 0;

    if (HAL_GPIO_ReadPin(LORA_DIO0_PORT, LORA_DIO0_PIN) == GPIO_PIN_SET) {
        // Packet received
        size = LoRa_ReadReg(0x13); // RxNbBytes
        LoRa_WriteReg(0x0D, LoRa_ReadReg(0x10)); // FifoAddrPtr = FifoRxCurrentAddr
        LoRa_ReadBuffer(0x00, buffer, size);

        // Clear IRQ
        LoRa_WriteReg(0x12, 0xFF);
    }

    return size;
}
