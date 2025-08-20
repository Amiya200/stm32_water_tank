#include "lora.h"
#include "stm32f1xx_hal.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

#define NSS_LOW()   HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET)
#define NSS_HIGH()  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET)

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
    LoRa_SetFrequency(433000000);

    /* PA config: PA_BOOST, max power (common for SX1278 Ra-02) */

    LoRa_WriteReg(0x09, 0x8F); // PA_BOOST, max power
 // RegPaConfig

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

    /* Clear IRQs and go to continuous RX */
    LoRa_WriteReg(0x12, 0xFF);
    LoRa_WriteReg(0x01, 0x81); // Standby
    HAL_Delay(2);
    LoRa_WriteReg(0x01, 0x85); // Continuous RX
}

/* --- send packet, poll TxDone, return to RX --- */
void LoRa_SendPacket(const uint8_t *buffer, uint8_t size) {
    /* Standby */
    LoRa_WriteReg(0x01, 0x81);

    /* Reset FIFO pointers */
    LoRa_WriteReg(0x0E, 0x00);
    LoRa_WriteReg(0x0D, 0x00);

    /* Write payload */
    LoRa_WriteBuffer(0x00, buffer, size);
    LoRa_WriteReg(0x22, size);

    /* Clear IRQs */
    LoRa_WriteReg(0x12, 0xFF);

    /* Enter TX */
    LoRa_WriteReg(0x01, 0x83);

    /* Wait for TxDone */
    uint32_t t0 = HAL_GetTick();
    while ((LoRa_ReadReg(0x12) & 0x08) == 0) {
        if ((HAL_GetTick() - t0) > 2000) break; // timeout 2s
        HAL_Delay(1);
    }

    /* Clear TxDone (if set) */
    LoRa_WriteReg(0x12, 0x08);

    /* Back to RX */
    LoRa_WriteReg(0x01, 0x85);
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
