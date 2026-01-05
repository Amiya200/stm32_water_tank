#include "lora.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "led.h"

/* ================= DEBUG ================= */
void Debug_Print(const char *s);

/* ================= GLOBALS ================= */
uint8_t loraMode = LORA_MODE_RECEIVER;
extern SPI_HandleTypeDef hspi1;

uint8_t rxBuffer[64];
uint32_t rxPacketCount = 0;

/* ================= CONSTANTS ================= */
#define LORA_FREQUENCY 433000000UL

/* ================= NSS ================= */
#define NSS_LOW()   HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET)
#define NSS_HIGH()  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET)

/* ================= SPI ================= */

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
    HAL_SPI_Receive(&hspi1, &rx, 1, HAL_MAX_DELAY);
    NSS_HIGH();
    return rx;
}

void LoRa_WriteBuffer(uint8_t addr, const uint8_t *buffer, uint8_t size)
{
    uint8_t a = addr | 0x80;
    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &a, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)buffer, size, HAL_MAX_DELAY);
    NSS_HIGH();
}

void LoRa_ReadBuffer(uint8_t addr, uint8_t *buffer, uint8_t size)
{
    uint8_t a = addr & 0x7F;
    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &a, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buffer, size, HAL_MAX_DELAY);
    NSS_HIGH();
}

/* ================= RESET (PRIVATE) ================= */

static void LoRa_Reset(void)
{
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
}

/* ================= INIT ================= */

void LoRa_Init(void)
{
    LoRa_Reset();

    /* LoRa + Sleep */
    LoRa_WriteReg(0x01, 0x80);
    HAL_Delay(10);

    /* Frequency */
    uint64_t frf = ((uint64_t)LORA_FREQUENCY << 19) / 32000000ULL;
    LoRa_WriteReg(0x06, frf >> 16);
    LoRa_WriteReg(0x07, frf >> 8);
    LoRa_WriteReg(0x08, frf);

    /* FIFO base */
    LoRa_WriteReg(0x0E, 0x00);
    LoRa_WriteReg(0x0F, 0x00);

    /* PA + LNA + AGC BOOST */
    LoRa_WriteReg(0x09, 0x8F);
    LoRa_WriteReg(0x0C, 0x23);
    LoRa_WriteReg(0x4D, 0x87);

    /* MODEM CONFIG (MATCH TX EXACTLY) */
    LoRa_WriteReg(0x1D, 0x72); // BW125, CR4/5, Explicit
    LoRa_WriteReg(0x1E, 0x70); // SF7, CRC OFF
    LoRa_WriteReg(0x26, 0x04);

    /* Public network */
    LoRa_WriteReg(0x39, 0x12);

    /* Clear IRQs */
    LoRa_WriteReg(0x12, 0xFF);

    /* RX continuous */
    LoRa_WriteReg(0x01, 0x85);

    LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_STEADY, 1);
}

/* ================= RECEIVE (DIO0-BASED, RELIABLE) ================= */

uint8_t LoRa_ReceivePacket(uint8_t *buffer, int16_t *rssi)
{
    /* DIO0 goes HIGH on RX_DONE */
    if (HAL_GPIO_ReadPin(LORA_DIO0_PORT, LORA_DIO0_PIN) == GPIO_PIN_RESET)
        return 0;

    uint8_t irq = LoRa_ReadReg(0x12);

    if (irq & 0x20) // CRC error
    {
        LoRa_WriteReg(0x12, 0xFF);
        return 0;
    }

    uint8_t len = LoRa_ReadReg(0x13);
    uint8_t fifoAddr = LoRa_ReadReg(0x10);

    LoRa_WriteReg(0x0D, fifoAddr);
    LoRa_ReadBuffer(0x00, buffer, len);

    int16_t raw = LoRa_ReadReg(0x1A);
    *rssi = -157 + raw;

    LoRa_WriteReg(0x12, 0xFF);

    return len;
}

/* ================= TASK ================= */

void LoRa_Task(void)
{
    if (loraMode != LORA_MODE_RECEIVER)
        return;

    int16_t rssi;
    uint8_t len = LoRa_ReceivePacket(rxBuffer, &rssi);

    if (len > 0)
    {
        rxBuffer[len] = '\0';
        rxPacketCount++;

        char msg[96];
        snprintf(msg, sizeof(msg),
                 "RX #%lu → %s | RSSI %d dBm\r\n",
                 rxPacketCount, rxBuffer, rssi);

        Debug_Print(msg);
        LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 120);
    }
}
