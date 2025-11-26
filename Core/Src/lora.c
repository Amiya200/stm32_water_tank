#include "lora.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include "led.h"

/* GLOBAL MODE VARIABLE — DEFINED ONCE HERE */
uint8_t loraMode = LORA_MODE_RECEIVER;   // Change in main.c if needed

extern SPI_HandleTypeDef hspi1;

uint8_t rxBuffer[64];
uint8_t txBuffer[64];

uint32_t txPacketCount = 0;
uint32_t rxPacketCount = 0;

// NSS control
#define NSS_LOW()   HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET)
#define NSS_HIGH()  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET)

#define LORA_FREQUENCY 433000000
#define LORA_TIMEOUT   2000

/* ----------------------------------------------------------
   SPI + LOW LEVEL HELPERS
---------------------------------------------------------- */
void LoRa_WriteReg(uint8_t addr, uint8_t data)
{
    uint8_t buf[2] = { (addr | 0x80), data };
    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
    NSS_HIGH();
}

uint8_t LoRa_ReadReg(uint8_t addr)
{
    uint8_t tx = addr & 0x7F;
    uint8_t rx = 0;
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

/* ----------------------------------------------------------
   RESET + INIT
---------------------------------------------------------- */
void LoRa_Reset(void)
{
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
}

void LoRa_SetFrequency(uint32_t freqHz)
{
    uint64_t frf = ((uint64_t)freqHz << 19) / 32000000ULL;

    LoRa_WriteReg(0x06, (uint8_t)(frf >> 16));
    LoRa_WriteReg(0x07, (uint8_t)(frf >> 8));
    LoRa_WriteReg(0x08, (uint8_t)(frf));
}

void LoRa_Init(void)
{
    LoRa_Reset();

    LoRa_WriteReg(0x01, 0x80);
    HAL_Delay(5);

    LoRa_SetFrequency(LORA_FREQUENCY);

    LoRa_WriteReg(0x09, 0x8F);
    LoRa_WriteReg(0x4D, 0x87);
    LoRa_WriteReg(0x0C, 0x23);

    LoRa_WriteReg(0x1D, 0x72);
    LoRa_WriteReg(0x1E, 0x74);
    LoRa_WriteReg(0x26, 0x04);

    LoRa_WriteReg(0x20, 0x00);
    LoRa_WriteReg(0x21, 0x08);

    LoRa_WriteReg(0x39, 0x22);
    LoRa_WriteReg(0x40, 0x00);
    LoRa_WriteReg(0x12, 0xFF);

    // LED initially ON
    LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_STEADY, 1);
}

/* ----------------------------------------------------------
   SET MODES
---------------------------------------------------------- */
void LoRa_SetStandby(void)
{
    LoRa_WriteReg(0x01, 0x81);
    HAL_Delay(2);
}

void LoRa_SetRxContinuous(void)
{
    LoRa_WriteReg(0x01, 0x85);
    HAL_Delay(2);
}

void LoRa_SetTx(void)
{
    LoRa_WriteReg(0x01, 0x83);
    HAL_Delay(2);
}

/* ----------------------------------------------------------
   SEND PACKET
---------------------------------------------------------- */
void LoRa_SendPacket(const uint8_t *buffer, uint8_t size)
{
    LoRa_SetStandby();

    LoRa_WriteReg(0x0E, 0x00);
    LoRa_WriteReg(0x0D, 0x00);

    LoRa_WriteBuffer(0x00, buffer, size);
    LoRa_WriteReg(0x22, size);

    LoRa_WriteReg(0x12, 0xFF);

    LoRa_SetTx();

    uint32_t t0 = HAL_GetTick();
    while (!(LoRa_ReadReg(0x12) & 0x08))
    {
        if (HAL_GetTick() - t0 > LORA_TIMEOUT) break;
    }

    LoRa_WriteReg(0x12, 0x08);
    LoRa_SetRxContinuous();
}

/* ----------------------------------------------------------
   RECEIVE PACKET WITH RSSI
---------------------------------------------------------- */
uint8_t LoRa_ReceivePacket(uint8_t *buffer, int16_t *rssi)
{
    uint8_t irq = LoRa_ReadReg(0x12);

    if (irq & 0x40)  // RxDone
    {
        if (irq & 0x20)
        {
            LoRa_WriteReg(0x12, 0xFF);
            return 0;
        }

        uint8_t len = LoRa_ReadReg(0x13);
        uint8_t addr = LoRa_ReadReg(0x10);

        LoRa_WriteReg(0x0D, addr);
        LoRa_ReadBuffer(0x00, buffer, len);

        int16_t raw_rssi = LoRa_ReadReg(0x1A);
        *rssi = -157 + raw_rssi;

        LoRa_WriteReg(0x12, 0xFF);
        return len;
    }

    return 0;
}

/* ----------------------------------------------------------
   MAIN LORA TASK
---------------------------------------------------------- */
void LoRa_Task(void)
{
    static uint32_t lastTx = 0;

    LoRa_SetRxContinuous();

    uint8_t version = LoRa_ReadReg(0x42);
    if (version != 0x12)
    {
        Debug_Print("LoRa NOT detected!\r\n");
        LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_OFF, 0);
        return;
    }

    /* ---------------------- RECEIVER MODE ---------------------- */
    if (loraMode == LORA_MODE_RECEIVER)
    {
        int16_t rssi;
        uint8_t len = LoRa_ReceivePacket(rxBuffer, &rssi);

        if (len > 0)
        {
            rxBuffer[len] = '\0';
            rxPacketCount++;

            char msg[128];
            snprintf(msg, sizeof(msg),
                    "RX #%lu → %s | RSSI: %d dBm\r\n",
                    rxPacketCount, rxBuffer, rssi);

            Debug_Print(msg);

            LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_STEADY, 0);
        }
        else
        {
            LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_OFF, 0);
        }
        return;
    }

    /* ---------------------- TRANSMITTER MODE ---------------------- */
    if (loraMode == LORA_MODE_TRANSMITTER)
    {
        uint32_t now = HAL_GetTick();

        LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_BLINK, 150); // TX blink

        if (now - lastTx >= 1000)
        {
            txPacketCount++;

            char msg[32];
            snprintf(msg, sizeof(msg),
                    "TX#%lu", txPacketCount);

            LoRa_SendPacket((uint8_t*)msg, strlen(msg));

            Debug_Print("TX → ");
            Debug_Print(msg);
            Debug_Print("\r\n");

            lastTx = now;
        }
    }
}
