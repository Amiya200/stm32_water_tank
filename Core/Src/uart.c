#include "uart.h"
#include <string.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart1;

/* ---------------- Configuration ---------------- */
#define UART_START_MARKER '@'
#define UART_END_MARKER   '#'

static uint8_t rxByte;
static char rxBuffer[UART_RX_BUFFER_SIZE];
static char rxReadyBuffer[UART_RX_BUFFER_SIZE];

static volatile uint16_t rxIndex = 0;
static volatile bool packetReady = false;
static volatile bool inPacket = false;

/* -------------------------------------------------
 * UART Initialization
 * ------------------------------------------------- */
void UART_Init(void)
{
    memset(rxBuffer, 0, sizeof(rxBuffer));
    memset(rxReadyBuffer, 0, sizeof(rxReadyBuffer));
    rxIndex = 0;
    packetReady = false;
    inPacket = false;

    HAL_UART_Receive_IT(&huart1, &rxByte, 1);  // start 1-byte receive interrupt
}

/* -------------------------------------------------
 * Transmit helpers
 * ------------------------------------------------- */
void UART_TransmitString(UART_HandleTypeDef *huart, const char *str)
{
    HAL_UART_Transmit(huart, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

void UART_TransmitByte(UART_HandleTypeDef *huart, uint8_t byte)
{
    HAL_UART_Transmit(huart, &byte, 1, HAL_MAX_DELAY);
}

/* -------------------------------------------------
 * RX Interrupt Handler
 * ------------------------------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // --- Packet parser ---
        if (!packetReady)
        {
            if (rxByte == UART_START_MARKER)
            {
                inPacket = true;
                rxIndex = 0;
                memset(rxBuffer, 0, sizeof(rxBuffer));
            }
            else if (inPacket && rxByte == UART_END_MARKER)
            {
                // End of packet
                rxBuffer[rxIndex] = UART_END_MARKER;
                rxBuffer[rxIndex + 1] = '\0';
                memcpy(rxReadyBuffer, rxBuffer, rxIndex + 2);
                packetReady = true;
                inPacket = false;
                rxIndex = 0;
            }
            else if (inPacket)
            {
                if (rxIndex < (UART_RX_BUFFER_SIZE - 2))
                {
                    rxBuffer[rxIndex++] = rxByte;
                }
                else
                {
                    // Overflow, reset safely
                    inPacket = false;
                    rxIndex = 0;
                    memset(rxBuffer, 0, sizeof(rxBuffer));
                }
            }
        }

        // Restart single-byte receive interrupt
        HAL_UART_Receive_IT(&huart1, &rxByte, 1);
    }
}

/* -------------------------------------------------
 * Retrieve one complete packet safely
 * ------------------------------------------------- */
bool UART_GetReceivedPacket(char *buffer, size_t buffer_size)
{
    if (!packetReady)
        return false;

    __disable_irq();
    size_t len = strlen(rxReadyBuffer);
    if (len >= buffer_size)
        len = buffer_size - 1;

    strncpy(buffer, rxReadyBuffer, len);
    buffer[len] = '\0';

    packetReady = false;
    memset(rxReadyBuffer, 0, sizeof(rxReadyBuffer));
    __enable_irq();

    return true;
}
