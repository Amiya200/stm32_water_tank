#include "uart.h"
//#include <stdbool.h> // Add this line
// RX buffers
static uint8_t rxByte;                             // single-byte buffer for HAL_UART_Receive_IT
static char rxBuffer[UART_RX_BUFFER_SIZE];         // internal storage buffer
static uint16_t rxIndex = 0;                       // current index in rxBuffer
static bool packetReady = false;                   // Flag to indicate if a complete packet is ready

/**
  * @brief Initialize UART reception (interrupt mode, one byte at a time).
  */
void UART_Init(void)
{
    // Clear the buffer and reset index
    memset(rxBuffer, 0, sizeof(rxBuffer));
    rxIndex = 0;
    packetReady = false;

    // Start UART reception (non-blocking)
    HAL_UART_Receive_IT(&huart1, &rxByte, 1);
}

/**
  * @brief Transmit a null-terminated string.
  */
void UART_TransmitString(UART_HandleTypeDef *huart, const char *str)
{
    HAL_UART_Transmit(huart, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/**
  * @brief Transmit a single byte.
  */
void UART_TransmitByte(UART_HandleTypeDef *huart, uint8_t byte)
{
    HAL_UART_Transmit(huart, &byte, 1, HAL_MAX_DELAY);
}

/**
  * @brief Callback when one byte is received (interrupt-driven).
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Check for buffer overflow before storing the byte
        if (rxIndex < (UART_RX_BUFFER_SIZE - 1))
        {
            rxBuffer[rxIndex++] = rxByte;

            // Check if the received byte is the delimiter
            if (rxByte == UART_RX_DELIMITER)
            {
                rxBuffer[rxIndex] = '\0'; // Null-terminate the received packet
                packetReady = true;       // Set flag that a complete packet is ready
            }
        }
        else
        {
            // Buffer overflow: reset buffer and index, discard current partial packet
            rxIndex = 0;
            memset(rxBuffer, 0, sizeof(rxBuffer));
            packetReady = false; // No complete packet available
            // Optionally, add an error log or indicator here
        }

        // Restart UART reception for the next byte
        HAL_UART_Receive_IT(&huart1, &rxByte, 1);
    }
}

bool UART_GetReceivedPacket(char *buffer, size_t buffer_size)
{
    if (packetReady)
    {
        // Ensure destination buffer is large enough
        size_t len = strlen(rxBuffer);
        if (len < buffer_size)
        {
            strncpy(buffer, rxBuffer, buffer_size - 1);
            buffer[buffer_size - 1] = '\0'; // Ensure null-termination

            // Reset internal buffer for next packet
            memset(rxBuffer, 0, sizeof(rxBuffer));
            rxIndex = 0;
            packetReady = false;
            return true;
        }
        else
        {
            // Destination buffer too small, discard internal packet
            memset(rxBuffer, 0, sizeof(rxBuffer));
            rxIndex = 0;
            packetReady = false;
            // Optionally, log an error
        }
    }
    return false; // Ensure to return false if no packet is ready
}

