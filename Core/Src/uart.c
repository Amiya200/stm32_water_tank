#include "uart.h"
#include "global.h" // Include the global header for motorStatus

/**
  * @brief Transmits a null-terminated string over UART.
  * @param huart: Pointer to the UART handle (e.g., &huart1)
  * @param str: Pointer to the string to transmit
  * @retval None
  */
void UART_TransmitString(UART_HandleTypeDef *huart, const char *str)
{
    HAL_UART_Transmit(huart, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/**
  * @brief Transmits a single byte over UART.
  * @param huart: Pointer to the UART handle (e.g., &huart1)
  * @param byte: The byte to transmit
  * @retval None
  */
void UART_TransmitByte(UART_HandleTypeDef *huart, uint8_t byte)
{
    HAL_UART_Transmit(huart, &byte, 1, HAL_MAX_DELAY);
}

/**
  * @brief Reads a data packet into a buffer.
  * @param buffer: Pointer to the buffer where the data will be stored
  * @param data: Pointer to the data to be copied
  * @param size: Size of the data to be copied
  * @retval None
  */
void UART_ReadDataPacket(char *buffer, const char *data, size_t size)
{
    if (buffer != NULL && data != NULL && size > 0)
    {
        strncpy(buffer, data, size);
        buffer[size] = '\0'; // Null-terminate the string
    }
}

/**
  * @brief Receives a null-terminated string over UART.
  * @param huart: Pointer to the UART handle (e.g., &huart1)
  * @param buffer: Pointer to the buffer where the received string will be stored
  * @param size: Maximum size of the buffer
  * @retval None
  */
void UART_ReceiveString(UART_HandleTypeDef *huart, char *buffer, size_t size)
{
    HAL_UART_Receive(huart, (uint8_t *)buffer, size - 1, HAL_MAX_DELAY); // Leave space for null terminator
    buffer[size - 1] = '\0'; // Ensure null termination
}

/**
  * @brief Processes received data and updates the motor status.
  * @param buffer: Pointer to the buffer containing the received data
  * @retval None
  */
void UART_ProcessReceivedData(char *buffer)
{
    if (strcmp(buffer, "@MT1#") == 0)
    {
        // Set motor status to ON
        motorStatus = 1; // Now this will work as motorStatus is declared extern
    }
    else if (strcmp(buffer, "@MT0#") == 0)
    {
        // Set motor status to OFF
        motorStatus = 0; // Now this will work as motorStatus is declared extern
    }
}
