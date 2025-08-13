#ifndef UART_H
#define UART_H

#include "stm32f1xx_hal.h" // Required for UART_HandleTypeDef
#include <string.h>        // Required for strlen

// Function prototypes
void UART_TransmitString(UART_HandleTypeDef *huart, const char *str);
void UART_TransmitByte(UART_HandleTypeDef *huart, uint8_t byte);
void UART_ReadDataPacket(char *buffer, const char *data, size_t size); // Function to read data packet
void UART_ReceiveString(UART_HandleTypeDef *huart, char *buffer, size_t size); // Function to receive a string
void UART_ProcessReceivedData(char *buffer); // Function to process received data

#endif // UART_H
