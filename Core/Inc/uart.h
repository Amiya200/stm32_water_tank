#ifndef __UART_H
#define __UART_H

#include "main.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h> // Add this line

#define UART_RX_BUFFER_SIZE 128
extern UART_HandleTypeDef huart1;
void UART_Init(void);
bool UART_GetReceivedPacket(char *buffer, size_t buffer_size);
void UART_TransmitString(UART_HandleTypeDef *huart, const char *str);
void UART_TransmitByte(UART_HandleTypeDef *huart, uint8_t byte);


// New function to check and retrieve a complete received packet
// Returns true if a complete packet is available, false otherwise.
// Copies the packet to 'buffer' and clears the internal buffer.
bool UART_GetReceivedPacket(char *buffer, size_t buffer_size);

// Removed: UART_ReadDataPacket as its functionality is better handled internally or by the caller.

#endif
