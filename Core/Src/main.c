/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @developer      : Amiya Krishna Gupta
  * @start_date     : 11 August 2025
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lcd_i2c.h"   // I2C LCD driver
#include "rtc_i2c.h"   // RTC driver
#include "global.h"    // Global variables like motorStatus
#include "adc.h"       // ADC wrapper
#include "lora.h"      // LoRa driver
#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern SPI_HandleTypeDef hspi1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// LoRa Modes
#define LORA_MODE_TRANSMITTER 1
#define LORA_MODE_RECEIVER    2
#define LORA_MODE_TRANCEIVER  3 // Both Transmitter and Receiver
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c2;
RTC_HandleTypeDef hrtc;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char lcdBuffer[20];
uint8_t rxBuffer[32];        // LoRa RX
uint8_t txBuffer[32];        // LoRa TX
uint8_t connectionStatus = 0; // 0 = lost, 1 = OK
ADC_Data adcData;            // ADC readings

// Variable to control LoRa mode: 1=Transmitter, 2=Receiver, 3=Transceiver
uint8_t loraMode = LORA_MODE_RECEIVER; // Default to Transceiver mode
/* USER CODE END PV */
char errMsg[50];
/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static uint8_t z = 0;

/* USER CODE BEGIN 0 */
void Debug_Print(char *msg) {
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

  /* MCU Configuration */
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
//  MX_RTC_Init(); // RTC is commented out in original, keeping it that way
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */
  lcd_init();
  ADC_Init(&hadc1);
  LoRa_Init(); // Initialize LoRa module

  Debug_Print("System Initialized\r\n");
  uint8_t modem = LoRa_ReadReg(0x1D);
  uint8_t modem2 = LoRa_ReadReg(0x1E);
  char dbg[50];
  sprintf(dbg, "ModemCfg1=0x%02X, ModemCfg2=0x%02X\r\n", modem, modem2);
  Debug_Print(dbg);

  // Set initial LoRa mode
  if (loraMode == LORA_MODE_RECEIVER || loraMode == LORA_MODE_TRANCEIVER) {
      LoRa_SetRxContinuous(); // Start in RX mode if receiver or transceiver
      Debug_Print("LoRa set to RX Continuous mode.\r\n");
  } else {
      LoRa_SetStandby(); // Otherwise, start in Standby
      Debug_Print("LoRa set to Standby mode.\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1) {
      // === Verify LoRa chip ===
      uint8_t version = LoRa_ReadReg(0x42);  // SX1278 RegVersion
      if (version != 0x12) {
          z = 1;
          char errMsg[50];
          sprintf(errMsg, "LoRa not found! RegVersion=0x%02X\r\n", version);
          Debug_Print(errMsg);
          HAL_Delay(2000);
          continue; // retry until chip responds
      }

      switch (loraMode) {
          case LORA_MODE_TRANSMITTER:
              // Transmitter logic
              Debug_Print("LoRa Mode: Transmitter\r\n");
              uint8_t tx_msg[] = "HELLO_TX";
              z=5;
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
                      char dbg_rx[50];
                      sprintf(dbg_rx, "Received: %s\r\n", rxBuffer);
                      Debug_Print(dbg_rx);

                      if (strncmp((char*)rxBuffer, "PING", 4) == 0) {
                          // Step 2: Reply with "ACK"
                          uint8_t ack_msg[] = "ACK";
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
                  // Step 4: Ready to receive normal data
                  uint8_t rx_len = LoRa_ReceivePacket(rxBuffer);
                  if (rx_len > 0) {
                      rxBuffer[rx_len] = '\0';
                      char dbg_rx2[50];
                      sprintf(dbg_rx2, "Data Received: %s\r\n", rxBuffer);
                      Debug_Print(dbg_rx2);
                      z = 8;
                  }
              }

              HAL_Delay(100);
              break;


//          case LORA_MODE_RECEIVER:
//              // Receiver logic
//              Debug_Print("LoRa Mode: Receiver\r\n");
//              uint8_t rx_len = LoRa_ReceivePacket(rxBuffer);
//              if (rx_len > 0) {
//                  rxBuffer[rx_len] = '\0'; // null terminate
//                  char dbg_rx[50];
//                  sprintf(dbg_rx, "Received: %s\r\n", rxBuffer);
//                  Debug_Print(dbg_rx);
//              }
//              HAL_Delay(100); // Check for packets frequently
//              break;

          case LORA_MODE_TRANCEIVER:
              // Transceiver logic (send and receive)
              Debug_Print("LoRa Mode: Transceiver\r\n");

              // Try to receive first
              uint8_t rx_len_tr = LoRa_ReceivePacket(rxBuffer);
              if (rx_len_tr > 0) {
                  rxBuffer[rx_len_tr] = '\0'; // null terminate
                  char dbg_rx_tr[50];
                  sprintf(dbg_rx_tr, "Received: %s\r\n", rxBuffer);
                  Debug_Print(dbg_rx_tr);

                  // If "PING" is received, send "ACK"
                  if (strncmp((char*)rxBuffer, "PING", 4) == 0) {
                      uint8_t ack_msg[] = "ACK";
                      LoRa_SendPacket(ack_msg, sizeof(ack_msg) - 1);
                      Debug_Print("Sent: ACK\r\n");
                  }
              }

              // Then send a PING
              uint8_t tx_msg_tr[] = "PING";
              LoRa_SendPacket(tx_msg_tr, sizeof(tx_msg_tr) - 1);
              Debug_Print("Sent: PING\r\n");

              // Wait for ACK (max 500 ms)
              connectionStatus = 0;
              for (int i = 0; i < 20; i++) {   // 20 x 25ms = 500ms
                  uint8_t len = LoRa_ReceivePacket(rxBuffer);
                  if (len > 0) {
                      rxBuffer[len] = '\0'; // null terminate
                      char dbg_ack[50];
                      sprintf(dbg_ack, "Received ACK check: %s\r\n", rxBuffer);
                      Debug_Print(dbg_ack);

                      if (strncmp((char*)rxBuffer, "ACK", 3) == 0) {
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

      // === Display RTC (remains unchanged) ===
      Get_Time();
      sprintf(lcdBuffer, "%02d:%02d:%02d", time.hour, time.minutes, time.seconds);
      lcd_put_cur(0, 0);
      lcd_send_string(lcdBuffer);

      sprintf(lcdBuffer, "%02d-%02d-20%02d", time.dayofmonth, time.month, time.year);
      lcd_put_cur(1, 0);
      lcd_send_string(lcdBuffer);

      // The main loop delay is now handled within each case or at the end of the transceiver case.
      // If you want a consistent delay for all modes, place it here.
      // HAL_Delay(1000); // Example: delay for 1 second per loop iteration
  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    // Use HSI (8 MHz internal) with PLL
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2; // HSI/2 = 4 MHz
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;             // 4*16 = 64 MHz
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Updated line
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }

    // Use LSI (internal ~40 kHz) for RTC, and HSI/6 for ADC
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}



/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void) {
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef DateToUpdate = {0};

    hrtc.Instance = RTC;
    hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND; // Set the asynchronous prescaler
    hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM; // Set output source
    if (HAL_RTC_Init(&hrtc) != HAL_OK) {
        Debug_Print("RTC Init Failed\r\n");
        Error_Handler(); // Handle error
    }

    // Set the time and date
    sTime.Hours = 0x0;
    sTime.Minutes = 0x0;
    sTime.Seconds = 0x0;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
        Debug_Print("Set Time Failed\r\n");
        Error_Handler(); // Handle error
    }

    DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
    DateToUpdate.Month = RTC_MONTH_JANUARY;
    DateToUpdate.Date = 0x1;
    DateToUpdate.Year = 0x0;
    if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK) {
        Debug_Print("Set Date Failed\r\n");
        Error_Handler(); // Handle error
    }
}



/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Relay1_Pin|Relay2_Pin|Relay3_Pin|SWITCH4_Pin
                          |LORA_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_SELECT_GPIO_Port, LORA_SELECT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED5_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : AC_voltage_Pin AC_current_Pin */
  GPIO_InitStruct.Pin = AC_voltage_Pin|AC_current_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay1_Pin Relay2_Pin Relay3_Pin SWITCH4_Pin
                           LORA_STATUS_Pin LED4_Pin LED5_Pin */
  GPIO_InitStruct.Pin = Relay1_Pin|Relay2_Pin|Relay3_Pin|SWITCH4_Pin
                          |LORA_STATUS_Pin|LED4_Pin|LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SWITCH1_Pin SWITCH2_Pin SWITCH3_Pin RF_DATA_Pin */
  GPIO_InitStruct.Pin = SWITCH1_Pin|SWITCH2_Pin|SWITCH3_Pin|RF_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LORA_SELECT_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LORA_SELECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
