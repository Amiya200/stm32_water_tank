/* ====================================================================
 * main.c  —  RECEIVER (motor-controller node) — ENHANCED
 *
 * IMPROVEMENTS:
 *  • Startup banner with device ID
 *  • Periodic connection status updates
 *  • Better debugging output
 * ==================================================================== */

#include "main.h"
#include "lcd_i2c.h"
#include "rtc_i2c.h"
#include "global.h"
#include "adc.h"
#include "lora.h"
#include "uart.h"
#include "model_handle.h"
#include "screen.h"
#include "led.h"
#include "relay.h"
#include <stdio.h>
#include <string.h>
#include "rf.h"
#include "stdio.h"
#include "acs712.h"
#include "device_id.h"

/* ── Private defines ────────────────────────────────────────────────── */
#define ADC_CHANNEL_COUNT 6
uint16_t adcBuffer[ADC_CHANNEL_COUNT];
#define ADC_BUFFER_SIZE ADC_CHANNEL_COUNT
float g_adcAvg[ADC_CHANNEL_COUNT] = {0};
float g_vADC_ACS = 0.0f;

extern float g_currentA;
extern float g_voltageV;

/* ── Private variables ──────────────────────────────────────────────── */
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c2;
RTC_HandleTypeDef hrtc;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

ADC_Data adcData;
char receivedUartPacket[UART_RX_BUFFER_SIZE];
int ak = 0;
bool g_screenUpdatePending = false;
extern uint8_t loraMode;

/* Status update timer */
static uint32_t lastStatusUpdate = 0;
#define STATUS_UPDATE_INTERVAL 15000  /* 15 seconds */

/* External function declarations */
extern bool Motor_GetStatus(void);

/* ── Private function prototypes ────────────────────────────────────── */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);

char dbg[64];
void Debug_Print(char *msg)
{
    UART_TransmitString(&huart1, msg);
}

void UART_PrintLn(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 1000);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 1000);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) { /* reserved */ }
}

/* ── Application entry point ────────────────────────────────────────── */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    MX_I2C2_Init();
    MX_TIM3_Init();

    /* Startup banner */
    HAL_Delay(100);
    UART_PrintLn("\r\n\r\n");
    UART_PrintLn("=========================================");
    UART_PrintLn("  HELONIX - RECEIVER (MOTOR CONTROLLER)");
    UART_PrintLn("  Firmware: v3.2 - LoRa Water Tank RX");
    UART_PrintLn("  UART: 115200 8N1");
    UART_PrintLn("=========================================");

    RTC_Init();
    lcd_init();
    ADC_Init(&hadc1);

    /* LoRa init with device ID display */
    LoRa_Init();  /* This prints its own banner */

    Screen_Init();
    UART_Init();
    Switches_Init();
    Relay_Init();
    LED_Init();
    ACS712_Init(&hadc1);

    HAL_Delay(100);
    Timer_EEPROM_EnsureValid();

    ModelHandle_LoadSettingsFromEEPROM();
    ModelHandle_LoadAutoSettings();
    ModelHandle_LoadTimerFromEEPROM();
    ModelHandle_LoadModeState();
    ModelHandle_LoadCountdown();
    ModelHandle_LoadBuzzerSettings();
    HAL_Delay(50);
    ModelHandle_OnPowerUp();
    RTC_GetTimeDate();

    /* Receiver always starts in RX mode */
    loraMode = LORA_MODE_RECEIVER;

    UART_PrintLn("\r\n[MAIN] All systems initialized");
    UART_PrintLn("[MAIN] Entering main loop...\r\n");

    while (1)
    {
        uint32_t now = HAL_GetTick();
        LoRa_Task();
        if (g_loraNewPacketFlag)
        {
            g_loraNewPacketFlag    = false;
            g_screenUpdatePending  = true;
        }
        ACS712_Update();
        ADC_ReadAllChannels(&hadc1, &adcData);
        RTC_GetTimeDate();
        if (UART_GetReceivedPacket(receivedUartPacket, sizeof(receivedUartPacket)))
        {
            UART_HandleCommand(receivedUartPacket);
            g_screenUpdatePending = true;
        }
        ModelHandle_CheckAutoTimerActivation();
        ModelHandle_Process();

        Screen_HandleSwitches();
        Screen_Update();
        LED_Task();

        if ((now - lastStatusUpdate) >= STATUS_UPDATE_INTERVAL)
        {
            lastStatusUpdate = now;

            char status[120];
            extern uint8_t g_loraConnected;

            snprintf(status, sizeof(status),
                     "[STATUS] LoRa: %s | Data: %s | TL: %u%% | Motor: %s",
                     g_loraConnected ? "CONNECTED" : "DISCONNECTED",
                     (LoRa_IsWirelessDataValid() ? "VALID" : "OFFLINE"),
                     LoRa_GetWirelessTankLevel(),
                     Motor_GetStatus() ? "ON" : "OFF");
            UART_PrintLn(status);
        }

        HAL_Delay(10);
    }
}

/* ── Clock configuration ────────────────────────────────────────────── */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState            = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC;
    PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSI;
    PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 8;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    uint32_t chList[] = {
        ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
        ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_7, ADC_CHANNEL_6
    };
    for (uint8_t r = 0; r < 8; r++)
    {
        sConfig.Channel      = chList[r];
        sConfig.Rank         = r + 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
    }
}

static void MX_I2C2_Init(void)
{
    hi2c2.Instance             = I2C2;
    hi2c2.Init.ClockSpeed      = 100000;
    hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1     = 0;
    hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2     = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 0xFFFF;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, Relay1_Pin | Relay2_Pin | Relay3_Pin |
                      LORA_STATUS_Pin | LED4_Pin | LED5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, LED1_Pin | LED2_Pin | LED3_Pin | LORA_SELECT_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = Relay1_Pin | Relay2_Pin | Relay3_Pin |
                            LORA_STATUS_Pin | LED4_Pin | LED5_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = SWITCH1_Pin | SWITCH2_Pin | SWITCH3_Pin | SWITCH4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = LED1_Pin | LED2_Pin | LED3_Pin | LORA_SELECT_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = RF_DATA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(RF_DATA_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    UART_PrintLn("[ERROR] Error_Handler called - system halted!");
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif
