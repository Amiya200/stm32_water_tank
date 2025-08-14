/* =======================================================================
 *  WT1.1 – main.c (CLEANED for EXTERNAL RTC over I2C1)
 *  - Uses ONLY I2C1 for LCD + RTC (DS1307/DS3231 via rtc_i2c.c)
 *  - Removes STM32 internal HAL RTC usage/init
 *  - Adds missing SPI/I2C handle declarations
 *  - Provides MX_I2C1_Init() implementation
 *  - Calls MX_SPI1_Init() to avoid unused warnings
 * ======================================================================= */

#include "main.h"
#include "adc.h"
#include "lcd_i2c.h"
#include "led.h"
#include "relay.h"
#include "rtc_i2c.h"   // your external RTC driver header
#include "switches.h"
#include "uart.h"
#include <stdio.h>

/* ===================== Global handles ===================== */
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;      // I2C1 used for LCD + RTC
SPI_HandleTypeDef hspi1;      // add SPI handle (used by MX_SPI1_Init)
UART_HandleTypeDef huart1;    // UART1

/* ===================== Private prototypes ===================== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);     // IMPLEMENTED below
static void MX_SPI1_Init(void);     // IMPLEMENTED below
static void MX_USART1_UART_Init(void);

static void ReadAndDisplayRTC(void);

/* ===================== Private variables ===================== */
static RTC_TimeTypeDef sTime;   // used as simple containers by rtc_i2c
static RTC_DateTypeDef sDate;
static char uart_tx_buffer[64];

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();              // Only I2C1 is used
    MX_SPI1_Init();              // Call to avoid unused warnings; keep if you use SPI
    MX_USART1_UART_Init();

    LED_Init();
    Relay_Init();
    Switches_Init();

    // LCD at 0x27 (shifted left for HAL): pass 7-bit address shifted by 1
    LCD_I2C_Init(&hi2c1, 0x27 << 1);

    // Optional: if your rtc_i2c driver supports checking validity, set default time/date here.
    // Example (pseudo):
    // if (!RTC_I2C_IsTimeValid(&hi2c1)) {
    //     sTime.Hours = 12; sTime.Minutes = 0; sTime.Seconds = 0;
    //     RTC_I2C_SetTime(&hi2c1, &sTime);
    //     sDate.WeekDay = RTC_WEEKDAY_MONDAY; sDate.Month = RTC_MONTH_JANUARY; sDate.Date = 1; sDate.Year = 25;
    //     RTC_I2C_SetDate(&hi2c1, &sDate);
    // }

    while (1)
    {
        ReadAndDisplayRTC();
        HAL_Delay(1000);
    }
}

/* ===================== Read & Display RTC ===================== */
static void ReadAndDisplayRTC(void)
{
    // NOTE: Using external RTC via rtc_i2c.c — pass I2C handle, not HAL RTC handle
    if (RTC_I2C_GetTime(&hi2c1, &sTime))
    {
        if (RTC_I2C_GetDate(&hi2c1, &sDate))
        {
            // UART
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                     "Time: %02d:%02d:%02d, Date: %02d/%02d/%04d\r\n",
                     sTime.Hours, sTime.Minutes, sTime.Seconds,
                     sDate.Date, sDate.Month, 2000 + sDate.Year);
            UART_TransmitString(&huart1, uart_tx_buffer);

            // LCD
            LCD_I2C_SetCursor(0, 0);
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "%02d:%02d:%02d",
                     sTime.Hours, sTime.Minutes, sTime.Seconds);
            LCD_I2C_PrintString(uart_tx_buffer);

            LCD_I2C_SetCursor(0, 1);
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "%02d/%02d/%02d",
                     sDate.Date, sDate.Month, sDate.Year);
            LCD_I2C_PrintString(uart_tx_buffer);
        }
        else
        {
            UART_TransmitString(&huart1, "Failed to get RTC Date!\r\n");
        }
    }
    else
    {
        UART_TransmitString(&huart1, "Failed to get RTC Time!\r\n");
    }
}

/* ===================== Clock Config ===================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* ===================== ADC1 Init ===================== */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    // Your custom ADC init (if defined in adc.c)
    ADC_Init(&hadc1);
}

/* ===================== I2C1 Init ===================== */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

/* ===================== SPI1 Init ===================== */
static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

/* ===================== UART1 Init ===================== */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

/* ===================== GPIO Init ===================== */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOB, Relay1_Pin|Relay2_Pin|Relay3_Pin|SWITCH4_Pin |
                            LORA_STATUS_Pin|RF_DATA_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(LORA_SELECT_GPIO_Port, LORA_SELECT_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED5_Pin, GPIO_PIN_SET);

    // PC13
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Analog inputs on PA (adjust to your actual pins)
    GPIO_InitStruct.Pin = AC_voltage_Pin|AC_current_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Outputs on PB
    GPIO_InitStruct.Pin = Relay1_Pin|Relay2_Pin|Relay3_Pin|SWITCH4_Pin |
                          LORA_STATUS_Pin|RF_DATA_Pin|LED4_Pin|LED5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Inputs on PB
    GPIO_InitStruct.Pin = SWITCH1_Pin|SWITCH2_Pin|SWITCH3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Outputs on PA
    GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LORA_SELECT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* ===================== Error Handler ===================== */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        LED_Func_On();
        HAL_Delay(100);
        LED_Func_Off();
        HAL_Delay(100);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif

/* =======================================================================
 *  adc.c – REQUIRED PATCH for huart1 visibility
 *
 *  // at top of adc.c
 *  #include "uart.h"          // for UART_TransmitString / UART_ReceiveString
 *  extern UART_HandleTypeDef huart1; // declare the UART1 handle from main.c
 *
 *  // Now calls like UART_TransmitString(&huart1, ...) and
 *  // UART_ReceiveString(&huart1, ...) will compile.
 * ======================================================================= */
