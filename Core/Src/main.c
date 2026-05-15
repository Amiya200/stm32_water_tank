/* ====================================================================
 * main.c  —  RECEIVER (motor-controller node)
 *
 * Dual-channel wireless:
 *   PRIMARY   — LoRa  Ra-02 SX1278  (bidirectional, ACK'd)
 *   SECONDARY — RF433 XY-MK-5V OOK  (simplex receive)
 *
 * Data source priority (evaluated every loop in adc.c):
 *   1. LoRa  — if LoRa_IsWirelessDataValid()   → inject LoRa data
 *   2. RF433 — else if RF_IsWirelessDataValid() → inject RF data
 *   3. Local — both links down                 → use physical probes
 *
 * Active channel is reported every STATUS_UPDATE_INTERVAL seconds.
 *
 * Timer usage
 *   TIM3 — reconfigured by RF_Init() to 1 MHz for RF bit-bang decode.
 *          LoRa_Task() uses only SPI + HAL_GetTick(), no TIM3.
 *
 * GPIO
 *   RF_DATA_Pin — INPUT (no pull).  Already correct in original code.
 *                 Connected to XY-MK-5V DATA output.
 * ==================================================================== */

#include "main.h"
#include "lcd_i2c.h"
#include "rtc_i2c.h"
#include "global.h"
#include "adc.h"
#include "lora.h"
#include "rf.h"
#include "uart.h"
#include "model_handle.h"
#include "screen.h"
#include "led.h"
#include "relay.h"
#include "acs712.h"
#include "device_id.h"
#include <stdio.h>
#include <string.h>

/* ── Private defines ────────────────────────────────────────────────── */
#define ADC_CHANNEL_COUNT        6
#define STATUS_UPDATE_INTERVAL   15000u   /* 15 s  */

/* ── Peripheral handles ─────────────────────────────────────────────── */
ADC_HandleTypeDef  hadc1;
I2C_HandleTypeDef  hi2c2;
RTC_HandleTypeDef  hrtc;
SPI_HandleTypeDef  hspi1;
TIM_HandleTypeDef  htim3;
UART_HandleTypeDef huart1;

/* ── Application data ───────────────────────────────────────────────── */
uint16_t adcBuffer[ADC_CHANNEL_COUNT];
float    g_adcAvg[ADC_CHANNEL_COUNT] = {0};
float    g_vADC_ACS  = 0.0f;
ADC_Data adcData;

extern float g_currentA;
extern float g_voltageV;

char     receivedUartPacket[UART_RX_BUFFER_SIZE];
bool     g_screenUpdatePending = false;
extern uint8_t loraMode;

/* ── Status timer ───────────────────────────────────────────────────── */
static uint32_t lastStatusUpdate = 0u;

/* ── External declarations ──────────────────────────────────────────── */
extern bool    Motor_GetStatus(void);
extern uint8_t g_loraConnected;

/* ── Private function prototypes ────────────────────────────────────── */
void SystemClock_Config(void);
static void MX_GPIO_Init       (void);
static void MX_ADC1_Init       (void);
static void MX_SPI1_Init       (void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init       (void);
static void MX_TIM3_Init       (void);

/* ── UART helpers ────────────────────────────────────────────────────── */
void Debug_Print(char *msg) { UART_TransmitString(&huart1, msg); }

void UART_PrintLn(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s,   (uint16_t)strlen(s), 1000u);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2u,               1000u);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) { /* reserved */ }
}

/* ── Helper: determine which channel is currently supplying data ─────── */
static const char *active_channel(void)
{
    if (LoRa_IsWirelessDataValid()) return "LORA";
    if (RF_IsWirelessDataValid())   return "RF-433";
    return "LOCAL-ADC";
}

/* ====================================================================
 *  main()
 * ==================================================================== */
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

    HAL_Delay(100u);

    UART_PrintLn("\r\n\r\n");
    UART_PrintLn("=========================================");
    UART_PrintLn("  HELONIX - RECEIVER (MOTOR CONTROLLER)");
    UART_PrintLn("  Firmware : Dual-Channel RX v5.0");
    UART_PrintLn("  Channels : LoRa Ra-02 + RF433 XY-MK-5V");
    UART_PrintLn("  UART     : 115200 8N1");
    UART_PrintLn("=========================================");

    RTC_Init();
    lcd_init();
    ADC_Init(&hadc1);

    /* ── LoRa init (primary channel) ─────────────────────────────── */
    LoRa_Init();

    /* ── RF433 init (secondary / backup channel) ─────────────────── *
     * RF_Init() reconfigures TIM3 to 1 MHz.                          *
     * LoRa_Task() uses SPI + HAL_GetTick() only — no TIM3 conflict.  */
    RF_Init();

    Screen_Init();
    UART_Init();
    Switches_Init();
    Relay_Init();
    LED_Init();
    ACS712_Init(&hadc1);

    HAL_Delay(100u);
    Timer_EEPROM_EnsureValid();

    ModelHandle_LoadSettingsFromEEPROM();
    ModelHandle_LoadAutoSettings();
    ModelHandle_LoadTimerFromEEPROM();
    ModelHandle_LoadModeState();
    ModelHandle_LoadCountdown();
    ModelHandle_LoadBuzzerSettings();
    HAL_Delay(50u);
    ModelHandle_OnPowerUp();
    RTC_GetTimeDate();

    loraMode = LORA_MODE_RECEIVER;

    UART_PrintLn("\r\n[MAIN] All systems initialized");
    UART_PrintLn("[MAIN] Data priority: LoRa > RF433 > Local ADC");
    UART_PrintLn("[MAIN] Entering main loop...\r\n");

    /* ================================================================
     *  Main loop
     *
     *  Step 1 : LoRa_Task()   — non-blocking IRQ poll, ACK/PONG reply
     *  Step 2 : RF_Task()     — check RF pin, decode if packet present
     *  Step 3 : New-data flags — trigger screen refresh on fresh data
     *  Step 4 : ACS712        — current / voltage update
     *  Step 5 : ADC           — read channels (wireless override inside)
     *  Step 6 : RTC           — time refresh
     *  Step 7 : UART commands — handle any incoming serial command
     *  Step 8 : Model process — motor FSM, auto/timer logic
     *  Step 9 : Screen + LED  — display & indicator update
     *  Step 10: Status print  — periodic channel status over UART
     *  Step 11: Loop delay    — 10 ms pace
     * ================================================================ */
    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* ── Step 1: LoRa service ───────────────────────────────────── *
         * Non-blocking: polls SX1278 IRQ register over SPI.           *
         * Sends ACK/PONG immediately when a valid packet is received.  */
        LoRa_Task();

        /* ── Step 2: RF433 receive ──────────────────────────────────── *
         * Non-blocking when pin is idle (<5 ms).                       *
         * Blocks up to ~600 ms when a valid preamble is detected —     *
         * acceptable because the motor FSM runs on a seconds timescale *
         * and LoRa already handles real-time updates.                  *
         *                                                               *
         * RF_Task() is called AFTER LoRa_Task() so that any pending   *
         * LoRa ACK is sent before we potentially block on RF decode.  */
        RF_Task();

        /* ── Step 3: new-data flags ─────────────────────────────────── */
        /* LoRa: flag set inside lora.c on every accepted packet        */
        if (g_loraNewPacketFlag)
        {
            g_loraNewPacketFlag   = false;
            g_screenUpdatePending = true;
        }

        /* RF433: detect transition from no-data → valid data           */
        {
            static bool s_rfWasValid = false;
            bool rf_now = RF_IsWirelessDataValid();
            if (!s_rfWasValid && rf_now)
                g_screenUpdatePending = true;
            s_rfWasValid = rf_now;
        }

        /* ── Step 4: ACS712 ──────────────────────────────────────────── */
        ACS712_Update();

        /* ── Step 5: ADC ─────────────────────────────────────────────── *
         * ADC_ReadAllChannels() applies the wireless override:         *
         *   if LoRa valid  → inject LoRa data  (CH0-3, CH5)           *
         *   else if RF valid → inject RF data                          *
         *   else             → keep raw local ADC readings             */
        ADC_ReadAllChannels(&hadc1, &adcData);

        /* ── Step 6: RTC ─────────────────────────────────────────────── */
        RTC_GetTimeDate();

        /* ── Step 7: UART command ────────────────────────────────────── */
        if (UART_GetReceivedPacket(receivedUartPacket, sizeof(receivedUartPacket)))
        {
            UART_HandleCommand(receivedUartPacket);
            g_screenUpdatePending = true;
        }

        /* ── Step 8: model process ───────────────────────────────────── */
        ModelHandle_CheckAutoTimerActivation();
        ModelHandle_Process();

        /* ── Step 9: screen + LED ────────────────────────────────────── */
        Screen_HandleSwitches();
        Screen_Update();
        LED_Task();

        /* ── Step 10: periodic status print ─────────────────────────── */
        if ((now - lastStatusUpdate) >= STATUS_UPDATE_INTERVAL)
        {
            lastStatusUpdate = now;

            /* Determine active source and tank level to display        */
            const char *src = active_channel();
            uint8_t     lvl = LoRa_IsWirelessDataValid()
                              ? LoRa_GetWirelessTankLevel()
                              : (RF_IsWirelessDataValid()
                                 ? RF_GetWirelessTankLevel()
                                 : 0u);
            uint8_t     wd  = LoRa_IsWirelessDataValid()
                              ? LoRa_GetWirelessWellDry()
                              : (RF_IsWirelessDataValid()
                                 ? RF_GetWirelessWellDry()
                                 : 0u);

            char status[160];
            snprintf(status, sizeof(status),
                "[STATUS] Src:%-9s | LoRa:%-4s | RF:%-3s | "
                "TL:%u%% | WD:%u | Motor:%s",
                src,
                LoRa_IsWirelessDataValid() ? "OK"  : "---",
                RF_IsWirelessDataValid()   ? "OK"  : "---",
                lvl, wd,
                Motor_GetStatus() ? "ON" : "OFF");
            UART_PrintLn(status);
        }

        /* ── Step 11: loop pace ──────────────────────────────────────── */
        HAL_Delay(10u);
    }
}

/* ====================================================================
 *  Peripheral init
 * ==================================================================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit     = {0};

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
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* PA15 = LoRa NSS — disable JTAG, keep SWD */
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    /* Safe defaults */
    HAL_GPIO_WritePin(GPIOB,
        Relay1_Pin | Relay2_Pin | Relay3_Pin | LORA_STATUS_Pin | LED4_Pin | LED5_Pin,
        GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA,
        LED1_Pin | LED2_Pin | LED3_Pin,
        GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_SELECT_GPIO_Port, LORA_SELECT_Pin, GPIO_PIN_SET);

    /* Relays + LEDs */
    GPIO_InitStruct.Pin   = Relay1_Pin | Relay2_Pin | Relay3_Pin |
                            LORA_STATUS_Pin | LED4_Pin | LED5_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Switches */
    GPIO_InitStruct.Pin  = SWITCH1_Pin | SWITCH2_Pin | SWITCH3_Pin | SWITCH4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* LEDs + LoRa NSS */
    GPIO_InitStruct.Pin   = LED1_Pin | LED2_Pin | LED3_Pin | LORA_SELECT_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Keep LoRa NSS HIGH */
    HAL_GPIO_WritePin(LORA_SELECT_GPIO_Port, LORA_SELECT_Pin, GPIO_PIN_SET);

    /* ── RF433 XY-MK-5V receiver data pin ──────────────────────────── *
     * INPUT, no pull.  The module drives the line; an internal pull   *
     * would distort the AGC-generated signal.                         *
     * No change from original — this block is kept here for clarity.  */
    GPIO_InitStruct.Pin  = RF_DATA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(RF_DATA_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    UART_PrintLn("[ERROR] Error_Handler called — system halted!");
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif
