/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "IS31FL3237.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union {
    unsigned char u8Byte;
    struct{
        unsigned bit0:1;
        unsigned bit1:1;
        unsigned bit2:1;
        unsigned bit3:1;
        unsigned bit4:1;
        unsigned bit5:1;
        unsigned bit6:1;
        unsigned bit7:1;
    } sb;
} xu_flag;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t Rx_Data_GetCMD_flag = {0},
        Rx_Data_cnt         = {0},
        Rx_Data[256]        = {0},
        Getchar[1]          = {0};

unsigned char isSW5_Headphone_button_Mute = 0;
unsigned char SW6_Pattern_button_ID       = 0;
unsigned char SW2_Preset_button_ID        = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void Polling_Task_1ms(void);
void Polling_Task_10ms(void);
void Polling_Task_50ms(void);
void Polling_Task_100ms(void);
void Polling_Task_1s(void);
void Task_ButtonGroup(void);
void VR1_PreGain(void);
void VR2_Microphone(void);
void VR3_Mix(void);
void VR4_Headphone(void);
void Task_Preset_LED_GPIO(void);
unsigned char Task_Button(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
unsigned char u8PresetLED = {0};

extern __IO uint32_t uwTick;
uint32_t             uwTick_Private;
unsigned short       uwTick_10ms  = 0;
unsigned short       uwTick_50ms  = 0;
unsigned short       uwTick_100ms = 0;
unsigned short       uwTick_1s    = 0;

uint32_t ADC_ConvertedValue[4] = {0};
#define  tor    8
#define  s0   372
#define  s1   745
#define  s2   1117
#define  s3   1489
#define  s4   1861
#define  s5   2234
#define  s6   2606
#define  s7   2978
#define  s8   3350
#define  s9   3723
#define ADC_A  ADC_ConvertedValue[0]
#define ADC_B  ADC_ConvertedValue[1]
#define ADC_C  ADC_ConvertedValue[2]
#define ADC_D  ADC_ConvertedValue[3]

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, ADC_ConvertedValue, 4);
  Hal_LED_Driver_Initial();

  //Init SW3/SW4 LED and LED9
  if(1) {
      LED9      = signle_LED_rgb_G;

      SW3_LED21 = signle_LED_rgb_G;
      SW3_LED19 = signle_LED_rgb_G;
      SW3_LED20 = signle_LED_rgb_G;

      SW4_LED18 = signle_LED_rgb_G;
      SW4_LED16 = signle_LED_rgb_G;
      SW4_LED15 = signle_LED_rgb_G;
  }

  printf("Init Done\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //Follow systick 1ms
    if(uwTick_Private != uwTick) {
        uwTick_Private++;

        Polling_Task_1ms();

        uwTick_10ms++;
        if(uwTick_10ms>=10) {
            uwTick_10ms = 0;

            Polling_Task_10ms();
        }

        uwTick_50ms++;
        if(uwTick_50ms>=50) {
            uwTick_50ms = 0;

            Polling_Task_50ms();
        }

        uwTick_100ms++;
        if(uwTick_100ms>=100) {
            uwTick_100ms = 0;

            Polling_Task_100ms();
        }

        uwTick_1s++;
        if(uwTick_1s>=1000) {
            uwTick_1s = 0;

            Polling_Task_1s();
        }

    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV10;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00602173;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /*
   * Target to : 64MHz / (64x50) = 20,000 Hz
   * Actually : 64MHz / ( (64+1) x (50+1) ) = 19,306.2 Hz
   */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  HAL_UART_Receive_IT(&huart2, Getchar, 1);
  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(1) {  //I2C LED Driver EN-GPIO
        #define ON  GPIO_PIN_SET
        #define OFF GPIO_PIN_RESET

        #define IS31FL3237_EN      GPIOA,GPIO_PIN_10
        #define IS31FL3237_EN_port GPIOA
        #define IS31FL3237_EN_pin  GPIO_PIN_10

        /* GPIO Ports Clock Enable */
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* Configure GPIO pin */
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

        /* Configure LED GPIO pin */
        GPIO_InitStruct.Pin = IS31FL3237_EN_pin; HAL_GPIO_Init(IS31FL3237_EN_port, &GPIO_InitStruct);

        /* Configure GPIO pin Output Level */
        HAL_GPIO_WritePin(IS31FL3237_EN, ON);
    }

    if(1) {
        #define ON  GPIO_PIN_SET
        #define OFF GPIO_PIN_RESET

        #define Preset_LED01      GPIOA,GPIO_PIN_4
        #define Preset_LED01_port GPIOA
        #define Preset_LED01_pin  GPIO_PIN_4

        #define Preset_LED02      GPIOA,GPIO_PIN_5
        #define Preset_LED02_port GPIOA
        #define Preset_LED02_pin  GPIO_PIN_5

        #define Preset_LED03      GPIOA,GPIO_PIN_11
        #define Preset_LED03_port GPIOA
        #define Preset_LED03_pin  GPIO_PIN_11

        #define Preset_LED04      GPIOA,GPIO_PIN_12
        #define Preset_LED04_port GPIOA
        #define Preset_LED04_pin  GPIO_PIN_12

        #define Preset_LED05      GPIOA,GPIO_PIN_15
        #define Preset_LED05_port GPIOA
        #define Preset_LED05_pin  GPIO_PIN_15

        #define Preset_LED06      GPIOB,GPIO_PIN_8
        #define Preset_LED06_port GPIOB
        #define Preset_LED06_pin  GPIO_PIN_8

        #define Preset_LED07      GPIOB,GPIO_PIN_9
        #define Preset_LED07_port GPIOB
        #define Preset_LED07_pin  GPIO_PIN_9

        #define Preset_LED08      GPIOA,GPIO_PIN_0
        #define Preset_LED08_port GPIOA
        #define Preset_LED08_pin  GPIO_PIN_0

        /* GPIO Ports Clock Enable */
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* Configure GPIO pin */
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

        /* Configure LED GPIO pin */
        GPIO_InitStruct.Pin = Preset_LED01_pin; HAL_GPIO_Init(Preset_LED01_port, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = Preset_LED02_pin; HAL_GPIO_Init(Preset_LED02_port, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = Preset_LED03_pin; HAL_GPIO_Init(Preset_LED03_port, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = Preset_LED04_pin; HAL_GPIO_Init(Preset_LED04_port, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = Preset_LED05_pin; HAL_GPIO_Init(Preset_LED05_port, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = Preset_LED06_pin; HAL_GPIO_Init(Preset_LED06_port, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = Preset_LED07_pin; HAL_GPIO_Init(Preset_LED07_port, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = Preset_LED08_pin; HAL_GPIO_Init(Preset_LED08_port, &GPIO_InitStruct);

        /* Configure GPIO pin Output Level */
        HAL_GPIO_WritePin(Preset_LED01, OFF);
        HAL_GPIO_WritePin(Preset_LED02, OFF);
        HAL_GPIO_WritePin(Preset_LED03, OFF);
        HAL_GPIO_WritePin(Preset_LED04, OFF);
        HAL_GPIO_WritePin(Preset_LED05, OFF);
        HAL_GPIO_WritePin(Preset_LED06, OFF);
        HAL_GPIO_WritePin(Preset_LED07, OFF);
        HAL_GPIO_WritePin(Preset_LED08, OFF);
    }

    if(1) {
        //#define Presets_Switch       GPIOA,GPIO_PIN_13
        //#define Presets_Switch_port  GPIOA
        //#define Presets_Switch_pin   GPIO_PIN_13

        #define Pattern_Switch       GPIOC,GPIO_PIN_6
        #define Pattern_Switch_port  GPIOC
        #define Pattern_Switch_pin   GPIO_PIN_6

        #define Headphone_Mute       GPIOA,GPIO_PIN_9
        #define Headphone_Mute_port  GPIOA
        #define Headphone_Mute_pin   GPIO_PIN_9

        #define Mix_Mute             GPIOA,GPIO_PIN_8
        #define Mix_Mute_port        GPIOA
        #define Mix_Mute_pin         GPIO_PIN_8

        #define Mic_Mute             GPIOB,GPIO_PIN_2
        #define Mic_Mute_port        GPIOB
        #define Mic_Mute_pin         GPIO_PIN_2

        /* GPIO Ports Clock Enable */
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        /* Configure Button pin */
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

        /*Configure GPIO pin : Mic_Mute_Pin */
        //GPIO_InitStruct.Pin  = Presets_Switch_pin; HAL_GPIO_Init(Presets_Switch_port, &GPIO_InitStruct);
        GPIO_InitStruct.Pin  = Pattern_Switch_pin; HAL_GPIO_Init(Pattern_Switch_port, &GPIO_InitStruct);
        GPIO_InitStruct.Pin  = Headphone_Mute_pin; HAL_GPIO_Init(Headphone_Mute_port, &GPIO_InitStruct);
        GPIO_InitStruct.Pin  = Mix_Mute_pin;       HAL_GPIO_Init(Mix_Mute_port,       &GPIO_InitStruct);
        GPIO_InitStruct.Pin  = Mic_Mute_pin;       HAL_GPIO_Init(Mic_Mute_port,       &GPIO_InitStruct);

        /* Configure GPIO pin */
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

        /* Configure LED GPIO pin */
        GPIO_InitStruct.Pin = Preset_LED01_pin; HAL_GPIO_Init(Preset_LED01_port, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = Preset_LED02_pin; HAL_GPIO_Init(Preset_LED02_port, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = Preset_LED03_pin; HAL_GPIO_Init(Preset_LED03_port, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = Preset_LED04_pin; HAL_GPIO_Init(Preset_LED04_port, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = Preset_LED05_pin; HAL_GPIO_Init(Preset_LED05_port, &GPIO_InitStruct);
    }
}

/* USER CODE BEGIN 4 */
void Polling_Task_1ms(void) {
    //Execute per 1ms
    if(1) {
        static uint8_t Rx_Data_cnt_Private = {0};
        if(Rx_Data_cnt_Private != Rx_Data_cnt) {

            //printf("%c", Rx_Data[Rx_Data_cnt_Private]);
            HAL_UART_Transmit(&huart2, (uint8_t *)&Rx_Data[Rx_Data_cnt_Private], 1, 0xFFFFFFFF);
            HAL_UART_Receive_IT(&huart2, Getchar, 1);

            switch(Rx_Data[Rx_Data_cnt_Private++]) {
                case '\r':
                case '\n':
                case '\a':
                case '\t':
                    Rx_Data_GetCMD_flag = 1;
                    break;

                default:
                    break;
            }
        }
    }
}

void Polling_Task_10ms(void) {
    Task_LED_Driver();
    Task_ButtonGroup();

    //Update GPIO_LED
    Task_Preset_LED_GPIO();
}

void Polling_Task_50ms(void) {

}

void Polling_Task_100ms(void) {
    if(SW6_Pattern_button_ID) {

    } else {
        VR1_PreGain();
    }

    VR2_Microphone();
    VR3_Mix();

    if(isSW5_Headphone_button_Mute) {

    } else {
        VR4_Headphone();
    }
}

void Polling_Task_1s(void) {
    printf("DMA-ADC:%ld,%ld,%ld,%ld\r\n", ADC_A, ADC_B, ADC_C, ADC_D);
}

void Task_ButtonGroup(void) {
    static unsigned char btnCNT[5] = {0};
    unsigned char Task_Button_temp = Task_Button();
    static const unsigned char ExecuteBTNevent  = 10,
                               HoldOnExecuteBTN = 11;

    if(Task_Button_temp & 0x01) {
        if(btnCNT[0]==ExecuteBTNevent) {
            btnCNT[0]=HoldOnExecuteBTN;

        } else if(btnCNT[0]==HoldOnExecuteBTN) {
            //Don't do anything
        } else {
            btnCNT[0]++;
        }
    } else {
        btnCNT[0] = 0;
    }

    if(Task_Button_temp & 0x02) {
        if(btnCNT[1]==ExecuteBTNevent) {
            btnCNT[1]=HoldOnExecuteBTN;

            //Get SW6 Pattern button event
            SW6_Pattern_button_ID++;
            if(SW6_Pattern_button_ID>=11) {
                SW6_Pattern_button_ID = 0;
            }
            switch(SW6_Pattern_button_ID) {
                default:
                case  0:  /* Don't do anything */                                                                  break;

                case  1: SW6_LED12 = signle_LED_rgb_R; SW6_LED10 = 0;                SW6_LED11 = 0;                break;
                case  2: SW6_LED12 = 0;                SW6_LED10 = signle_LED_rgb_R; SW6_LED11 = 0;                break;
                case  3: SW6_LED12 = 0;                SW6_LED10 = 0;                SW6_LED11 = signle_LED_rgb_R; break;

                case  4: SW6_LED12 = signle_LED_rgb_G; SW6_LED10 = 0;                SW6_LED11 = 0;                break;
                case  5: SW6_LED12 = 0;                SW6_LED10 = signle_LED_rgb_G; SW6_LED11 = 0;                break;
                case  6: SW6_LED12 = 0;                SW6_LED10 = 0;                SW6_LED11 = signle_LED_rgb_G; break;

                case  7: SW6_LED12 = signle_LED_rgb_B; SW6_LED10 = 0;                SW6_LED11 = 0;                break;
                case  8: SW6_LED12 = 0;                SW6_LED10 = signle_LED_rgb_B; SW6_LED11 = 0;                break;
                case  9: SW6_LED12 = 0;                SW6_LED10 = 0;                SW6_LED11 = signle_LED_rgb_B; break;

                case 10: SW6_LED12 = 0;                SW6_LED10 = 0;                SW6_LED11 = 0;                break;
            }

            //Get SW2 Preset button event
            SW2_Preset_button_ID++;
            if(SW2_Preset_button_ID>8) {
                SW2_Preset_button_ID = 0;
            }
            u8PresetLED = 1<<SW2_Preset_button_ID;

            //Check LED9
            switch(LED9) {
            	default:
            	case 0:                LED9 = signle_LED_rgb_R; break;
            	case signle_LED_rgb_R: LED9 = signle_LED_rgb_G; break;
            	case signle_LED_rgb_G: LED9 = signle_LED_rgb_B; break;
            	case signle_LED_rgb_B: LED9 = 0;                break;
            }

        } else if(btnCNT[1]==HoldOnExecuteBTN) {
            //Don't do anything
        } else {
            btnCNT[1]++;
        }
    } else {
        btnCNT[1] = 0;
    }

    if(Task_Button_temp & 0x04) {
        if(btnCNT[2]==ExecuteBTNevent) {
            btnCNT[2]=HoldOnExecuteBTN;

            //Get SW5 Headphone button event
            isSW5_Headphone_button_Mute^=1;
            if(isSW5_Headphone_button_Mute) {
                SW5_LED17 = signle_LED_rgb_R;
                SW5_LED13 = signle_LED_rgb_R;
                SW5_LED14 = signle_LED_rgb_R;
            }

        } else if(btnCNT[2]==HoldOnExecuteBTN) {
            //Don't do anything
        } else {
            btnCNT[2]++;
        }
    } else {
        btnCNT[2] = 0;
    }

    if(Task_Button_temp & 0x08) {
        if(btnCNT[3]==ExecuteBTNevent) {
            btnCNT[3]=HoldOnExecuteBTN;

            //Get SW4 MIX button event
            static char LED_Toggle = 0;
            LED_Toggle^=1;
            if(LED_Toggle) {
                SW4_LED18 = signle_LED_rgb_R;
                SW4_LED16 = signle_LED_rgb_R;
                SW4_LED15 = signle_LED_rgb_R;
            } else {
                SW4_LED18 = signle_LED_rgb_G;
                SW4_LED16 = signle_LED_rgb_G;
                SW4_LED15 = signle_LED_rgb_G;
            }

        } else if(btnCNT[3]==HoldOnExecuteBTN) {
            //Don't do anything
        } else {
            btnCNT[3]++;
        }
    } else {
        btnCNT[3] = 0;
    }

    if(Task_Button_temp & 0x10) {
        if(btnCNT[4]==ExecuteBTNevent) {
            btnCNT[4]=HoldOnExecuteBTN;

            //Get SW3 MIC button event
            static char LED_Toggle = 0;
            LED_Toggle^=1;
            if(LED_Toggle) {
                SW3_LED21 = signle_LED_rgb_R;
                SW3_LED19 = signle_LED_rgb_R;
                SW3_LED20 = signle_LED_rgb_R;
            } else {
                SW3_LED21 = signle_LED_rgb_G;
                SW3_LED19 = signle_LED_rgb_G;
                SW3_LED20 = signle_LED_rgb_G;
            }

        } else if(btnCNT[4]==HoldOnExecuteBTN) {
            //Don't do anything
        } else {
            btnCNT[4]++;
        }
    } else {
        btnCNT[4] = 0;
    }
}

void VR1_PreGain(void) {
        static uint32_t PG_VR = 0;
        PG_VR = (PG_VR*3 + ADC_B*7)/10;

               if(                  PG_VR<=s0-tor) {
            //Seg 0
            SW6_LED11 = signle_LED_rgb_R;
            SW6_LED10 = signle_LED_rgb_R;
            SW6_LED12 = signle_LED_rgb_R;
        } else if(s0+tor<=PG_VR && PG_VR<=s1-tor) {
            //Seg 1
            SW6_LED11 = signle_LED_rgb_R + signle_LED_rgb_G;
            SW6_LED10 = signle_LED_rgb_R;
            SW6_LED12 = signle_LED_rgb_R;
        } else if(s1+tor<=PG_VR && PG_VR<=s2-tor) {
            //Seg 2
            SW6_LED11 = signle_LED_rgb_G;
            SW6_LED10 = signle_LED_rgb_R + signle_LED_rgb_G;
            SW6_LED12 = signle_LED_rgb_R;
        } else if(s2+tor<=PG_VR && PG_VR<=s3-tor) {
            //Seg 3
            SW6_LED11 = signle_LED_rgb_G;
            SW6_LED10 = signle_LED_rgb_G;
            SW6_LED12 = signle_LED_rgb_R + signle_LED_rgb_G;
        } else if(s3+tor<=PG_VR && PG_VR<=s4-tor) {
            //Seg 4
            SW6_LED11 = signle_LED_rgb_G;
            SW6_LED10 = signle_LED_rgb_G;
            SW6_LED12 = signle_LED_rgb_G;
        } else if(s4+tor<=PG_VR && PG_VR<=s5-tor) {
            //Seg 5
            SW6_LED11 = signle_LED_rgb_G + signle_LED_rgb_B;
            SW6_LED10 = signle_LED_rgb_G;
            SW6_LED12 = signle_LED_rgb_G;
        } else if(s5+tor<=PG_VR && PG_VR<=s6-tor) {
            //Seg 6
            SW6_LED11 = signle_LED_rgb_B;
            SW6_LED10 = signle_LED_rgb_G + signle_LED_rgb_B;
            SW6_LED12 = signle_LED_rgb_G;
        } else if(s6+tor<=PG_VR && PG_VR<=s7-tor) {
            //Seg 7
            SW6_LED11 = signle_LED_rgb_B;
            SW6_LED10 = signle_LED_rgb_B;
            SW6_LED12 = signle_LED_rgb_G + signle_LED_rgb_B;
        } else if(s7+tor<=PG_VR && PG_VR<=s8-tor) {
            //Seg 8
            SW6_LED11 = signle_LED_rgb_B;
            SW6_LED10 = signle_LED_rgb_B;
            SW6_LED12 = signle_LED_rgb_B;
        } else if(s8+tor<=PG_VR && PG_VR<=s9-tor) {
            //Seg 9
            SW6_LED11 = signle_LED_rgb_G + signle_LED_rgb_B;
            SW6_LED10 = signle_LED_rgb_G + signle_LED_rgb_B;
            SW6_LED12 = signle_LED_rgb_G + signle_LED_rgb_B;
        } else if(s9+tor<=PG_VR) {
            //Seg 10
            SW6_LED11 = signle_LED_rgb_R + signle_LED_rgb_G + signle_LED_rgb_B;
            SW6_LED10 = signle_LED_rgb_R + signle_LED_rgb_G + signle_LED_rgb_B;
            SW6_LED12 = signle_LED_rgb_R + signle_LED_rgb_G + signle_LED_rgb_B;
        }
}

void VR2_Microphone(void) {
    static uint32_t Mic_VR = 0;
    Mic_VR = (Mic_VR*3 + ADC_C*7)/10;

           if(                  Mic_VR<=s0-tor) {
        //Seg 0
        arrayLED[1] = 0x000;
    } else if(s0+tor<=Mic_VR && Mic_VR<=s1-tor) {
        //Seg 1
        arrayLED[1] = 0x001;
    } else if(s1+tor<=Mic_VR && Mic_VR<=s2-tor) {
        //Seg 2
        arrayLED[1] = 0x003;
    } else if(s2+tor<=Mic_VR && Mic_VR<=s3-tor) {
        //Seg 3
        arrayLED[1] = 0x007;
    } else if(s3+tor<=Mic_VR && Mic_VR<=s4-tor) {
        //Seg 4
        arrayLED[1] = 0x00F;
    } else if(s4+tor<=Mic_VR && Mic_VR<=s5-tor) {
        //Seg 5
        arrayLED[1] = 0x01F;
    } else if(s5+tor<=Mic_VR && Mic_VR<=s6-tor) {
        //Seg 6
        arrayLED[1] = 0x03F;
    } else if(s6+tor<=Mic_VR && Mic_VR<=s7-tor) {
        //Seg 7
        arrayLED[1] = 0x07F;
    } else if(s7+tor<=Mic_VR && Mic_VR<=s8-tor) {
        //Seg 8
        arrayLED[1] = 0x0FF;
    } else if(s8+tor<=Mic_VR && Mic_VR<=s9-tor) {
        //Seg 9
        arrayLED[1] = 0x1FF;
    } else if(s9+tor<=Mic_VR) {
        //Seg 10
        arrayLED[1] = 0x3FF;
    }
}

void VR3_Mix(void) {
        static uint32_t Mix_VR = 0;
        Mix_VR = (Mix_VR*3 + ADC_A*7)/10;

               if(                  Mix_VR<=s0-tor) {
            //Seg 0
            arrayLED[0] = 0x000;
        } else if(s0+tor<=Mix_VR && Mix_VR<=s1-tor) {
            //Seg 1
            arrayLED[0] = 0x001;
        } else if(s1+tor<=Mix_VR && Mix_VR<=s2-tor) {
            //Seg 2
            arrayLED[0] = 0x003;
        } else if(s2+tor<=Mix_VR && Mix_VR<=s3-tor) {
            //Seg 3
            arrayLED[0] = 0x007;
        } else if(s3+tor<=Mix_VR && Mix_VR<=s4-tor) {
            //Seg 4
            arrayLED[0] = 0x00F;
        } else if(s4+tor<=Mix_VR && Mix_VR<=s5-tor) {
            //Seg 5
            arrayLED[0] = 0x01F;
        } else if(s5+tor<=Mix_VR && Mix_VR<=s6-tor) {
            //Seg 6
            arrayLED[0] = 0x03F;
        } else if(s6+tor<=Mix_VR && Mix_VR<=s7-tor) {
            //Seg 7
            arrayLED[0] = 0x07F;
        } else if(s7+tor<=Mix_VR && Mix_VR<=s8-tor) {
            //Seg 8
            arrayLED[0] = 0x0FF;
        } else if(s8+tor<=Mix_VR && Mix_VR<=s9-tor) {
            //Seg 9
            arrayLED[0] = 0x1FF;
        } else if(s9+tor<=Mix_VR) {
            //Seg 10
            arrayLED[0] = 0x3FF;
        }
}

void VR4_Headphone(void) {
        static uint32_t HP_VR = 0;
        HP_VR = (HP_VR*3 + ADC_D*7)/10;

               if(                  HP_VR<=s0-tor) {
            //Seg 0
            SW5_LED14 = signle_LED_rgb_R;
            SW5_LED13 = signle_LED_rgb_R;
            SW5_LED17 = signle_LED_rgb_R;
        } else if(s0+tor<=HP_VR && HP_VR<=s1-tor) {
            //Seg 1
            SW5_LED14 = signle_LED_rgb_R + signle_LED_rgb_G;
            SW5_LED13 = signle_LED_rgb_R;
            SW5_LED17 = signle_LED_rgb_R;
        } else if(s1+tor<=HP_VR && HP_VR<=s2-tor) {
            //Seg 2
            SW5_LED14 = signle_LED_rgb_G;
            SW5_LED13 = signle_LED_rgb_R + signle_LED_rgb_G;
            SW5_LED17 = signle_LED_rgb_R;
        } else if(s2+tor<=HP_VR && HP_VR<=s3-tor) {
            //Seg 3
            SW5_LED14 = signle_LED_rgb_G;
            SW5_LED13 = signle_LED_rgb_G;
            SW5_LED17 = signle_LED_rgb_R + signle_LED_rgb_G;
        } else if(s3+tor<=HP_VR && HP_VR<=s4-tor) {
            //Seg 4
            SW5_LED14 = signle_LED_rgb_G;
            SW5_LED13 = signle_LED_rgb_G;
            SW5_LED17 = signle_LED_rgb_G;
        } else if(s4+tor<=HP_VR && HP_VR<=s5-tor) {
            //Seg 5
            SW5_LED14 = signle_LED_rgb_G + signle_LED_rgb_B;
            SW5_LED13 = signle_LED_rgb_G;
            SW5_LED17 = signle_LED_rgb_G;
        } else if(s5+tor<=HP_VR && HP_VR<=s6-tor) {
            //Seg 6
            SW5_LED14 = signle_LED_rgb_B;
            SW5_LED13 = signle_LED_rgb_G + signle_LED_rgb_B;
            SW5_LED17 = signle_LED_rgb_G;
        } else if(s6+tor<=HP_VR && HP_VR<=s7-tor) {
            //Seg 7
            SW5_LED14 = signle_LED_rgb_B;
            SW5_LED13 = signle_LED_rgb_B;
            SW5_LED17 = signle_LED_rgb_G + signle_LED_rgb_B;
        } else if(s7+tor<=HP_VR && HP_VR<=s8-tor) {
            //Seg 8
            SW5_LED14 = signle_LED_rgb_B;
            SW5_LED13 = signle_LED_rgb_B;
            SW5_LED17 = signle_LED_rgb_B;
        } else if(s8+tor<=HP_VR && HP_VR<=s9-tor) {
            //Seg 9
            SW5_LED14 = signle_LED_rgb_G + signle_LED_rgb_B;
            SW5_LED13 = signle_LED_rgb_G + signle_LED_rgb_B;
            SW5_LED17 = signle_LED_rgb_G + signle_LED_rgb_B;
        } else if(s9+tor<=HP_VR) {
            //Seg 10
            SW5_LED14 = signle_LED_rgb_R + signle_LED_rgb_G + signle_LED_rgb_B;
            SW5_LED13 = signle_LED_rgb_R + signle_LED_rgb_G + signle_LED_rgb_B;
            SW5_LED17 = signle_LED_rgb_R + signle_LED_rgb_G + signle_LED_rgb_B;
        }
}

void Task_Preset_LED_GPIO(void) {
    static xu_flag u8PresetLED_Save = {0};

    if(u8PresetLED_Save.u8Byte != u8PresetLED) {
        u8PresetLED_Save.u8Byte = u8PresetLED;
        HAL_GPIO_WritePin(Preset_LED01, u8PresetLED_Save.sb.bit0);
        HAL_GPIO_WritePin(Preset_LED02, u8PresetLED_Save.sb.bit1);
        HAL_GPIO_WritePin(Preset_LED03, u8PresetLED_Save.sb.bit2);
        HAL_GPIO_WritePin(Preset_LED04, u8PresetLED_Save.sb.bit3);
        HAL_GPIO_WritePin(Preset_LED05, u8PresetLED_Save.sb.bit4);
        HAL_GPIO_WritePin(Preset_LED06, u8PresetLED_Save.sb.bit5);
        HAL_GPIO_WritePin(Preset_LED07, u8PresetLED_Save.sb.bit6);
        HAL_GPIO_WritePin(Preset_LED08, u8PresetLED_Save.sb.bit7);
    }
}

unsigned char Task_Button(void) {
    xu_flag xfButton = {0};

    //xfButton.sb.bit0 = !HAL_GPIO_ReadPin(Presets_Switch);
    xfButton.sb.bit1 = !HAL_GPIO_ReadPin(Pattern_Switch);
    xfButton.sb.bit2 = !HAL_GPIO_ReadPin(Headphone_Mute);
    xfButton.sb.bit3 = !HAL_GPIO_ReadPin(Mix_Mute);
    xfButton.sb.bit4 = !HAL_GPIO_ReadPin(Mic_Mute);

    return xfButton.u8Byte;
}

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
}

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */

  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFFFFFF);
  return ch;
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file.
   */
  Rx_Data[Rx_Data_cnt] = Getchar[0];
  Rx_Data_cnt++;

  HAL_UART_Receive_IT(&huart2, Getchar, 1);
}
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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
