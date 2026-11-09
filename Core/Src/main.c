/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t imu1_id = 0;                             // センサーから返ってきた識別値を保存する変数
volatile HAL_StatusTypeDef imu1_spi_status = HAL_ERROR;   // SPIの送受信処理が正常に終わったか保存

volatile uint8_t imu2_id = 0;                             
volatile HAL_StatusTypeDef imu2_spi_status = HAL_ERROR;

volatile uint8_t imu3_id = 0;
volatile HAL_StatusTypeDef imu3_spi_status = HAL_ERROR;

volatile uint8_t imu1_ctrl1 = 0;
volatile HAL_StatusTypeDef imu1_write_status = HAL_ERROR;
volatile HAL_StatusTypeDef imu1_read_status = HAL_ERROR;

volatile HAL_StatusTypeDef imu1_accel_config_status = HAL_ERROR;
volatile HAL_StatusTypeDef imu1_accel_status = HAL_ERROR;

uint8_t motion_tx[15] = {0x20 | 0x80};  //0x20は温度、0x22はジャイロセンサー、0x28は、加速度データが入っている最初のレジスタのアドレス 0x29とついていく
uint8_t motion_rx[15] = {0};

volatile uint32_t imu_tick = 0;

volatile int16_t temperture = 0;

volatile int16_t  imu1_ax = 0;
volatile int16_t  imu1_ay = 0;
volatile int16_t  imu1_az = 0;

volatile int16_t imu1_gx = 0;
volatile int16_t imu1_gy = 0;
volatile int16_t imu1_gz = 0;

static volatile uint8_t imu1_dma_busy = 0;
volatile uint32_t imu1_sample_count = 0;
volatile uint32_t imu1_dma_errors = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  if (htim ->Instance == TIM6) {

    HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_RESET);
    imu1_accel_status = HAL_SPI_TransmitReceive(&hspi2, motion_tx, motion_rx, 15, 100);
    HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_SET);

    if (imu1_accel_status == HAL_OK) {  // accel_rx[0]	命令送信中の受信値

      temperture = (int16_t)(((uint16_t)motion_rx[2] << 8 | motion_rx[1]));

      imu1_gx = (int16_t)(((uint16_t)motion_rx[4] << 8) | motion_rx[3]);
      imu1_gy = (int16_t)(((uint16_t)motion_rx[6] << 8) | motion_rx[5]);
      imu1_gz = (int16_t)(((uint16_t)motion_rx[8] << 8) | motion_rx[7]);

      imu1_ax = (int16_t)(((uint16_t)motion_rx[10]  << 8) | motion_rx[9]);
      imu1_ay = (int16_t)(((uint16_t)motion_rx[12] << 8) | motion_rx[11]);
      imu1_az = (int16_t)(((uint16_t)motion_rx[14] << 8) | motion_rx[13]);
    }
  }

}
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
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  //printf("hello world\n");
  HAL_Delay(100);                                             // 起動順位

  uint8_t tx[2] = {0x0F | 0x80, 0x00};                      // 送信する2バイト 0x80で読み取りの意味 0x0Fを読み取る 0x0Fは、センサーの識別値が入った`WHO_AM_I`レジスタのアドレス ,は配列の値区切り
  uint8_t rx[2] = {0};                                              // 受信する2バイト

  // /*IMU1_CS_GPIO_Port	GPIOB	Bポート IMU1_CS_Pin	GPIO_PIN_12	12番ピン*/
  // HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_RESET);           // IMU1を選択する 
  // imu1_spi_status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);// SPI2で2バイトを送信し、同時に2バイトを受信する &hspi2はspi2を意味している(どのspiか, 送信, 受信, 8bit設定より2バイト, 完了を待つ時間ms)
  // HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_SET);             // IMU1との通信を終了する

  // // 通信処理が正常に終わった場合、識別値を保存する 
  // if (imu1_spi_status == HAL_OK){
  //   imu1_id = rx[1];
  // }

  // HAL_GPIO_WritePin(IMU2_CS_GPIO_Port, IMU2_CS_Pin, GPIO_PIN_RESET); 
  // imu2_spi_status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);
  // HAL_GPIO_WritePin(IMU2_CS_GPIO_Port, IMU2_CS_Pin, GPIO_PIN_SET); 

  // if (imu2_spi_status == HAL_OK){
  //   imu2_id = rx[1];
  // }

  // HAL_GPIO_WritePin(IMU3_CS_GPIO_Port, IMU3_CS_Pin, GPIO_PIN_RESET); 
  // imu3_spi_status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);
  // HAL_GPIO_WritePin(IMU3_CS_GPIO_Port, IMU3_CS_Pin, GPIO_PIN_SET);

  // if (imu3_spi_status == HAL_OK){
  //   imu3_id = rx[1];
  // }

  // // CTRLへ設定を書き込む 0x06を書き込む120Hz 最上位bitが0なので0x10は書き込み命令で0x10レジスタに書き込む
  // tx[0] = 0x10;
  // tx[1] = 0x06;

  // HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_RESET);
  // imu1_write_status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);
  // HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_SET);

  // // CTRL1を読み返す    0x10 | 0x80 = 0x90 0x00は返答を受け取るためのダミー 0x90は最上位bitが1なので読み取り命令 0x10を読み取る
  // tx[0] = 0x10 | 0x80;
  // tx[1] = 0x00;

  // HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_RESET);
  // imu1_read_status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);
  // HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_SET);

  // if (imu1_read_status == HAL_OK){
  //   imu1_ctrl1 = rx[1];
  // }

  // tx[0] = 0x12;
  // tx[1] = 0x44;  //BDUとIF_INCを有効にする設定

  // HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_RESET);
  // imu1_accel_config_status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);
  // HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_SET);

  // if (imu1_accel_config_status != HAL_OK) {
  //   Error_Handler();
  // }

  tx[0] = 0x11;
  tx[1] = 0x06;

  HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef gyro_config_status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);
  HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_SET);

  if (gyro_config_status != HAL_OK){
    Error_Handler();
  }

  HAL_Delay(100);

  uint32_t last_imu_tick = 0;
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);

  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {  //タイマースタート
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(100);

    // uint32_t tick = imu_tick;

    // if (tick == last_imu_tick){  //タイマーから値が来たか確認　来ていたら値が増えるので !=であり、while内の動作が行われる
    //   continue;
    // }

    // last_imu_tick = tick;

    // uint8_t motion_tx[15] = {0x20 | 0x80};  //0x20は温度、0x22はジャイロセンサー、0x28は、加速度データが入っている最初のレジスタのアドレス 0x29とついていく
    // uint8_t motion_rx[15] = {0};
    // HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_RESET);
    // imu1_accel_status = HAL_SPI_TransmitReceive(&hspi2, motion_tx, motion_rx, 15, 100);
    // HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_SET);

    // if (imu1_accel_status == HAL_OK) {  // accel_rx[0]	命令送信中の受信値

    //   temperture = (int16_t)(((uint16_t)motion_rx[2] << 8 | motion_rx[1]));

    //   imu1_gx = (int16_t)(((uint16_t)motion_rx[4] << 8) | motion_rx[3]);
    //   imu1_gy = (int16_t)(((uint16_t)motion_rx[6] << 8) | motion_rx[5]);
    //   imu1_gz = (int16_t)(((uint16_t)motion_rx[8] << 8) | motion_rx[7]);

    //   imu1_ax = (int16_t)(((uint16_t)motion_rx[10]  << 8) | motion_rx[9]);
    //   imu1_ay = (int16_t)(((uint16_t)motion_rx[12] << 8) | motion_rx[11]);
    //   imu1_az = (int16_t)(((uint16_t)motion_rx[14] << 8) | motion_rx[13]);
    // }

    printf("temperture = %d imu1_gx = %d imu1_gy = %d imu1_gz = %d imu1_ax = %d imu1_ay = %d imu1_az = %d\n",temperture, imu1_gx, imu1_gy, imu1_gz, imu1_ax, imu1_ay, imu1_az);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 15999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IMU3_CS_Pin|IMU2_CS_Pin|IMU1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : IMU3_CS_Pin IMU2_CS_Pin IMU1_CS_Pin */
  GPIO_InitStruct.Pin = IMU3_CS_Pin|IMU2_CS_Pin|IMU1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
