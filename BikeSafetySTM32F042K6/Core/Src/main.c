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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
HAL_I2C_StateTypeDef i2cState;		// State of i2c
HAL_StatusTypeDef i2cStatus;		// Status of i2c
HAL_StatusTypeDef spiStatus;		// Status of spi
HAL_StatusTypeDef uartStatus;		// Status of uart
HAL_StatusTypeDef uart2Status;		// Status of uart2

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint32_t counter 		= 0;
uint32_t counter2 		= 0;


// MPU9250 Variables
uint32_t finalXAccValue 			= 0;
uint32_t finalXAccValueWithOffset 	= 0;
uint32_t finalYAccValue 			= 0;
uint32_t finalYAccValueWithOffset 	= 0;
uint32_t finalZAccValue 			= 0;
uint32_t finalZAccValueWithOffset 	= 0;

uint32_t timerVal 	= 0;
uint8_t clockCykles = 0;

uint32_t refXAccValueWithOffset = 0;
uint32_t refYAccValueWithOffset = 0;
uint32_t refZAccValueWithOffset = 0;

uint8_t lockedDevice = 0;

uint8_t dataReceiveI2cBuffer 	= 0;	// MP9250
uint8_t receiveUARTData[30] 	= {0};	// RFID
uint8_t UARTDataKey[30] 		= {2,51,66,48,48,50,54,48,66,53,57,52,70,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	// RFID KEY
uint8_t receiveUART2Data[150] 	= {0};	// GSM/GNSS

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	//=========================MPU9250
	// Registers
7	uint8_t IMUDevAddr 				= 0xd0;
	uint8_t PWR_MGMT_1[2] 			= {0x6b, 0b00100000};	// or 4
	uint8_t PWR_MGMT_2[2] 			= {0x6c, 0b00000000};	// 0 to enable all or 255 to disable all
	uint8_t WHO_AM_I 				= 0x75;
	uint8_t LP_ACCEL_ODR[2] 		= {0x1e, 0b00001000}; 	// 8 = output frequency 62.50Hz
	uint8_t ACCEL_CONFIG[2] 		= {0x1c, 0x0}; 			// 0x0 for 2g, 0x8 for 4g, 0x10 for 8g,0x18 for 16g
	uint8_t ACCEL_XOUT_L 			= 0x3c;
	uint8_t ACCEL_XOUT_H 			= 0x3b;
	uint8_t ACCEL_YOUT_L 			= 0x3e;
	uint8_t ACCEL_YOUT_H 			= 0x3d;
	uint8_t ACCEL_ZOUT_L 			= 0x40;
	uint8_t ACCEL_ZOUT_H 			= 0x3f;
	//=========================MPU9250

	//=========================GSM
	uint8_t AT[] 					= "AT\r";
	uint8_t ATI[] 					= "ATI\r";
	uint8_t AT_CFUN[] 				= "AT+CFUN=1\r";			// Full Functionality Configuration
	uint8_t AT_COPS_OPCHK[] 		= "AT+COPS=?\r";			// Returns all operators available
	uint8_t AT_COPS_CRNT[] 			= "AT+COPS?\r";				// Returns current operator
	uint8_t AT_COPS_RGSTR[] 		= "AT+COPS=0\r";			// Register to operator network AT+COPS=<mode>,[<format>[,<oper>]]
	uint8_t AT_CMGF[] 				= "AT+CMGF=1\r";			// Set to text mode
	uint8_t AT_CSCS[] 				= "AT+CSCS=\"GSM\"\r";		// Set character
	uint8_t AT_CSQ[] 				= "AT+CSQ\r";				// Get Signal Strength in dBm
	uint8_t AT_CPOWD[] 				= "AT+CPOWD=1\r";			// Power OFF Modem
	uint8_t AT_CMGS_SEND_CTRLZ[] 	= "\x1a";					// Send Control
	uint8_t AT_CMGS_SEND_MSG_BUF[] 	= "AT+CMGS=\"+358443xxxxxx\"\rTesting9";
	//=========================GSM

	//=========================GNSS
	uint8_t AT_CGNSPWR_ON[] 	= "AT+CGNSPWR=1\r";				// GNSS turns Power ON
	uint8_t AT_CGNSPWR_OFF[] 	= "AT+CGNSPWR=0\r";				// GNSS turns Power OFF
	uint8_t AT_CGNSSEQ[] 		= "AT+CGNSSEQ=\"RMC\"\r";		// RMC for GGA
	uint8_t AT_CGNSINF[] 		= "AT+CGNSINF\r";				// Gets data from GNSS
	uint8_t AT_CGNSURC[] 		= "AT+CGNSURC=1\r";
	//=========================GNSS

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  /*
  //=========================GSM
  uart2Status = HAL_UART_Transmit(&huart2, AT_COPS_RGSTR, sizeof(AT_COPS_RGSTR), 1000);
  uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 1000);
  HAL_Delay(10);
  uart2Status = HAL_UART_Transmit(&huart2, AT_COPS_CRNT, sizeof(AT_COPS_CRNT), 1000);
  uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 1000);
  HAL_Delay(5000);
  uart2Status = HAL_UART_Transmit(&huart2, AT_CMGF, sizeof(AT_CMGF), 1000);
  uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 1000);
  HAL_Delay(10);
  memset(receiveUART2Data, '?', sizeof(receiveUART2Data));
  uart2Status = HAL_UART_Transmit(&huart2, AT_CSCS, sizeof(AT_CSCS), 1000);
  uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 1000);
  HAL_Delay(10);
  memset(receiveUART2Data, '?', sizeof(receiveUART2Data));
  uart2Status = HAL_UART_Transmit(&huart2, AT_CMGS_SEND_MSG_BUF, sizeof(AT_CMGS_SEND_MSG_BUF), 1000);
  uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 1000);
  HAL_Delay(10);
  memset(receiveUART2Data, '?', sizeof(receiveUART2Data));
  uart2Status = HAL_UART_Transmit(&huart2, AT_CMGS_SEND_CTRLZ, sizeof(AT_CMGS_SEND_CTRLZ), 1000);
  uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 1000);
  HAL_Delay(10);
  //=========================GSM
  */

  /*
  //=========================GNSS
  uart2Status = HAL_UART_Transmit(&huart2, AT_CGNSPWR_ON, sizeof(AT_CGNSPWR_ON), 1000);
  uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 1000);
  HAL_Delay(1000);
  memset(receiveUART2Data, '?', sizeof(receiveUART2Data));
  uart2Status = HAL_UART_Transmit(&huart2, AT_CGNSSEQ, sizeof(AT_CGNSSEQ), 1000);
  uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 1000);
  HAL_Delay(1000);
  memset(receiveUART2Data, '?', sizeof(receiveUART2Data));
  uart2Status = HAL_UART_Transmit(&huart2, AT_CGNSURC, sizeof(AT_CGNSURC), 1000);
  uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 1000);
  HAL_Delay(1000);
  memset(receiveUART2Data, '?', sizeof(receiveUART2Data));
  uart2Status = HAL_UART_Transmit(&huart2, AT_CGNSINF, sizeof(AT_CGNSINF), 1000);
  uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 1000);
  HAL_Delay(1000);
  memset(receiveUART2Data, '?', sizeof(receiveUART2Data));
  //=========================GNSS
  */

  //=========================MPU9250
  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, PWR_MGMT_1, sizeof(PWR_MGMT_1), 10);
  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, &dataReceiveI2cBuffer, 1, 1000);

  HAL_Delay(10);
  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, PWR_MGMT_2, sizeof(PWR_MGMT_2), 10);
  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, &dataReceiveI2cBuffer, 1, 1000);

  HAL_Delay(10);
  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, &WHO_AM_I, sizeof(WHO_AM_I), 10);
  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, &dataReceiveI2cBuffer, 1, 1000);

  HAL_Delay(10);
  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, LP_ACCEL_ODR, sizeof(LP_ACCEL_ODR), 10);
  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, &dataReceiveI2cBuffer, 1, 1000);

  HAL_Delay(10);
  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, ACCEL_CONFIG, sizeof(ACCEL_CONFIG), 10);
  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, &dataReceiveI2cBuffer, 1, 1000);
  //=========================MPU9250

  /*
  for(uint16_t i = 0; i < 256; i++)	// Used to scan for I2C devices
  {
	  i2cStatus = HAL_I2C_IsDeviceReady(&hi2c1, i, 1, 1);
	  if(i2cStatus == HAL_OK)
	  {
		  IMUDevAddr = i;
		  break;
	  }
	  //HAL_Delay(200);
	  counter2 = counter2 +1;
  }
  */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_TIM_Base_Start(&htim16);
  uartStatus = HAL_UART_Receive_IT(&huart1, receiveUARTData, 14);
  while (1)
  {
	  i2cState = HAL_I2C_GetState(&hi2c1);

	  //=========================MPU9250
	  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, &ACCEL_XOUT_L, sizeof(ACCEL_XOUT_L), 10);
	  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, &dataReceiveI2cBuffer, 1, 100);
	  finalXAccValue = dataReceiveI2cBuffer;
	  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, &ACCEL_XOUT_H, sizeof(ACCEL_XOUT_H), 10);
	  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, &dataReceiveI2cBuffer, 1, 100);
	  finalXAccValue = finalXAccValue + (dataReceiveI2cBuffer << 8);
	  finalXAccValueWithOffset = finalXAccValue + 40000;

	  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, &ACCEL_YOUT_L, sizeof(ACCEL_YOUT_L), 10);
	  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, &dataReceiveI2cBuffer, 1, 100);
	  finalYAccValue = dataReceiveI2cBuffer;
	  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, &ACCEL_YOUT_H, sizeof(ACCEL_YOUT_H), 10);
	  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, &dataReceiveI2cBuffer, 1, 100);
	  finalYAccValue = finalYAccValue + (dataReceiveI2cBuffer << 8);
	  finalYAccValueWithOffset = finalYAccValue + 40000;

	  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, &ACCEL_ZOUT_L, sizeof(ACCEL_ZOUT_L), 10);
	  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, &dataReceiveI2cBuffer, 1, 100);
	  finalZAccValue = dataReceiveI2cBuffer;
	  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, &ACCEL_ZOUT_H, sizeof(ACCEL_ZOUT_H), 10);
	  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, &dataReceiveI2cBuffer, 1, 100);
	  finalZAccValue = finalZAccValue + (dataReceiveI2cBuffer << 8);
	  finalZAccValueWithOffset = finalZAccValue + 88000;
	  //=========================MPU9250

	  //=========================RFID
	  //uartStatus = HAL_UART_Receive(&huart1, receiveUARTData, 14, 100);
	  //=========================RFID

/*
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);	// ONBOARD LED
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	  HAL_Delay(500);

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);	// EXTERNAL LED
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	  HAL_Delay(500);*/

	  /*
	  //=========================GNSS
	  uart2Status = HAL_UART_Transmit(&huart2, AT_CGNSINF, sizeof(AT_CGNSINF), 1000);
	  uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 1000);
	  HAL_Delay(1000);
	  memset(receiveUART2Data, '?', sizeof(receiveUART2Data));
	  //=========================GNSS
	   */
	  if(counter2 != 0){
		  if(__HAL_TIM_GET_COUNTER(&htim16) < timerVal){
			  clockCykles++;
			  timerVal = __HAL_TIM_GET_COUNTER(&htim16);
		  }
		  else{
			  timerVal = __HAL_TIM_GET_COUNTER(&htim16);
		  }
	  }


	  if(lockedDevice != 1){
		  if(clockCykles > 33 && counter2 < 15){
			  clockCykles = 0;
			  counter2 = 0;
		  }
		  else if(counter2 > 15){
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		  }
		  else if(checkMovment()){
			  if(counter2 == 0){
				  timerVal = __HAL_TIM_GET_COUNTER(&htim16);
				  counter2++;
			  }
			  else{
				  counter2++;
			  }
		  }
	  }
	  else{
		  if(finalZAccValueWithOffset < 100000){
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		  }
		  else{
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		  }
	  }

	  counter = counter +1;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x2000090E;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 79;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);	// ONBOARD LED
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
	 /*HAL_Delay(500);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	  HAL_Delay(500);*/
	  uartStatus = HAL_UART_Receive_IT(&huart1, receiveUARTData, 14);
	  if(checkKey(receiveUARTData, UARTDataKey)){
		  if(!lockedDevice){
			  lockedDevice = 1;
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		  }
		  else{
			  lockedDevice = 0;
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		  }

	  }
}

int checkKey(uint8_t arr1[],  uint8_t arr2[])
{
	int i;
	for(i = 0; i < 30; i++)
	{
		if(arr1[i] != arr2[i])
		{
			return 0;
		}
	}
	return 1;
}

int checkMovment()
{
	uint32_t xDiff = abs(refXAccValueWithOffset - finalXAccValueWithOffset);
	uint32_t yDiff = abs(refYAccValueWithOffset - finalYAccValueWithOffset);
	uint32_t zDiff = abs(refZAccValueWithOffset - finalZAccValueWithOffset);

	refXAccValueWithOffset = finalXAccValueWithOffset;
	refYAccValueWithOffset = finalYAccValueWithOffset;
	refZAccValueWithOffset = finalZAccValueWithOffset;

	if(xDiff > 200 || yDiff > 200 || zDiff > 200){
		return 1;
	}
	else{
		return 0;
	}
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
