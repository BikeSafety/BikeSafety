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

uint8_t IMUDevAddr 				= 0;
uint8_t ACCEL_XOUT_L 			= 0;
uint8_t ACCEL_XOUT_H 			= 0;
uint8_t ACCEL_YOUT_L 			= 0;
uint8_t ACCEL_YOUT_H 			= 0;
uint8_t ACCEL_ZOUT_L 			= 0;
uint8_t ACCEL_ZOUT_H 			= 0;

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

//TODO Move all these to the functions
uint8_t dummyBuffer[]			= "1,1,20150327014838.000,31.221783,-00.000100,24.123456,0.28,0.0,1,,1.9,2.2,1.0,,8,4,,,42,,";
//uint8_t dummyBuffer[]			= "1,1,20150327014838.000,31.221783,60.123456,24.123456,0.28,0.0,1,,1.9,2.2,1.0,,8,4,,,42,,";
uint8_t dummyBuffer2[]			= "1,1,20150327014838.000,31.221783,60.123458,24.123458,0.28,0.0,1,,1.9,2.2,1.0,,8,4,,,42,,";
uint8_t longLat[20];
uint8_t degMinSecBuffer[3];
int commaElement;
int counterGNSS;
int latStart;
int latEndLongStart;
int longEnd;
int latDegSize;
int longDegSize;
int latNegDeg;
int longNegDeg;
int gnssFixElement;
int gnssFixStatus;

int latDeg;
int latMin;
int latSecFirst;
int latSecSecond;

int longDeg;
int longMin;
int longSecFirst;
int longSecSecond;

float latInMeters;
float prevlatInMeters;
float dlatInMeters;

float longInMeters;
float prevlongInMeters;
float dlongInMeters;

int enterToFunction = 0;

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
struct LatLongStruct latlongstructinstance; 		// TODO
struct LatLongStruct prevlatlongstructinstance; 	// TODO
struct OffsetFromHome offsetfromhome;				// TODO
char homeLocked = 0;							// TODO
struct GsmStruct gsmstruct;							// TODO

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

  mpuInit();

  gnssInit();
  latlongstructinstance = getLatLongInMeters();
  //offsetfromhome = getOffsetFromHome(latlongstructinstance, prevlatlongstructinstance, homeLocked);
  //homeLocked = 1;
  prevlatlongstructinstance = latlongstructinstance;
  //HAL_Delay(1000);


  gsmInit();											// GSM initializer
  gsmstruct.phoneNumber = "+35844350xxxx";				// Enter number in this format
  gsmstruct.message		= "Sent from Otakaari 5tryreyre";	// Enter message to be send in this format
  sendGsmMessage(gsmstruct);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_TIM_Base_Start(&htim16);
  while (1)
  {

	  //i2cState = HAL_I2C_GetState(&hi2c1);
	  if(lockedDevice == 1 || counter2 != 0){
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
	  }
	  if(lockedDevice == 1){
		  //counter = counter +1;
		  if(finalZAccValueWithOffset < 100000){
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		  }
		  else{
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		  }
	  }

	  /*else if(counter2 != 0){
		  homeLocked = 1;
		  latlongstructinstance = getLatLongInMeters();
		  if(__HAL_TIM_GET_COUNTER(&htim16) < timerVal){
			  clockCykles++;
			  timerVal = __HAL_TIM_GET_COUNTER(&htim16);
		  }
		  else{
			  timerVal = __HAL_TIM_GET_COUNTER(&htim16);
		  }

		  if(clockCykles > 33 && counter2 < 15){
			  clockCykles = 0;
			  counter2 = 0;
			  //INT_ENABLE[1] = 0x40;
		  }
		  else if(counter2 > 15){
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);	// EXTERNAL LED
			  HAL_Delay(500);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			  HAL_Delay(500);
		  }
		  else if(getOffsetFromHome(latlongstructinstance, prevlatlongstructinstance, homeLocked) > 200){
			  counter2++;
		  }

	  }*/

	  else{
		HAL_SuspendTick();

		  HAL_PWR_EnableSleepOnExit ();
		  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

		  HAL_ResumeTick();
	  }


	  //=========================GNSS
	  //latlongstructinstance = getLatLongInMeters();
	  //offsetfromhome = getOffsetFromHome(latlongstructinstance, prevlatlongstructinstance, homeLocked);
	  //prevlatlongstructinstance = latlongstructinstance;
	  //=========================GNSS

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

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_4 && lockedDevice == 0) // If The INT Source Is EXTI Line4 (A4 Pin)
    {
    	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3); // Toggle The ONBOARD LED
    	//timerVal = __HAL_TIM_GET_COUNTER(&htim16);
    	//counter2++;
    	//HAL_PWR_DisableSleepOnExit ();
    	//uint8_t INT_ENABLE[2] 			= {0x38, 0x00};
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    	/*latlongstructinstance = getLatLongInMeters();
    	homeLocked = 1;
    	offsetfromhome = getOffsetFromHome(latlongstructinstance, prevlatlongstructinstance, homeLocked);
    	if(offsetfromhome.offsetLatInMeters > 150 || offsetfromhome.offsetLongInMeters > 150){
    		//sendGsmMessage(gsmstruct);
    		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
    	}*/
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);	// ONBOARD LED
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
	 /*HAL_Delay(500);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	  HAL_Delay(500);*/
	  uartStatus = HAL_UART_Receive_IT(&huart1, receiveUARTData, 14);
	  if(checkKey(receiveUARTData, UARTDataKey) == 1){
		  if(!lockedDevice){
			  lockedDevice = 1;
			  counter2 = 0;
			  HAL_PWR_DisableSleepOnExit ();
			  homeLocked = 0;
			  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		  }
		  else{
			  lockedDevice = 0;
			  homeLocked = 1;
			  getOffsetFromHome(latlongstructinstance, prevlatlongstructinstance, homeLocked);
			  prevlatlongstructinstance = latlongstructinstance;
			  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
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

	if(xDiff > 400 || yDiff > 400 || zDiff > 400){
		return 1;
	}
	else{
		return 0;
	}
}
struct LatLongStruct getLatLongInMeters(void){
	enterToFunction = 1; // TODO delete
	struct LatLongStruct latlongstruct = {0};	// Stores LatLong metric values
	HAL_StatusTypeDef uart2Status;				// Status of uart2
	uint8_t AT_CGNSINF[] = "AT+CGNSINF\r";		// Gets data from GNSS

    uart2Status = HAL_UART_Transmit(&huart2, AT_CGNSINF, sizeof(AT_CGNSINF), 1000);
    uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 1000);
    HAL_Delay(100);

	//dummyBuffer[0]	= "1,1,20150327014838.000,31.221783,60.123456,24.123456,0.28,0.0,1,,1.9,2.2,1.0,,8,4,,,42,,";

	for(int i = 0; i < sizeof(receiveUART2Data); i++){
		if(receiveUART2Data[i] == ','){
			commaElement = i;
			counterGNSS++;
		}
		if(counterGNSS == 1){
			gnssFixElement = i;
			if(receiveUART2Data[i] == '1'){
				gnssFixStatus = 1;
			}
			if(receiveUART2Data[i] == '0'){
				gnssFixStatus = 0;
			}
		}
		if(counterGNSS == 2){
			latStart = i+1;
			if(receiveUART2Data[latStart+1] == '-'){latNegDeg = 1;}
			else{latNegDeg = 0;}
			for(int j = latStart; j < latStart+6; j++){
				if(receiveUART2Data[j] == '.'){latDegSize = j-latStart-1-latNegDeg;}
			}
		}
		if(counterGNSS == 3){
			latEndLongStart = i+1;
			if(receiveUART2Data[latEndLongStart+1] == '-'){longNegDeg = 1;}
			else{longNegDeg = 0;}
			for(int j = latEndLongStart; j < latEndLongStart+6; j++){
				if(receiveUART2Data[j] == '.'){longDegSize = j-latEndLongStart-1-longNegDeg;}
			}
		}
		if(counterGNSS == 6){
			longEnd = i;
			break;
		}
	}
	counterGNSS = 0;

	memset(degMinSecBuffer, '0', sizeof(degMinSecBuffer));
	for(int i = 0; i < latDegSize; i++){degMinSecBuffer[i+3-latDegSize] = receiveUART2Data[latStart+i+1+latNegDeg];}
	latDeg = atoi(degMinSecBuffer);
	memset(degMinSecBuffer, '0', sizeof(degMinSecBuffer));
	for(int i = 1; i < 3; i++){degMinSecBuffer[i] = receiveUART2Data[latStart+i+latDegSize+1+latNegDeg];}
	latMin = atoi(degMinSecBuffer);
	for(int i = 1; i < 3; i++){degMinSecBuffer[i] = receiveUART2Data[latStart+i+latDegSize+3+latNegDeg];}
	latSecFirst = atoi(degMinSecBuffer);
	for(int i = 1; i < 3; i++){degMinSecBuffer[i] = receiveUART2Data[latStart+i+latDegSize+5+latNegDeg];}
	latSecSecond = atoi(degMinSecBuffer);
	memset(degMinSecBuffer, '0', sizeof(degMinSecBuffer));

	for(int i = 0; i < longDegSize; i++){degMinSecBuffer[i+3-longDegSize] = receiveUART2Data[latEndLongStart+i+1+longNegDeg];}
	longDeg = atoi(degMinSecBuffer);
	memset(degMinSecBuffer, '0', sizeof(degMinSecBuffer));
	for(int i = 1; i < 3; i++){degMinSecBuffer[i] = receiveUART2Data[latEndLongStart+i+longDegSize+1+longNegDeg];}
	longMin = atoi(degMinSecBuffer);
	for(int i = 1; i < 3; i++){degMinSecBuffer[i] = receiveUART2Data[latEndLongStart+i+longDegSize+3+longNegDeg];}
	longSecFirst = atoi(degMinSecBuffer);
	for(int i = 1; i < 3; i++){degMinSecBuffer[i] = receiveUART2Data[latEndLongStart+i+longDegSize+5+longNegDeg];}
	longSecSecond = atoi(degMinSecBuffer);
	memset(degMinSecBuffer, '0', sizeof(degMinSecBuffer));

	latInMeters = (latDeg*111000.0)+(latMin*1850.0)+(latSecFirst*30.0)+(latSecSecond*0.3);
	//dlatInMeters = abs(latInMeters - prevlatInMeters);
	//prevlatInMeters = latInMeters;

	longInMeters = (longDeg*111000.0)+(longMin*1850.0)+(longSecFirst*30.0)+(longSecSecond*0.3);
	//dlongInMeters = abs(longInMeters - prevlongInMeters);
	//prevlongInMeters = longInMeters;
	memset(receiveUART2Data, '?', sizeof(receiveUART2Data));

	latlongstruct.gnssFixOk 	= gnssFixStatus;
	latlongstruct.uartStatusOk	= uart2Status;
	latlongstruct.latNeg		= latNegDeg;
	latlongstruct.longNeg		= longNegDeg;
	latlongstruct.latInMeters	= latInMeters;
	latlongstruct.longInMeters	= longInMeters;

	return latlongstruct;
}

struct OffsetFromHome getOffsetFromHome(struct LatLongStruct latlongstruct, struct LatLongStruct prevlatlongstruct, char homeLocked){
	enterToFunction = 2; // TODO delete
	struct OffsetFromHome offsetfromhome;
	if(homeLocked == 0){
		offsetfromhome.offsetLatInMeters = 0.0;
		offsetfromhome.offsetLongInMeters = 0.0;
	}
	if(homeLocked == 1){
		offsetfromhome.offsetLatInMeters = abs(latlongstruct.latInMeters - prevlatlongstruct.latInMeters);
		offsetfromhome.offsetLongInMeters = abs(latlongstruct.longInMeters - prevlatlongstruct.longInMeters);
	}
	return offsetfromhome;
}

char mpuInit(void){
	char initStatus = 0;
	//=========================MPU9250
	// Registers
	IMUDevAddr 						= 0xd0;
	/*uint8_t PWR_MGMT_1[2] 			= {0x6b, 0b00100000};	// or 4
	//uint8_t PWR_MGMT_2[2] 			= {0x6c, 0b00000000};	// 0 to enable all or 255 to disable all
	uint8_t PWR_MGMT_2[2] 			= {0x6c, 0b00000111};	//
	uint8_t WHO_AM_I 				= 0x75;
	uint8_t LP_ACCEL_ODR[2] 		= {0x1e, 0b00001000}; 	// 8 = output frequency 62.50Hz
	uint8_t ACCEL_CONFIG[2] 		= {0x1c, 0x0}; 			// 0x0 for 2g, 0x8 for 4g, 0x10 for 8g,0x18 for 16g
	uint8_t ACCEL_CONFIG_2[2] 		= {0x1d, 0b00000101};	//1d
	uint8_t INT_ENABLE[2] 			= {0x38, 0x40};			// enable motion interrupt
	uint8_t MOT_DETECT_CTRL[2] 		= {0x69, 0b11000000};	// enable hardware intelligence
	uint8_t WOM_THR[2]				= {0x1f, 0x7f};			// threshold
	uint8_t maskLP_ACCEL_ODR[2] 	= {0x1e, 0b00000100}; 	// frequency of wake-up
	uint8_t PWR_MGMT_1_new[2] 		= {0x6b, 0b00100000};	// cycle mode
	uint8_t INT_PIN_CFG[2] 			= {0x37, 0b00110000};	//3a*/

	uint8_t ACCEL_CONFIG[2] 		= {0x1c, 0x0}; 			// 0x0 for 2g, 0x8 for 4g, 0x10 for 8g,0x18 for 16g
	uint8_t WHO_AM_I 				= 0x75;
    uint8_t PWR_MGMT_1[2] = {0x6b, 0b00000001};//6b
    uint8_t PWR_MGMT_2[2] = {0x6c, 0b00000111};//6c
    uint8_t ACCEL_CONFIG_2[2] ={0x1d, 0b00000101};//1d
    uint8_t INT_ENABLE[2] = {0x38, 0x40};//38
    uint8_t MOT_DETECT_CTRL[2] = {0x69, 0b11000000};//69
    uint8_t WOM_THR[2]={0x1f, 0x7f};//1f
    uint8_t LP_ACCEL_ODR[2] = {0x1e, 0b00000100};//1e
    uint8_t PWR_MGMT_1_new[2] = {0x6b, 0b00100000};//6b
    uint8_t INT_PIN_CFG[2] = {0x37, 0b00110000};//3a

	ACCEL_XOUT_L 			= 0x3c;
	ACCEL_XOUT_H 			= 0x3b;
	ACCEL_YOUT_L 			= 0x3e;
	ACCEL_YOUT_H 			= 0x3d;
	ACCEL_ZOUT_L 			= 0x40;
	ACCEL_ZOUT_H 			= 0x3f;

	i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, PWR_MGMT_1, sizeof(PWR_MGMT_1), 10);
	i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, &dataReceiveI2cBuffer, 1, 10);

	HAL_Delay(10);
	i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, PWR_MGMT_2, sizeof(PWR_MGMT_2), 10);
	i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, &dataReceiveI2cBuffer, 1, 10);

	HAL_Delay(10);
	i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, &WHO_AM_I, sizeof(WHO_AM_I), 10);
	i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, &dataReceiveI2cBuffer, 1, 10);

	HAL_Delay(10);
	i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, LP_ACCEL_ODR, sizeof(LP_ACCEL_ODR), 10);
	i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, &dataReceiveI2cBuffer, 1, 10);

	HAL_Delay(10);
	i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, ACCEL_CONFIG, sizeof(ACCEL_CONFIG), 10);
	i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, &dataReceiveI2cBuffer, 1, 10);
//=========================MPU9250
	uartStatus = HAL_UART_Receive_IT(&huart1, receiveUARTData, 14);
	initStatus = 1;
	return initStatus;
}



char gnssInit(void){
	// TODO Add proper status return or error handling
	char initStatus = 0;
	//=========================GNSS
	uint8_t AT_CGNSPWR_ON[] 	= "AT+CGNSPWR=1\r";				// GNSS turns Power ON
	uint8_t AT_CGNSPWR_OFF[] 	= "AT+CGNSPWR=0\r";				// GNSS turns Power OFF
	uint8_t AT_CGNSSEQ[] 		= "AT+CGNSSEQ=\"RMC\"\r";		// RMC for GGA
	uint8_t AT_CGNSINF[] 		= "AT+CGNSINF\r";				// Gets data from GNSS
	uint8_t AT_CGNSURC_SET[] 	= "AT+CGNSURC=0\r";
	uint8_t AT_CGNSURC_ASK[] 	= "AT+CGNSURC?\r";
	//=========================GNSS

	//=========================GNSS
	uart2Status = HAL_UART_Transmit(&huart2, AT_CGNSPWR_ON, sizeof(AT_CGNSPWR_ON), 1000);
	uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 1000);
	HAL_Delay(20);
	memset(receiveUART2Data, '?', sizeof(receiveUART2Data));

	uart2Status = HAL_UART_Transmit(&huart2, AT_CGNSURC_SET, sizeof(AT_CGNSURC_SET), 1000);
	uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 1000);
	HAL_Delay(10);
	memset(receiveUART2Data, '?', sizeof(receiveUART2Data));

	uart2Status = HAL_UART_Transmit(&huart2, AT_CGNSSEQ, sizeof(AT_CGNSSEQ), 1000);
	uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 1000);
	HAL_Delay(10);
	memset(receiveUART2Data, '?', sizeof(receiveUART2Data));
	//=========================GNSS
	initStatus = 1;
	return initStatus;
}


char gsmInit(void){
	// TODO Add proper status return or error handling
	char initStatus = 0;
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
	//=========================GSM

	//=========================GSM
	uart2Status = HAL_UART_Transmit(&huart2, AT_COPS_RGSTR, sizeof(AT_COPS_RGSTR), 10);
	uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 10);
	HAL_Delay(10);

	uart2Status = HAL_UART_Transmit(&huart2, AT_COPS_CRNT, sizeof(AT_COPS_CRNT), 10);
	uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 10);
	HAL_Delay(50);

	uart2Status = HAL_UART_Transmit(&huart2, AT_CMGF, sizeof(AT_CMGF), 10);
	uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 10);
	HAL_Delay(10);
	memset(receiveUART2Data, '?', sizeof(receiveUART2Data));
	initStatus = 1;
	return initStatus;
}

char sendGsmMessage(struct GsmStruct gsmstruct){
	// TODO Add proper status return or error handling
	gsmstruct.numberOk = 0;
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
	//uint8_t AT_CMGS_SEND_MSG_BUF[] 	= "AT+CMGS=\"+35844350xxxx\"\rMESSAGE";
	//=========================GSM
	char AT_CMGS_SEND_MSG_BUF_STRT[] = "AT+CMGS=\"";
	char AT_CMGS_SEND_MSG_BUF_MIDL[] = "\"\r";

	char AT_CMGS_SEND_MSG_BUF_TOT[sizeof(AT_CMGS_SEND_MSG_BUF_STRT)+sizeof(AT_CMGS_SEND_MSG_BUF_MIDL)+strlen(gsmstruct.phoneNumber)+strlen(gsmstruct.message)-1];
	strcpy(AT_CMGS_SEND_MSG_BUF_TOT, AT_CMGS_SEND_MSG_BUF_STRT);
	strcat(AT_CMGS_SEND_MSG_BUF_TOT, gsmstruct.phoneNumber);
	strcat(AT_CMGS_SEND_MSG_BUF_TOT, AT_CMGS_SEND_MSG_BUF_MIDL);
	strcat(AT_CMGS_SEND_MSG_BUF_TOT, gsmstruct.message);
	//=========================GSM
	memset(receiveUART2Data, '?', sizeof(receiveUART2Data));

	uart2Status = HAL_UART_Transmit(&huart2, AT_CSCS, sizeof(AT_CSCS), 10);
	uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 10);
	HAL_Delay(10);
	memset(receiveUART2Data, '?', sizeof(receiveUART2Data));

	uart2Status = HAL_UART_Transmit(&huart2, AT_CMGS_SEND_MSG_BUF_TOT, sizeof(AT_CMGS_SEND_MSG_BUF_TOT), 10);
	uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 10);
	HAL_Delay(10);
	memset(receiveUART2Data, '?', sizeof(receiveUART2Data));

	uart2Status = HAL_UART_Transmit(&huart2, AT_CMGS_SEND_CTRLZ, sizeof(AT_CMGS_SEND_CTRLZ), 1000);
	uart2Status = HAL_UART_Receive(&huart2, receiveUART2Data, sizeof(receiveUART2Data), 10);
	HAL_Delay(10);
	memset(receiveUART2Data, '?', sizeof(receiveUART2Data));
	//=========================GSM
	gsmstruct.numberOk = 1;
	return gsmstruct.numberOk;
}

/* TODO Implement proper I2C if needed
void i2cScanDevices(void){
	for(uint16_t i = 0; i < 256; i++)	// Used to scan for I2C devices
	{
	  i2cStatus = HAL_I2C_IsDeviceReady(&hi2c1, i, 1, 1);
	  if(i2cStatus == HAL_OK)
	  {
		  IMUDevAddr = i;
		  break;
	  }
	  //HAL_Delay(200);
	  //counter2 = counter2 +1;
	}

}
*/

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
