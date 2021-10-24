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
HAL_StatusTypeDef i2cStatus;		  // Status of i2c
HAL_StatusTypeDef spiStatus;		  // Status of spi
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint32_t counter = 0;
uint32_t counter2 = 0;

//***************************MPU9250
// Registers
uint8_t IMUDevAddr 					        = 208;
uint8_t PWR_MGMT_1[2] 				      = {107, 32/*or 4*/};
uint8_t PWR_MGMT_2[2] 				      = {108, 0/*0 to enable all or 255 to disable all*/};
uint8_t WHO_AM_I[1] 				        = {117};
uint8_t LP_ACCEL_ODR[2] 			      = {30, 8/* 8 = output frequency 62.50Hz*/};
uint8_t ACCEL_CONFIG[2] 			      = {28, 24/*8 for 2g, 24 for 16g*/};
uint8_t dataReceiveI2cBufferLow[1] 	= {0};
uint8_t dataReceiveI2cBufferHigh[1] = {0};
uint8_t ACCEL_XOUT_L[1] 			= {60};
uint8_t ACCEL_XOUT_H[1] 			= {59};
uint8_t ACCEL_YOUT_L[1] 			= {62};
uint8_t ACCEL_YOUT_H[1] 			= {61};
uint8_t ACCEL_ZOUT_L[1] 			= {64};
uint8_t ACCEL_ZOUT_H[1] 			= {63};

// Variables
uint64_t finalXAccValue 			= 0;
uint64_t finalXAccValueWithOffset 	= 0;
uint64_t finalYAccValue 			= 0;
uint64_t finalYAccValueWithOffset 	= 0;
uint64_t finalZAccValue 			= 0;
uint64_t finalZAccValueWithOffset 	= 0;

uint8_t readXAccReg = 60;
//***************************MPU9250

//***************************RFID
/*
uint8_t VersionReg[1] = {238};									// Should give d146
uint8_t Status1Reg[1] = {142};									//
uint8_t FIFODataReg[1] = {146};									// Read
uint8_t FIFOLevelReg[1] = {148};								// Read
uint8_t FIFOLevelRegFlush[2] = {20, 128};						// FiFo flush
uint8_t CommandReg[1] = {130}; 									// Read CommandReg
uint8_t CommandRegWriteActivateReceive[2] = {2, 8}; 			// Write activate Receive
uint8_t CommandRegWriteStopReceiver[2] = {2, 0}; 				// Write activate Receive
uint8_t CommandRegWriteGenerateRndmID[2] = {2, 2}; 				// Write Generate rndm ID

uint8_t receiveSPIData[64] = {0};
*/
//***************************RFID

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
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
  /* USER CODE BEGIN 2 */

  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, PWR_MGMT_1, (uint8_t)2, 10);
  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, dataReceiveI2cBufferLow, (uint8_t)1, 1000);

  HAL_Delay(10);
  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, PWR_MGMT_2, (uint8_t)2, 10);
  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, dataReceiveI2cBufferLow, (uint8_t)1, 1000);

  HAL_Delay(10);
  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, WHO_AM_I, (uint8_t)1, 10);
  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, dataReceiveI2cBufferLow, (uint8_t)1, 1000);

  HAL_Delay(10);
  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, LP_ACCEL_ODR, (uint8_t)2, 10);
  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, dataReceiveI2cBufferLow, (uint8_t)1, 1000);

  HAL_Delay(10);
  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, ACCEL_CONFIG, (uint8_t)2, 10);
  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, dataReceiveI2cBufferLow, (uint8_t)1, 1000);


  //****************************RFID
  /*
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); 	// RFID reset
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  //spiStatus = HAL_SPI_TransmitReceive(&hspi1, VersionReg, receiveSPIData, 1, 1000);
  //spiStatus = HAL_SPI_Transmit(&hspi1, VersionReg, (uint8_t)1, 100);
  //spiStatus = HAL_SPI_Receive(&hspi1, &receiveSPIData, (uint8_t)1, 1000);
  //spiStatus = HAL_SPI_Receive(&hspi1, receiveSPIData[1], (uint8_t)1, 1000);
  */
  //****************************RFID

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
	  i2cState = HAL_I2C_GetState(&hi2c1);
	  /*
	  for(uint16_t i = 0; i < 256; i++)
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
	  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, ACCEL_XOUT_L, (uint8_t)1, 10);
	  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, dataReceiveI2cBufferLow, (uint8_t)1, 1000);
	  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, ACCEL_XOUT_H, (uint8_t)1, 10);
	  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, dataReceiveI2cBufferHigh, (uint8_t)1, 1000);
	  finalXAccValue = dataReceiveI2cBufferHigh[0] << 8;
	  finalXAccValue = finalXAccValue + dataReceiveI2cBufferLow[0];
	  finalXAccValueWithOffset = finalXAccValue + 40000;

	  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, ACCEL_YOUT_L, (uint8_t)1, 10);
	  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, dataReceiveI2cBufferLow, (uint8_t)1, 1000);
	  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, ACCEL_YOUT_H, (uint8_t)1, 10);
	  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, dataReceiveI2cBufferHigh, (uint8_t)1, 1000);
	  finalYAccValue = dataReceiveI2cBufferHigh[0] << 8;
	  finalYAccValue = finalYAccValue + dataReceiveI2cBufferLow[0];
	  finalYAccValueWithOffset = finalYAccValue + 40000;

	  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, ACCEL_ZOUT_L, (uint8_t)1, 10);
	  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, dataReceiveI2cBufferLow, (uint8_t)1, 1000);
	  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, IMUDevAddr, ACCEL_ZOUT_H, (uint8_t)1, 10);
	  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, IMUDevAddr, dataReceiveI2cBufferHigh, (uint8_t)1, 1000);
	  finalZAccValue = dataReceiveI2cBufferHigh[0] << 8;
	  finalZAccValue = finalZAccValue + dataReceiveI2cBufferLow[0];
	  finalZAccValueWithOffset = finalZAccValue + 88000;

    /*
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	  HAL_Delay(500);
    */

	  //****************************RFID BEGIN

	  /*
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  HAL_Delay(100);
	  spiStatus = HAL_SPI_Transmit(&hspi1, VersionReg, 1, 100);
	  spiStatus = HAL_SPI_Receive(&hspi1, receiveSPIData, 1, 1000);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	  HAL_Delay(1000);

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  HAL_Delay(100);
	  spiStatus = HAL_SPI_Transmit(&hspi1, Status1Reg, 1, 100);
	  spiStatus = HAL_SPI_Receive(&hspi1, receiveSPIData, 1, 1000);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	  HAL_Delay(1000);

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  HAL_Delay(100);
	  spiStatus = HAL_SPI_Transmit(&hspi1, FIFOLevelRegFlush, 2, 100);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	  HAL_Delay(10);

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  HAL_Delay(10);
	  spiStatus = HAL_SPI_Transmit(&hspi1, CommandRegWriteActivateReceive, 2, 100);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	  HAL_Delay(10);

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  spiStatus = HAL_SPI_Transmit(&hspi1, CommandRegWriteStopReceiver, 2, 100);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	  HAL_Delay(100);

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  spiStatus = HAL_SPI_Transmit(&hspi1, FIFODataReg, 1, 100);
	  spiStatus = HAL_SPI_Receive(&hspi1, receiveSPIData, 64, 1000);
	  HAL_Delay(10);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	  HAL_Delay(10);

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  spiStatus = HAL_SPI_Transmit(&hspi1, FIFODataReg, 1, 100);
	  spiStatus = HAL_SPI_Receive(&hspi1, receiveSPIData, 64, 1000);
	  HAL_Delay(10);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	  HAL_Delay(10);
	  */

	  //HAL_SPI_Transmit(hspi, pData, Size, Timeout)

	  //****************************RFID END
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
