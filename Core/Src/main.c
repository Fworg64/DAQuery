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
#include "GIRAFFE_fdc2114.h"
#include "checksum.h"
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
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t DO_SCAN = 0;
uint8_t UART_TX_DONE = 0;

GIRAFFE_FDC2114_IT_state_t i2c_state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void GIRAFFE_UART_Print(UART_HandleTypeDef *huart, char _out[]);
GIRAFFE_i2c_status_t i2c_read_wrapper(uint16_t dev_addr, uint8_t *data, uint16_t len);
GIRAFFE_i2c_status_t i2c_write_wrapper(uint16_t dev_addr, uint8_t *data, uint16_t len);
GIRAFFE_i2c_status_t i2c_read_wrapper_IT(uint16_t dev_addr, uint8_t *data, uint16_t len);
GIRAFFE_i2c_status_t i2c_write_wrapper_IT(uint16_t dev_addr, uint8_t *data, uint16_t len);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
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
  __USART2_CLK_ENABLE();
  __I2C3_CLK_ENABLE();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  // initialize fdc library
  GIRAFFE_FDC2114_initialize_lib(&i2c_read_wrapper, &i2c_write_wrapper);
  GIRAFFE_FDC2114_initialize_IT(&i2c_read_wrapper_IT, &i2c_write_wrapper_IT);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // shutdown
  HAL_Delay(10); //flashy
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); //flashy
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); // un-shutdown
  HAL_Delay(10);
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  //Check for differential sensor jumper
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) // set with jumper to A1
  {
	  GIRAFFE_FDC2114_set_single_ended();
	  for (uint8_t index=0; index<20; index++)
	  {
		  HAL_Delay(100); //flashy
	      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); //flashy
	  }
  }
  GIRAFFE_FDC2114_init_dev(); //not checking if OK, but we could
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //uint8_t outstr[] = "0000AABBCCDD"; //four seq/status bytes, 2 bytes per channel
  HAL_StatusTypeDef transmit_status;

  struct packet_data_s
  {
	  uint16_t status;
	  uint16_t seq;
	  GIRAFFE_FDC2114_channel_data_t chan_data;
	  uint16_t crc;
  } next_packet_data;

  union packet_u
  {
	  uint8_t outstr[14];
	  struct packet_data_s packet_data;
  } packet;

  uint8_t startup_message[] = "DOG-TOOTHv03"; // note null termination
  uint16_t packet_payload_crc = crc_16(startup_message, sizeof(startup_message)-1);
  memcpy(packet.outstr, startup_message, sizeof(startup_message)-1);
  packet.packet_data.crc = packet_payload_crc;
  transmit_status = HAL_UART_Transmit(&huart1, packet.outstr, sizeof(packet.outstr), 1000);

  memset(packet.outstr, 0, sizeof(packet.outstr));
  //packet.outstr = "DOG-TOOTHv02";

  HAL_TIM_Base_Start_IT(&htim1);


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (DO_SCAN)// && HAL_GPIO_ReadPin(GPIOC, FDC_INTB_Pin))
	{
	  transmit_status = HAL_UART_Transmit_IT(&huart1,
	  	               packet.outstr, sizeof(packet.outstr));
	  GIRAFFE_FDC2114_read_channels_IT(&i2c_state);

	  //transmit_status = HAL_UART_Transmit(&huart1, packet.outstr,
	//		  sizeof(packet.outstr), HAL_MAX_DELAY);
	  //GIRAFFE_FDC2114_read_channels(&next_packet_data.chan_data);
	  next_packet_data.status = 0;
	  next_packet_data.seq++;

	  while ((UART_TX_DONE == 0) // while the uart is not done or the i2c is not done (and i2c is in a normal state)
			 || ((i2c_state.num_reads_remaining != 0)
			      && (!GIRAFFE_is_error_severe(i2c_state.error))))
	  {}
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	  GIRAFFE_FDC2114_get_chan_data_from_state(&i2c_state, &next_packet_data.chan_data);

	  packet.packet_data = next_packet_data; // *copy* data to packet
	  // The union takes care of putting the data in a c string
	  packet_payload_crc = crc_16(packet.outstr, 12); // Calculate checksum on header, seq, and data (12 bytes)
	  packet.packet_data.crc = packet_payload_crc; // sign packet with checksum

	  UART_TX_DONE = 0;
	  DO_SCAN = 0;
	}
	//HAL_Delay(1);

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
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C3;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00702991;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 32;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3040;
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

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SNGLHIGH_GPIO_Port, SNGLHIGH_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : FDC_INTB_Pin */
  GPIO_InitStruct.Pin = FDC_INTB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FDC_INTB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SNGLSNS_Pin */
  GPIO_InitStruct.Pin = SNGLSNS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SNGLSNS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SNGLHIGH_Pin */
  GPIO_InitStruct.Pin = SNGLHIGH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SNGLHIGH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void GIRAFFE_UART_Print(UART_HandleTypeDef *huart, char _out[])
{
	HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
}

GIRAFFE_i2c_status_t i2c_read_wrapper(uint16_t dev_addr, uint8_t *data, uint16_t len)
{
	if (HAL_I2C_Master_Receive(&hi2c3, dev_addr, data, len, HAL_MAX_DELAY) == HAL_OK)
	return I2C_OK;
	else return I2C_ERROR;
}

GIRAFFE_i2c_status_t i2c_write_wrapper(uint16_t dev_addr, uint8_t *data, uint16_t len)
{
	if (HAL_I2C_Master_Transmit(&hi2c3, dev_addr, data, len, HAL_MAX_DELAY) == HAL_OK)
	return I2C_OK;
	else return I2C_ERROR;
}

GIRAFFE_i2c_status_t i2c_read_wrapper_IT(uint16_t dev_addr, uint8_t *data, uint16_t len)
{
	if (HAL_I2C_Master_Receive_IT(&hi2c3, dev_addr, data, len) == HAL_OK)
	return I2C_OK;
	else return I2C_ERROR;
}

GIRAFFE_i2c_status_t i2c_write_wrapper_IT(uint16_t dev_addr, uint8_t *data, uint16_t len)
{
	if (HAL_I2C_Master_Transmit_IT(&hi2c3, dev_addr, data, len) == HAL_OK)
	return I2C_OK;
	else return I2C_ERROR;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	  if (GIRAFFE_is_error_severe(i2c_state.error))
      {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	  }
	  else
	  {
		  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); //flashy
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	  }
      DO_SCAN = 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	UART_TX_DONE = 1;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	GIRAFFE_FDC2114_write_clbk(&i2c_state);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	GIRAFFE_FDC2114_read_clbk(&i2c_state);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	switch (hi2c->ErrorCode)
	{
	case HAL_I2C_ERROR_AF:
    case HAL_I2C_ERROR_ARLO:
    case HAL_I2C_ERROR_BERR:
    	i2c_state.error = BUS_ERROR;
    	break;
    default:
    	i2c_state.error = OTHER_ERROR;
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
