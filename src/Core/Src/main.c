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
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define kbcols 14
#define kbrows 5

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t kb_col[kbcols]={col0_Pin,col1_Pin,col2_Pin,col3_Pin,col4_Pin,col5_Pin,col6_Pin,col7_Pin,col8_Pin,col9_Pin,col10_Pin,col11_Pin,col12_Pin,col13_Pin};
GPIO_TypeDef* kb_col_port[kbcols]={col0_GPIO_Port,col1_GPIO_Port,col2_GPIO_Port,col3_GPIO_Port,col4_GPIO_Port,col5_GPIO_Port,col6_GPIO_Port,col7_GPIO_Port,col8_GPIO_Port,col9_GPIO_Port,col10_GPIO_Port,col11_GPIO_Port,col12_GPIO_Port,col13_GPIO_Port};
uint16_t kb_row[kbrows]={row0_Pin,row1_Pin,row2_Pin,row3_Pin,row4_Pin};
GPIO_TypeDef* kb_row_port[kbrows]={row0_GPIO_Port,row1_GPIO_Port,row2_GPIO_Port,row3_GPIO_Port,row4_GPIO_Port};

uint8_t kb_state[kbcols][kbrows] = {0};
uint8_t kb_laststate[kbcols][kbrows] = {0};
uint8_t kb_pushed[kbcols][kbrows] = {0};
uint8_t kb_released[kbcols][kbrows] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void print_adc();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

void read_keys(int report){
	int i = 0;
	uint8_t MSG[16];
	while(i<kbcols){
		int j = 0;
		HAL_GPIO_WritePin(kb_col_port[i], kb_col[i], 1);
		while(j<kbrows){
			kb_laststate[i][j] = kb_state[i][j];
			kb_state[i][j] = HAL_GPIO_ReadPin(kb_row_port[j], kb_row[j]);
			if(kb_laststate[i][j] != kb_state[i][j] && kb_state[i][j]==1) {
				kb_pushed[i][j]=1;
				memset(MSG, 0, sizeof(MSG));
				int msglength;
				sprintf(MSG, "1 %d 1\r\n%n", i+kbcols*j, &msglength);
				if (report) HAL_UART_Transmit(&huart2, MSG, msglength*sizeof(char), 100);
			}
			else kb_pushed[i][j]=0;
			if(kb_laststate[i][j] != kb_state[i][j] && kb_state[i][j]==0) {
							kb_released[i][j]=1;
							memset(MSG, 0, sizeof(MSG));
							int msglength;
							sprintf(MSG, "1 %d 0\r\n%n", i+kbcols*j, &msglength);
							if (report) HAL_UART_Transmit(&huart2, MSG, msglength*sizeof(char), 100);
						}
			else kb_released[i][j]=0;
			j++;
		}
		HAL_GPIO_WritePin(kb_col_port[i], kb_col[i], 0);
		i++;
	};


}

void print_adc(){
	uint8_t writer= 0b10010000;
	HAL_I2C_Mem_Write(&hi2c1,75<<1,0x3,I2C_MEMADD_SIZE_8BIT,&writer,1,0xFFF);
	uint8_t MSG[128] = {"\e[3J\033c\e[1;34mADC READS:\e[0m\r\n\n\n\n\n"};
	HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
	int adresses_ADC[6]={0xe, 0xF, 0x10, 0x11, 0x12, 0x13};
	float resolution_ADC[6]={255, 255, 255, 128, 255, 255};
	float maxvalue_ADC[6]={5.1, 5.1, 100, 7.62, 5.66, 3.39};
	char *descriptions_ADC[6]={"BatV","SysV","NTCV","InpV","ChgA","InpA"};
	int i = 0;
	uint8_t reader;
	while(i<6){
		memset(MSG, 0, sizeof(MSG));
		HAL_I2C_Mem_Read(&hi2c1, 75<<1, adresses_ADC[i], I2C_MEMADD_SIZE_8BIT, &reader, 1, 0xFFF);
		float input = reader;
		float temp1 = input/resolution_ADC[i];
		float output = temp1*maxvalue_ADC[i];
		sprintf(MSG, "\r\n%s: %5.3f\r\n", descriptions_ADC[i], output);
		HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
		i++;
	}
	HAL_Delay(1000);
}

void edit_mpreg(){
	uint8_t MSG[128];
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
	HAL_Delay(150);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
	int i = 0;
	uint8_t reader;
	//breakpoint here, edit i to select register to edit/view
	memset(MSG, 0, sizeof(MSG));
	HAL_I2C_Mem_Read(&hi2c1, 75<<1, i, I2C_MEMADD_SIZE_8BIT, &reader, 1, 0xFFF);
	sprintf(MSG, "%d read as: "BYTE_TO_BINARY_PATTERN"\r\n",i, BYTE_TO_BINARY(reader));
	HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
	//breakpoint here, edit reader to desired value
	sprintf(MSG, "%d changed to: "BYTE_TO_BINARY_PATTERN"\r\n",i, BYTE_TO_BINARY(reader));
	HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
	HAL_I2C_Mem_Write(&hi2c1,75<<1,i,I2C_MEMADD_SIZE_8BIT,&reader,1,0xFFF);
	HAL_I2C_Mem_Read(&hi2c1, 75<<1, i, I2C_MEMADD_SIZE_8BIT, &reader, 1, 0xFFF);
	sprintf(MSG, "%d confirm: "BYTE_TO_BINARY_PATTERN"\r\n",i, BYTE_TO_BINARY(reader));
	HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
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
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  //HAL_I2C_Mem_Read(&hi2c1, 75<<1, 0x00, I2C_MEMADD_SIZE_8BIT,&reader, 1, 0xFFF);
  uint8_t writer= 0b11111110;
  HAL_I2C_Mem_Write(&hi2c1,75<<1,0x7,I2C_MEMADD_SIZE_8BIT,&writer,1,0xFFF);
  writer= 0b01001000;
  HAL_I2C_Mem_Write(&hi2c1,75<<1,0x0,I2C_MEMADD_SIZE_8BIT,&writer,1,0xFFF);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 1);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
  uint8_t MSG[64] = {"\r\nWelcome to TAD DEBUG\r\n\n\n\n\n"};
  HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
  int globalstate = 3;
  int report_state = 1;
  while (1)
  {
	  read_keys(report_state);
	  if(kb_pushed[0][0] && kb_state[13][4]) globalstate=0;
	  if(kb_pushed[0][1] && kb_state[13][4]) globalstate=1;
	  if(kb_pushed[0][2] && kb_state[13][4]) globalstate=2;
	  if(kb_pushed[0][3] && kb_state[13][4]) globalstate=3;
	  if(kb_pushed[0][4] && kb_state[13][4]) globalstate=4;
	  switch (globalstate){
	  case 0:
		  print_adc();
		  break;
	  case 1:
		  edit_mpreg();
		  break;
	  case 2:
		  if (!report_state) report_state = 1;
		  else report_state = 0;
		  globalstate = 3;
		  break;
	  case 3:
		  break;
	  case 4:
		  HAL_UART_Transmit(&huart2, "list\n", sizeof("list\n"), 100);
		  globalstate = 3;
	  	  break;
	  }
	  //}
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00303D5B;
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

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  huart2.Init.BaudRate = 4800;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, col12_Pin|col13_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_6|col0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, col1_Pin|col2_Pin|col3_Pin|col4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, col5_Pin|col6_Pin|col7_Pin|col8_Pin
                          |col9_Pin|col10_Pin|col11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : col12_Pin col13_Pin */
  GPIO_InitStruct.Pin = col12_Pin|col13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA6 col0_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_6|col0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : row0_Pin row1_Pin row2_Pin row3_Pin */
  GPIO_InitStruct.Pin = row0_Pin|row1_Pin|row2_Pin|row3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : row4_Pin */
  GPIO_InitStruct.Pin = row4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(row4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : col1_Pin col2_Pin col3_Pin col4_Pin */
  GPIO_InitStruct.Pin = col1_Pin|col2_Pin|col3_Pin|col4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : col5_Pin col6_Pin col7_Pin col8_Pin
                           col9_Pin col10_Pin col11_Pin */
  GPIO_InitStruct.Pin = col5_Pin|col6_Pin|col7_Pin|col8_Pin
                          |col9_Pin|col10_Pin|col11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
