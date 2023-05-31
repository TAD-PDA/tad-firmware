/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define col12_Pin GPIO_PIN_13
#define col12_GPIO_Port GPIOC
#define col13_Pin GPIO_PIN_14
#define col13_GPIO_Port GPIOC
#define row0_Pin GPIO_PIN_12
#define row0_GPIO_Port GPIOB
#define row1_Pin GPIO_PIN_13
#define row1_GPIO_Port GPIOB
#define row2_Pin GPIO_PIN_14
#define row2_GPIO_Port GPIOB
#define row3_Pin GPIO_PIN_15
#define row3_GPIO_Port GPIOB
#define row4_Pin GPIO_PIN_8
#define row4_GPIO_Port GPIOA
#define col0_Pin GPIO_PIN_15
#define col0_GPIO_Port GPIOA
#define col1_Pin GPIO_PIN_0
#define col1_GPIO_Port GPIOD
#define col2_Pin GPIO_PIN_1
#define col2_GPIO_Port GPIOD
#define col3_Pin GPIO_PIN_2
#define col3_GPIO_Port GPIOD
#define col4_Pin GPIO_PIN_3
#define col4_GPIO_Port GPIOD
#define col5_Pin GPIO_PIN_3
#define col5_GPIO_Port GPIOB
#define col6_Pin GPIO_PIN_4
#define col6_GPIO_Port GPIOB
#define col7_Pin GPIO_PIN_5
#define col7_GPIO_Port GPIOB
#define col8_Pin GPIO_PIN_6
#define col8_GPIO_Port GPIOB
#define col9_Pin GPIO_PIN_7
#define col9_GPIO_Port GPIOB
#define col10_Pin GPIO_PIN_8
#define col10_GPIO_Port GPIOB
#define col11_Pin GPIO_PIN_9
#define col11_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
