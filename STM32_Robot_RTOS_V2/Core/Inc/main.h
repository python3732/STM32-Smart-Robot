/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f1xx_hal.h"

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
#define L1_Pin GPIO_PIN_0
#define L1_GPIO_Port GPIOF
#define L2_Pin GPIO_PIN_1
#define L2_GPIO_Port GPIOF
#define L3_Pin GPIO_PIN_2
#define L3_GPIO_Port GPIOF
#define L4_Pin GPIO_PIN_3
#define L4_GPIO_Port GPIOF
#define L5_Pin GPIO_PIN_4
#define L5_GPIO_Port GPIOF
#define L6_Pin GPIO_PIN_5
#define L6_GPIO_Port GPIOF
#define L7_Pin GPIO_PIN_6
#define L7_GPIO_Port GPIOF
#define L8_Pin GPIO_PIN_7
#define L8_GPIO_Port GPIOF
#define US_ECHO_Pin GPIO_PIN_14
#define US_ECHO_GPIO_Port GPIOB
#define US_TRIG_Pin GPIO_PIN_8
#define US_TRIG_GPIO_Port GPIOD
#define TRACK_L_Pin GPIO_PIN_4
#define TRACK_L_GPIO_Port GPIOG
#define TRACK_R_Pin GPIO_PIN_6
#define TRACK_R_GPIO_Port GPIOG
#define TRACK_M_Pin GPIO_PIN_8
#define TRACK_M_GPIO_Port GPIOG
#define AIN2_Pin GPIO_PIN_11
#define AIN2_GPIO_Port GPIOC
#define AIN1_Pin GPIO_PIN_0
#define AIN1_GPIO_Port GPIOD
#define EN_A_Pin GPIO_PIN_2
#define EN_A_GPIO_Port GPIOD
#define EN_B_Pin GPIO_PIN_4
#define EN_B_GPIO_Port GPIOD
#define BIN2_Pin GPIO_PIN_6
#define BIN2_GPIO_Port GPIOD
#define BIN1_Pin GPIO_PIN_9
#define BIN1_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
