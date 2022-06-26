/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define D6_Pin GPIO_PIN_2
#define D6_GPIO_Port GPIOE
#define D7_Pin GPIO_PIN_3
#define D7_GPIO_Port GPIOE
#define RS_Pin GPIO_PIN_4
#define RS_GPIO_Port GPIOE
#define EN_Pin GPIO_PIN_5
#define EN_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_2
#define KEY1_GPIO_Port GPIOG
#define KEY2_Pin GPIO_PIN_3
#define KEY2_GPIO_Port GPIOG
#define TX1_EN_Pin GPIO_PIN_12
#define TX1_EN_GPIO_Port GPIOA
#define TX2_EN_Pin GPIO_PIN_1
#define TX2_EN_GPIO_Port GPIOD
#define D4_Pin GPIO_PIN_0
#define D4_GPIO_Port GPIOE
#define D5_Pin GPIO_PIN_1
#define D5_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

#define DIGIT1_E 0
#define DIGIT2_E 1

#define EVENT_RESET 0
#define EVENT_KEY1_PRESSED 1
#define EVENT_KEY2_PRESSED 2
#define EVENT_KEY1_RELEASED 3
#define EVENT_KEY2_RELEASED 4
#define EVENT_RX_COMPLETE 5
#define EVENT_QUERRY 6

#define ERROR_RESET 0
#define ERROR_CHECKSUM 1
#define ERROR_UNKNOWN_ADDR 2 

#define KEY_PRESSED 0b1100     //
#define KEY_RELEASED 0b0011

#define STATE_ASSIGNED_ADDR 0
#define STATE_OPERATION 1

#define ADDR 0x10



#define CHECKSUM_ERROR 0
#define CHECKSUM_NO_ERROR 1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
