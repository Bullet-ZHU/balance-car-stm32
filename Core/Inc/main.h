/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define Ganwei_AD0_Pin GPIO_PIN_0
#define Ganwei_AD0_GPIO_Port GPIOA
#define Ganwei_AD1_Pin GPIO_PIN_1
#define Ganwei_AD1_GPIO_Port GPIOA
#define Ganwei_AD2_Pin GPIO_PIN_2
#define Ganwei_AD2_GPIO_Port GPIOA
#define OLED_SCL_Pin GPIO_PIN_4
#define OLED_SCL_GPIO_Port GPIOA
#define OLED_SDA_Pin GPIO_PIN_5
#define OLED_SDA_GPIO_Port GPIOA
#define G356_TX_Pin GPIO_PIN_10
#define G356_TX_GPIO_Port GPIOB
#define G356_RX_Pin GPIO_PIN_11
#define G356_RX_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_12
#define BIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_13
#define BIN1_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_14
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_15
#define AIN2_GPIO_Port GPIOB
#define PWMB_Pin GPIO_PIN_9
#define PWMB_GPIO_Port GPIOA
#define Encoder_E1A_Pin GPIO_PIN_15
#define Encoder_E1A_GPIO_Port GPIOA
#define Encoder_E1B_Pin GPIO_PIN_3
#define Encoder_E1B_GPIO_Port GPIOB
#define Encoder_E2A_Pin GPIO_PIN_4
#define Encoder_E2A_GPIO_Port GPIOB
#define Encoder_E2B_Pin GPIO_PIN_5
#define Encoder_E2B_GPIO_Port GPIOB
#define HC_05_TX_Pin GPIO_PIN_6
#define HC_05_TX_GPIO_Port GPIOB
#define HC_05_RX_Pin GPIO_PIN_7
#define HC_05_RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
