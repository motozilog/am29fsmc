/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define UP_Pin GPIO_PIN_0
#define UP_GPIO_Port GPIOC
#define DOWN_Pin GPIO_PIN_1
#define DOWN_GPIO_Port GPIOC
#define LEFT_Pin GPIO_PIN_2
#define LEFT_GPIO_Port GPIOC
#define RIGHT_Pin GPIO_PIN_3
#define RIGHT_GPIO_Port GPIOC
#define OK_Pin GPIO_PIN_4
#define OK_GPIO_Port GPIOC
#define CANCEL_Pin GPIO_PIN_5
#define CANCEL_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_0
#define LED_G_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_1
#define LED_R_GPIO_Port GPIOB
#define SDIO_CD_Pin GPIO_PIN_8
#define SDIO_CD_GPIO_Port GPIOA
#define A26_Pin GPIO_PIN_15
#define A26_GPIO_Port GPIOG
#define A27_Pin GPIO_PIN_3
#define A27_GPIO_Port GPIOB
#define A28_Pin GPIO_PIN_4
#define A28_GPIO_Port GPIOB
#define A29_Pin GPIO_PIN_5
#define A29_GPIO_Port GPIOB
#define A30_Pin GPIO_PIN_8
#define A30_GPIO_Port GPIOB
#define A31_Pin GPIO_PIN_9
#define A31_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
