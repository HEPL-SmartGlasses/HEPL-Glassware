/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define SPI2_SD_CS_Pin GPIO_PIN_0
#define SPI2_SD_CS_GPIO_Port GPIOB
<<<<<<< HEAD
#define BACK_Pin GPIO_PIN_7
#define BACK_GPIO_Port GPIOB
#define BACK_EXTI_IRQn EXTI9_5_IRQn
#define DOWN_Pin GPIO_PIN_3
#define DOWN_GPIO_Port GPIOH
#define DOWN_EXTI_IRQn EXTI3_IRQn
#define UP_Pin GPIO_PIN_8
#define UP_GPIO_Port GPIOB
#define UP_EXTI_IRQn EXTI9_5_IRQn
#define START_Pin GPIO_PIN_9
#define START_GPIO_Port GPIOB
#define START_EXTI_IRQn EXTI9_5_IRQn
=======
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define SPI3_ATTN_Pin GPIO_PIN_6
#define SPI3_ATTN_GPIO_Port GPIOB
#define SPI3_ATTN_EXTI_IRQn EXTI9_5_IRQn
>>>>>>> main

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
