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
#define SPI1_XBEE_CS_Pin GPIO_PIN_4
#define SPI1_XBEE_CS_GPIO_Port GPIOA
#define Up_Button_Pin GPIO_PIN_0
#define Up_Button_GPIO_Port GPIOB
#define Up_Button_EXTI_IRQn EXTI0_IRQn
#define Menu_Button_Pin GPIO_PIN_1
#define Menu_Button_GPIO_Port GPIOB
#define Menu_Button_EXTI_IRQn EXTI1_IRQn
#define Down_Button_Pin GPIO_PIN_2
#define Down_Button_GPIO_Port GPIOB
#define Down_Button_EXTI_IRQn EXTI2_IRQn
#define SPI2_SD_CS_Pin GPIO_PIN_11
#define SPI2_SD_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
