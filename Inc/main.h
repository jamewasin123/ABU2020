/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include "stdio.h"
#include <stdlib.h>
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENC_A_T5_Pin GPIO_PIN_0
#define ENC_A_T5_GPIO_Port GPIOA
#define ENC_B_T5_Pin GPIO_PIN_1
#define ENC_B_T5_GPIO_Port GPIOA
#define ENC_A_T3_Pin GPIO_PIN_6
#define ENC_A_T3_GPIO_Port GPIOA
#define ENC_B_T3_Pin GPIO_PIN_7
#define ENC_B_T3_GPIO_Port GPIOA
#define ENC_A_T1_Pin GPIO_PIN_9
#define ENC_A_T1_GPIO_Port GPIOE
#define ENC_B__T1_Pin GPIO_PIN_11
#define ENC_B__T1_GPIO_Port GPIOE
#define M4_B_Pin GPIO_PIN_8
#define M4_B_GPIO_Port GPIOD
#define M4_A_Pin GPIO_PIN_9
#define M4_A_GPIO_Port GPIOD
#define M3_B_Pin GPIO_PIN_10
#define M3_B_GPIO_Port GPIOD
#define M3_A_Pin GPIO_PIN_11
#define M3_A_GPIO_Port GPIOD
#define M2_B_Pin GPIO_PIN_12
#define M2_B_GPIO_Port GPIOD
#define M2_A_Pin GPIO_PIN_13
#define M2_A_GPIO_Port GPIOD
#define M1_B_Pin GPIO_PIN_14
#define M1_B_GPIO_Port GPIOD
#define M1_A_Pin GPIO_PIN_15
#define M1_A_GPIO_Port GPIOD
#define M4_PWM_Pin GPIO_PIN_6
#define M4_PWM_GPIO_Port GPIOC
#define M2_PWM_Pin GPIO_PIN_8
#define M2_PWM_GPIO_Port GPIOC
#define M3_PWM_Pin GPIO_PIN_9
#define M3_PWM_GPIO_Port GPIOC
#define ENC_A_T2_Pin GPIO_PIN_15
#define ENC_A_T2_GPIO_Port GPIOA
#define ENC_B_T2_Pin GPIO_PIN_3
#define ENC_B_T2_GPIO_Port GPIOB
#define M1_PWM_Pin GPIO_PIN_9
#define M1_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
