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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC1_IN0_DISTANCE_Pin GPIO_PIN_0
#define ADC1_IN0_DISTANCE_GPIO_Port GPIOA
#define TIM2_PWM_CH2_Pin GPIO_PIN_1
#define TIM2_PWM_CH2_GPIO_Port GPIOA
#define TIM2_PWM_CH1_Pin GPIO_PIN_5
#define TIM2_PWM_CH1_GPIO_Port GPIOA
#define TIM3_ENCODER_CH1_Pin GPIO_PIN_6
#define TIM3_ENCODER_CH1_GPIO_Port GPIOA
#define TIM3_ENCODER_CH2_Pin GPIO_PIN_7
#define TIM3_ENCODER_CH2_GPIO_Port GPIOA
#define TIM2_PWM_CH4_Pin GPIO_PIN_2
#define TIM2_PWM_CH4_GPIO_Port GPIOB
#define TIM2_PWM_CH3_Pin GPIO_PIN_10
#define TIM2_PWM_CH3_GPIO_Port GPIOB
#define TIM8_ENCODER_CH1_Pin GPIO_PIN_6
#define TIM8_ENCODER_CH1_GPIO_Port GPIOC
#define TIM8_ENCODER_CH2_Pin GPIO_PIN_7
#define TIM8_ENCODER_CH2_GPIO_Port GPIOC
#define GPIO_MOT1_DIR_Pin GPIO_PIN_8
#define GPIO_MOT1_DIR_GPIO_Port GPIOA
#define GPIO_MOT2_DIR_Pin GPIO_PIN_9
#define GPIO_MOT2_DIR_GPIO_Port GPIOA
#define GPIO_MOT3_DIR_Pin GPIO_PIN_10
#define GPIO_MOT3_DIR_GPIO_Port GPIOC
#define GPIO_MOT4_DIR_Pin GPIO_PIN_11
#define GPIO_MOT4_DIR_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
