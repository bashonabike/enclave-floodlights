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
#include "stm32f3xx_hal.h"

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
#define Button6_Pin GPIO_PIN_0
#define Button6_GPIO_Port GPIOF
#define Button6_EXTI_IRQn EXTI0_IRQn
#define Button5_Pin GPIO_PIN_1
#define Button5_GPIO_Port GPIOF
#define Button5_EXTI_IRQn EXTI1_IRQn
#define Knob1_Pin GPIO_PIN_0
#define Knob1_GPIO_Port GPIOA
#define Knob2_Pin GPIO_PIN_1
#define Knob2_GPIO_Port GPIOA
#define Knob8_Pin GPIO_PIN_2
#define Knob8_GPIO_Port GPIOA
#define Knob3_Pin GPIO_PIN_3
#define Knob3_GPIO_Port GPIOA
#define Knob4_Pin GPIO_PIN_4
#define Knob4_GPIO_Port GPIOA
#define Knob5_Pin GPIO_PIN_5
#define Knob5_GPIO_Port GPIOA
#define Knob5A6_Pin GPIO_PIN_6
#define Knob5A6_GPIO_Port GPIOA
#define Knob6_Pin GPIO_PIN_7
#define Knob6_GPIO_Port GPIOA
#define Floodlight2Grn_Pin GPIO_PIN_0
#define Floodlight2Grn_GPIO_Port GPIOB
#define Floodlight1Grn_Pin GPIO_PIN_1
#define Floodlight1Grn_GPIO_Port GPIOB
#define Button4_Pin GPIO_PIN_8
#define Button4_GPIO_Port GPIOA
#define Button4_EXTI_IRQn EXTI9_5_IRQn
#define Mains_Pin GPIO_PIN_9
#define Mains_GPIO_Port GPIOA
#define Mains_EXTI_IRQn EXTI9_5_IRQn
#define Floodlight2Red_Pin GPIO_PIN_10
#define Floodlight2Red_GPIO_Port GPIOA
#define Button3_Pin GPIO_PIN_11
#define Button3_GPIO_Port GPIOA
#define Floodlight2Blu_Pin GPIO_PIN_12
#define Floodlight2Blu_GPIO_Port GPIOA
#define IndicLED_Pin GPIO_PIN_3
#define IndicLED_GPIO_Port GPIOB
#define Button1_Pin GPIO_PIN_4
#define Button1_GPIO_Port GPIOB
#define Button1_EXTI_IRQn EXTI4_IRQn
#define Button2_Pin GPIO_PIN_5
#define Button2_GPIO_Port GPIOB
#define Button2_EXTI_IRQn EXTI9_5_IRQn
#define Floodlight1Blu_Pin GPIO_PIN_6
#define Floodlight1Blu_GPIO_Port GPIOB
#define Floodlight1Red_Pin GPIO_PIN_7
#define Floodlight1Red_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
