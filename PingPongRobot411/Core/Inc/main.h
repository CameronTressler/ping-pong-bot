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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Ultrasonic_echo_Pin GPIO_PIN_0
#define Ultrasonic_echo_GPIO_Port GPIOC
#define Ultrasonic_echo_EXTI_IRQn EXTI0_IRQn
#define Ultrasonic_echoC1_Pin GPIO_PIN_1
#define Ultrasonic_echoC1_GPIO_Port GPIOC
#define Ultrasonic_echoC1_EXTI_IRQn EXTI1_IRQn
#define N64_transmit_Pin GPIO_PIN_0
#define N64_transmit_GPIO_Port GPIOA
#define N64_receive_Pin GPIO_PIN_1
#define N64_receive_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Solenoid_enable__gate0_Pin GPIO_PIN_5
#define Solenoid_enable__gate0_GPIO_Port GPIOC
#define Hbridge_3_NDIR_Pin GPIO_PIN_6
#define Hbridge_3_NDIR_GPIO_Port GPIOC
#define Hbridge_3_DIR_Pin GPIO_PIN_7
#define Hbridge_3_DIR_GPIO_Port GPIOC
#define Hbridge_2_NDIR_Pin GPIO_PIN_8
#define Hbridge_2_NDIR_GPIO_Port GPIOC
#define Hbridge_0_NDIR_Pin GPIO_PIN_11
#define Hbridge_0_NDIR_GPIO_Port GPIOA
#define Hbridge_0_DIR_Pin GPIO_PIN_12
#define Hbridge_0_DIR_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Hbridge_2_DIR_Pin GPIO_PIN_10
#define Hbridge_2_DIR_GPIO_Port GPIOC
#define Hbridge_1_NDIR_Pin GPIO_PIN_11
#define Hbridge_1_NDIR_GPIO_Port GPIOC
#define Hbridge_1_DIR_Pin GPIO_PIN_12
#define Hbridge_1_DIR_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
