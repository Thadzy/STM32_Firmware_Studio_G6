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
#include "stm32g4xx_hal.h"

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
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define Reed_Close_Pin GPIO_PIN_0
#define Reed_Close_GPIO_Port GPIOC
#define Reed_Down_Pin GPIO_PIN_1
#define Reed_Down_GPIO_Port GPIOC
#define Reed_Open_Pin GPIO_PIN_3
#define Reed_Open_GPIO_Port GPIOC
#define Current_Sensor_Pin GPIO_PIN_0
#define Current_Sensor_GPIO_Port GPIOA
#define Gripper_Up_Pin GPIO_PIN_1
#define Gripper_Up_GPIO_Port GPIOA
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define Gripper_Down_Pin GPIO_PIN_4
#define Gripper_Down_GPIO_Port GPIOA
#define E_Stop_Pin GPIO_PIN_5
#define E_Stop_GPIO_Port GPIOA
#define Selected_Mode_Pin GPIO_PIN_6
#define Selected_Mode_GPIO_Port GPIOA
#define Reset_Btn_Pin GPIO_PIN_7
#define Reset_Btn_GPIO_Port GPIOA
#define Reed_Up_Pin GPIO_PIN_0
#define Reed_Up_GPIO_Port GPIOB
#define Relay_Sysmode_Pin GPIO_PIN_11
#define Relay_Sysmode_GPIO_Port GPIOB
#define Relay_MotorPower_Pin GPIO_PIN_12
#define Relay_MotorPower_GPIO_Port GPIOB
#define Relay_SysStatus_Pin GPIO_PIN_9
#define Relay_SysStatus_GPIO_Port GPIOC
#define Motor_PWM_Pin GPIO_PIN_8
#define Motor_PWM_GPIO_Port GPIOA
#define Motor_Direction_Pin GPIO_PIN_9
#define Motor_Direction_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define Proximity_Sensor_Pin GPIO_PIN_9
#define Proximity_Sensor_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
