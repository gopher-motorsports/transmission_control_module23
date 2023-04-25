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
#define SWITCH_FAULT_3V3_Pin GPIO_PIN_13
#define SWITCH_FAULT_3V3_GPIO_Port GPIOC
#define SWITCH_FAULT_5V_Pin GPIO_PIN_15
#define SWITCH_FAULT_5V_GPIO_Port GPIOC
#define GEAR_POS_POT_Pin GPIO_PIN_0
#define GEAR_POS_POT_GPIO_Port GPIOC
#define SHIFT_POT_Pin GPIO_PIN_1
#define SHIFT_POT_GPIO_Port GPIOC
#define CLUTCH_POT_Pin GPIO_PIN_2
#define CLUTCH_POT_GPIO_Port GPIOC
#define AUX1_T_Pin GPIO_PIN_3
#define AUX1_T_GPIO_Port GPIOC
#define DOWNSHIFT_SOL_Pin GPIO_PIN_0
#define DOWNSHIFT_SOL_GPIO_Port GPIOA
#define CLUTCH_SOL_Pin GPIO_PIN_1
#define CLUTCH_SOL_GPIO_Port GPIOA
#define SLOW_CLUTCH_SOL_Pin GPIO_PIN_2
#define SLOW_CLUTCH_SOL_GPIO_Port GPIOA
#define UPSHIFT_SOL_Pin GPIO_PIN_3
#define UPSHIFT_SOL_GPIO_Port GPIOA
#define SPK_CUT_Pin GPIO_PIN_4
#define SPK_CUT_GPIO_Port GPIOA
#define TRANS_SPEED_Pin GPIO_PIN_5
#define TRANS_SPEED_GPIO_Port GPIOA
#define AUX1_C_Pin GPIO_PIN_6
#define AUX1_C_GPIO_Port GPIOA
#define HBEAT_Pin GPIO_PIN_7
#define HBEAT_GPIO_Port GPIOA
#define GSENSE_LED_Pin GPIO_PIN_4
#define GSENSE_LED_GPIO_Port GPIOC
#define AUX2_C_Pin GPIO_PIN_1
#define AUX2_C_GPIO_Port GPIOB
#define FAULT_LED_Pin GPIO_PIN_9
#define FAULT_LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
