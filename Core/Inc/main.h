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
#define ECU_SPK_CUT_Pin GPIO_PIN_0
#define ECU_SPK_CUT_GPIO_Port GPIOC
#define DAM_LED_Pin GPIO_PIN_2
#define DAM_LED_GPIO_Port GPIOC
#define HEARTBEAT_Pin GPIO_PIN_3
#define HEARTBEAT_GPIO_Port GPIOC
#define TRANS_Pin GPIO_PIN_0
#define TRANS_GPIO_Port GPIOA
#define GRN_LED_Pin GPIO_PIN_5
#define GRN_LED_GPIO_Port GPIOA
#define CLUTCH_POT_Pin GPIO_PIN_0
#define CLUTCH_POT_GPIO_Port GPIOB
#define SHIFT_POT_Pin GPIO_PIN_1
#define SHIFT_POT_GPIO_Port GPIOB
#define UPSHIFT_SOL_Pin GPIO_PIN_6
#define UPSHIFT_SOL_GPIO_Port GPIOC
#define DOWNSHIFT_SOL_Pin GPIO_PIN_7
#define DOWNSHIFT_SOL_GPIO_Port GPIOC
#define CLUTCH_SOL_Pin GPIO_PIN_8
#define CLUTCH_SOL_GPIO_Port GPIOC
#define CLUTCH_SLOW_DROP_Pin GPIO_PIN_9
#define CLUTCH_SLOW_DROP_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
