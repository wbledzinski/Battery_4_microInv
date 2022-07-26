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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BATT_BCKP_DRV_Pin GPIO_PIN_4
#define BATT_BCKP_DRV_GPIO_Port GPIOA
#define BAT_SWITCH_PWM_Pin GPIO_PIN_5
#define BAT_SWITCH_PWM_GPIO_Port GPIOA
#define BAT_SWITCH_OFF_Pin GPIO_PIN_6
#define BAT_SWITCH_OFF_GPIO_Port GPIOA
#define INV_SWITCH_DRV_Pin GPIO_PIN_7
#define INV_SWITCH_DRV_GPIO_Port GPIOA
#define C3_Pin GPIO_PIN_4
#define C3_GPIO_Port GPIOC
#define C1_Pin GPIO_PIN_5
#define C1_GPIO_Port GPIOC
#define C4_Pin GPIO_PIN_0
#define C4_GPIO_Port GPIOB
#define C2_Pin GPIO_PIN_1
#define C2_GPIO_Port GPIOB
#define CD_Pin GPIO_PIN_2
#define CD_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_12
#define LED4_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_13
#define LED3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOB
#define MEAS_PWR_Pin GPIO_PIN_8
#define MEAS_PWR_GPIO_Port GPIOA
#define EXT_O_Pin GPIO_PIN_8
#define EXT_O_GPIO_Port GPIOB
#define EXT_I_Pin GPIO_PIN_9
#define EXT_I_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
