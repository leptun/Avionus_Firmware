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
#include "stm32f7xx_hal.h"

#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_system.h"
#include "stm32f7xx_ll_exti.h"
#include "stm32f7xx_ll_cortex.h"
#include "stm32f7xx_ll_utils.h"
#include "stm32f7xx_ll_pwr.h"
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_rtc.h"
#include "stm32f7xx_ll_tim.h"
#include "stm32f7xx_ll_gpio.h"

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
#define UI_LED_GPS_Pin GPIO_PIN_2
#define UI_LED_GPS_GPIO_Port GPIOE
#define UI_LED_STATUS_Pin GPIO_PIN_3
#define UI_LED_STATUS_GPIO_Port GPIOE
#define POWER_D5_EN_Pin GPIO_PIN_4
#define POWER_D5_EN_GPIO_Port GPIOE
#define POWER_D5_nFAULT_Pin GPIO_PIN_5
#define POWER_D5_nFAULT_GPIO_Port GPIOE
#define POWER_D1_EN_Pin GPIO_PIN_13
#define POWER_D1_EN_GPIO_Port GPIOC
#define UI_SW_USER_Pin GPIO_PIN_2
#define UI_SW_USER_GPIO_Port GPIOF
#define POWER_D2_EN_Pin GPIO_PIN_11
#define POWER_D2_EN_GPIO_Port GPIOF
#define POWER_D4_EN_Pin GPIO_PIN_12
#define POWER_D4_EN_GPIO_Port GPIOF
#define RF_RX_SWITCH_Pin GPIO_PIN_13
#define RF_RX_SWITCH_GPIO_Port GPIOF
#define RF_TX_SWITCH_Pin GPIO_PIN_14
#define RF_TX_SWITCH_GPIO_Port GPIOF
#define RF_NRESET_Pin GPIO_PIN_15
#define RF_NRESET_GPIO_Port GPIOF
#define RF_NSS_Pin GPIO_PIN_11
#define RF_NSS_GPIO_Port GPIOE
#define UI_LED_RX_Pin GPIO_PIN_10
#define UI_LED_RX_GPIO_Port GPIOD
#define UI_LED_TX_Pin GPIO_PIN_11
#define UI_LED_TX_GPIO_Port GPIOD
#define POWER_LEDs_EN_Pin GPIO_PIN_4
#define POWER_LEDs_EN_GPIO_Port GPIOG
#define POWER_SERVO_5V_AUX_EN_Pin GPIO_PIN_5
#define POWER_SERVO_5V_AUX_EN_GPIO_Port GPIOG
#define POWER_SERVO_5V_AUX_nFAULT_Pin GPIO_PIN_6
#define POWER_SERVO_5V_AUX_nFAULT_GPIO_Port GPIOG
#define POWER_D3_EN_Pin GPIO_PIN_15
#define POWER_D3_EN_GPIO_Port GPIOG
#define UI_BUZZER_Pin GPIO_PIN_9
#define UI_BUZZER_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
