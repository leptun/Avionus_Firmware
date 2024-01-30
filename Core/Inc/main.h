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
#include "stm32f7xx_ll_adc.h"
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_system.h"
#include "stm32f7xx_ll_exti.h"
#include "stm32f7xx_ll_cortex.h"
#include "stm32f7xx_ll_utils.h"
#include "stm32f7xx_ll_pwr.h"
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
void MX_SDMMC2_SD_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UI_LED_1_Pin GPIO_PIN_2
#define UI_LED_1_GPIO_Port GPIOE
#define UI_LED_2_Pin GPIO_PIN_3
#define UI_LED_2_GPIO_Port GPIOE
#define POWER_D5_EN_Pin GPIO_PIN_4
#define POWER_D5_EN_GPIO_Port GPIOE
#define POWER_D5_nFAULT_Pin GPIO_PIN_5
#define POWER_D5_nFAULT_GPIO_Port GPIOE
#define POWER_D5_nFAULT_EXTI_IRQn EXTI9_5_IRQn
#define POWER_D1_EN_Pin GPIO_PIN_13
#define POWER_D1_EN_GPIO_Port GPIOC
#define UI_SW_USER_Pin GPIO_PIN_2
#define UI_SW_USER_GPIO_Port GPIOF
#define UI_SW_USER_EXTI_IRQn EXTI2_IRQn
#define MEAS_D3_3V3_Pin GPIO_PIN_3
#define MEAS_D3_3V3_GPIO_Port GPIOF
#define MEAS_D4_3V3_Pin GPIO_PIN_4
#define MEAS_D4_3V3_GPIO_Port GPIOF
#define MEAS_D5_5V_Pin GPIO_PIN_5
#define MEAS_D5_5V_GPIO_Port GPIOF
#define MEAS_D2_1V8_Pin GPIO_PIN_10
#define MEAS_D2_1V8_GPIO_Port GPIOF
#define MEAS_POWER_IN_Pin GPIO_PIN_0
#define MEAS_POWER_IN_GPIO_Port GPIOC
#define MEAS_I_D1_5V_Pin GPIO_PIN_1
#define MEAS_I_D1_5V_GPIO_Port GPIOC
#define MEAS_D1_5V_Pin GPIO_PIN_2
#define MEAS_D1_5V_GPIO_Port GPIOC
#define MEAS_D2_3V3_Pin GPIO_PIN_3
#define MEAS_D2_3V3_GPIO_Port GPIOC
#define SERVO_CH15_Pin GPIO_PIN_0
#define SERVO_CH15_GPIO_Port GPIOA
#define SERVO_CH14_Pin GPIO_PIN_1
#define SERVO_CH14_GPIO_Port GPIOA
#define SERVO_CH13_Pin GPIO_PIN_2
#define SERVO_CH13_GPIO_Port GPIOA
#define SERVO_CH12_Pin GPIO_PIN_3
#define SERVO_CH12_GPIO_Port GPIOA
#define MEAS_SERVO_Pin GPIO_PIN_4
#define MEAS_SERVO_GPIO_Port GPIOC
#define MEAS_I_SERVO_5V5_Pin GPIO_PIN_5
#define MEAS_I_SERVO_5V5_GPIO_Port GPIOC
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
#define UI_LED_3_Pin GPIO_PIN_10
#define UI_LED_3_GPIO_Port GPIOD
#define UI_LED_4_Pin GPIO_PIN_11
#define UI_LED_4_GPIO_Port GPIOD
#define SERVO_CH5_Pin GPIO_PIN_12
#define SERVO_CH5_GPIO_Port GPIOD
#define SERVO_CH4_Pin GPIO_PIN_13
#define SERVO_CH4_GPIO_Port GPIOD
#define SERVO_CH6_Pin GPIO_PIN_14
#define SERVO_CH6_GPIO_Port GPIOD
#define SERVO_CH7_Pin GPIO_PIN_15
#define SERVO_CH7_GPIO_Port GPIOD
#define POWER_LEDs_EN_Pin GPIO_PIN_4
#define POWER_LEDs_EN_GPIO_Port GPIOG
#define POWER_SERVO_5V_AUX_EN_Pin GPIO_PIN_5
#define POWER_SERVO_5V_AUX_EN_GPIO_Port GPIOG
#define POWER_SERVO_5V_AUX_nFAULT_Pin GPIO_PIN_6
#define POWER_SERVO_5V_AUX_nFAULT_GPIO_Port GPIOG
#define POWER_SERVO_5V_AUX_nFAULT_EXTI_IRQn EXTI9_5_IRQn
#define SERVO_CH0_Pin GPIO_PIN_6
#define SERVO_CH0_GPIO_Port GPIOC
#define SERVO_CH1_Pin GPIO_PIN_7
#define SERVO_CH1_GPIO_Port GPIOC
#define SERVO_CH2_Pin GPIO_PIN_8
#define SERVO_CH2_GPIO_Port GPIOC
#define SERVO_CH3_Pin GPIO_PIN_9
#define SERVO_CH3_GPIO_Port GPIOC
#define SERVO_CH8_Pin GPIO_PIN_8
#define SERVO_CH8_GPIO_Port GPIOA
#define SERVO_CH9_Pin GPIO_PIN_9
#define SERVO_CH9_GPIO_Port GPIOA
#define SERVO_CH10_Pin GPIO_PIN_10
#define SERVO_CH10_GPIO_Port GPIOA
#define SERVO_CH11_Pin GPIO_PIN_11
#define SERVO_CH11_GPIO_Port GPIOA
#define SD_CARD_CD_Pin GPIO_PIN_13
#define SD_CARD_CD_GPIO_Port GPIOG
#define SD_CARD_CD_EXTI_IRQn EXTI15_10_IRQn
#define SD_CARD_WP_Pin GPIO_PIN_14
#define SD_CARD_WP_GPIO_Port GPIOG
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
