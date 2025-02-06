/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define AUD_nSDMODE_Pin GPIO_PIN_6
#define AUD_nSDMODE_GPIO_Port GPIOE
#define SW_IC_INT_Pin GPIO_PIN_3
#define SW_IC_INT_GPIO_Port GPIOF
#define SW_IC_INT_EXTI_IRQn EXTI3_IRQn
#define MCU_USB_OC__DET_Pin GPIO_PIN_5
#define MCU_USB_OC__DET_GPIO_Port GPIOA
#define MCU_USB_OC__DET_EXTI_IRQn EXTI9_5_IRQn
#define LED_DRV_nRESET_Pin GPIO_PIN_8
#define LED_DRV_nRESET_GPIO_Port GPIOD
#define LED_DRV_nOE_Pin GPIO_PIN_9
#define LED_DRV_nOE_GPIO_Port GPIOD
#define ASSIST_BTN_P_Pin GPIO_PIN_10
#define ASSIST_BTN_P_GPIO_Port GPIOD
#define ASSIST_BTN_P_EXTI_IRQn EXTI15_10_IRQn
#define ASSIST_BTN_N_Pin GPIO_PIN_12
#define ASSIST_BTN_N_GPIO_Port GPIOD
#define ASSIST_BTN_N_EXTI_IRQn EXTI15_10_IRQn
#define MCU_24V_MOTOR_ON_Pin GPIO_PIN_2
#define MCU_24V_MOTOR_ON_GPIO_Port GPIOG
#define MC_5V_PWR_EN_Pin GPIO_PIN_3
#define MC_5V_PWR_EN_GPIO_Port GPIOG
#define WIDM_5V_PWR_EN_Pin GPIO_PIN_4
#define WIDM_5V_PWR_EN_GPIO_Port GPIOG
#define MCU_SW_CLR_Pin GPIO_PIN_7
#define MCU_SW_CLR_GPIO_Port GPIOG
#define nPB_IN_MCU_Pin GPIO_PIN_8
#define nPB_IN_MCU_GPIO_Port GPIOG
#define MC_FDCAN1_RX_Pin GPIO_PIN_0
#define MC_FDCAN1_RX_GPIO_Port GPIOD
#define MC_FDCAN1_TX_Pin GPIO_PIN_1
#define MC_FDCAN1_TX_GPIO_Port GPIOD
#define MCU_USB_PWR_ON_Pin GPIO_PIN_6
#define MCU_USB_PWR_ON_GPIO_Port GPIOD
#define LED_DRV_SPI_MOSI_Pin GPIO_PIN_7
#define LED_DRV_SPI_MOSI_GPIO_Port GPIOD
#define LED_DRV_SPI_MISO_Pin GPIO_PIN_9
#define LED_DRV_SPI_MISO_GPIO_Port GPIOG
#define LED_DRV_SPI_NSS_Pin GPIO_PIN_10
#define LED_DRV_SPI_NSS_GPIO_Port GPIOG
#define LED_DRV_SPI_SCK_Pin GPIO_PIN_11
#define LED_DRV_SPI_SCK_GPIO_Port GPIOG
#define MC_24V_PWR_EN_Pin GPIO_PIN_12
#define MC_24V_PWR_EN_GPIO_Port GPIOG
#define ID_SET1_Pin GPIO_PIN_2
#define ID_SET1_GPIO_Port GPIOF
#define ID_SET2_Pin GPIO_PIN_3
#define ID_SET2_GPIO_Port GPIOF
#define ID_SET3_Pin GPIO_PIN_4
#define ID_SET3_GPIO_Port GPIOF
#define ID_SET4_Pin GPIO_PIN_5
#define ID_SET4_GPIO_Port GPIOF

#define STATUS_LED_B_Pin GPIO_PIN_13
#define STATUS_LED_B_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
