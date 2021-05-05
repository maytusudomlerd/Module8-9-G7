/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Prox_EXIT2_J1_Pin GPIO_PIN_2
#define Prox_EXIT2_J1_GPIO_Port GPIOE
#define Prox_EXIT3_J2_Pin GPIO_PIN_3
#define Prox_EXIT3_J2_GPIO_Port GPIOE
#define Prox_EXIT4_J3_Pin GPIO_PIN_4
#define Prox_EXIT4_J3_GPIO_Port GPIOE
#define Prox_EXIT5_J4_Pin GPIO_PIN_5
#define Prox_EXIT5_J4_GPIO_Port GPIOE
#define Pulse_TIM16_J3_Pin GPIO_PIN_6
#define Pulse_TIM16_J3_GPIO_Port GPIOF
#define Pulse_TIM17_J4_Pin GPIO_PIN_7
#define Pulse_TIM17_J4_GPIO_Port GPIOF
#define Pulse_TIM13_J1_Pin GPIO_PIN_8
#define Pulse_TIM13_J1_GPIO_Port GPIOF
#define Pulse_TIM14_J2_Pin GPIO_PIN_9
#define Pulse_TIM14_J2_GPIO_Port GPIOF
#define DYNAMIXEL_DATA_Pin GPIO_PIN_1
#define DYNAMIXEL_DATA_GPIO_Port GPIOA
#define dYNAMIXEL_TX_Pin GPIO_PIN_2
#define dYNAMIXEL_TX_GPIO_Port GPIOA
#define DYNAMIXEL_RX_Pin GPIO_PIN_3
#define DYNAMIXEL_RX_GPIO_Port GPIOA
#define Enc_TIM3_J2_A_Pin GPIO_PIN_6
#define Enc_TIM3_J2_A_GPIO_Port GPIOA
#define Enc_TIM3_J2_B_Pin GPIO_PIN_7
#define Enc_TIM3_J2_B_GPIO_Port GPIOA
#define RS485_RX_Pin GPIO_PIN_12
#define RS485_RX_GPIO_Port GPIOB
#define RS485_TX_Pin GPIO_PIN_13
#define RS485_TX_GPIO_Port GPIOB
#define USB_TX_Pin GPIO_PIN_14
#define USB_TX_GPIO_Port GPIOB
#define USB_RX_Pin GPIO_PIN_15
#define USB_RX_GPIO_Port GPIOB
#define ST_LINK_TX_Pin GPIO_PIN_8
#define ST_LINK_TX_GPIO_Port GPIOD
#define ST_LINK_RX_Pin GPIO_PIN_9
#define ST_LINK_RX_GPIO_Port GPIOD
#define Enc_TIM4_J3_A_Pin GPIO_PIN_12
#define Enc_TIM4_J3_A_GPIO_Port GPIOD
#define Enc_TIM4_J3_B_Pin GPIO_PIN_13
#define Enc_TIM4_J3_B_GPIO_Port GPIOD
#define Enc_TIM8_J4_A_Pin GPIO_PIN_6
#define Enc_TIM8_J4_A_GPIO_Port GPIOC
#define Enc_TIM8_J4_B_Pin GPIO_PIN_7
#define Enc_TIM8_J4_B_GPIO_Port GPIOC
#define Enc_TIM1_J1_A_Pin GPIO_PIN_8
#define Enc_TIM1_J1_A_GPIO_Port GPIOA
#define Enc_TIM1_J1_B_Pin GPIO_PIN_9
#define Enc_TIM1_J1_B_GPIO_Port GPIOA
#define Chessboard_SCk_Pin GPIO_PIN_10
#define Chessboard_SCk_GPIO_Port GPIOC
#define Chessboard_MISO_Pin GPIO_PIN_11
#define Chessboard_MISO_GPIO_Port GPIOC
#define Chessboard_MOSI_Pin GPIO_PIN_12
#define Chessboard_MOSI_GPIO_Port GPIOC
#define DIR_J4_Pin GPIO_PIN_9
#define DIR_J4_GPIO_Port GPIOG
#define DIR_J3_Pin GPIO_PIN_10
#define DIR_J3_GPIO_Port GPIOG
#define DIR_J2_Pin GPIO_PIN_11
#define DIR_J2_GPIO_Port GPIOG
#define DIR_J1_Pin GPIO_PIN_12
#define DIR_J1_GPIO_Port GPIOG
#define Ena_J4_Pin GPIO_PIN_8
#define Ena_J4_GPIO_Port GPIOB
#define Ena_J3_Pin GPIO_PIN_9
#define Ena_J3_GPIO_Port GPIOB
#define Ena_J2_Pin GPIO_PIN_0
#define Ena_J2_GPIO_Port GPIOE
#define Ena_J1_Pin GPIO_PIN_1
#define Ena_J1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
