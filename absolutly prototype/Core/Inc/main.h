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
#define Proximity_J1_Pin GPIO_PIN_2
#define Proximity_J1_GPIO_Port GPIOE
#define Proximity_J2_Pin GPIO_PIN_3
#define Proximity_J2_GPIO_Port GPIOE
#define Proximity_J3_Pin GPIO_PIN_4
#define Proximity_J3_GPIO_Port GPIOE
#define Proximity_J4_Pin GPIO_PIN_5
#define Proximity_J4_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define PC_Rx_Pin GPIO_PIN_6
#define PC_Rx_GPIO_Port GPIOF
#define PC_Tx_Pin GPIO_PIN_7
#define PC_Tx_GPIO_Port GPIOF
#define STEP_J2_Pin GPIO_PIN_9
#define STEP_J2_GPIO_Port GPIOF
#define INC_J1_A_Pin GPIO_PIN_0
#define INC_J1_A_GPIO_Port GPIOA
#define INC_J1_B_Pin GPIO_PIN_1
#define INC_J1_B_GPIO_Port GPIOA
#define INC_J2_A_Pin GPIO_PIN_2
#define INC_J2_A_GPIO_Port GPIOA
#define INC_J2_B_Pin GPIO_PIN_3
#define INC_J2_B_GPIO_Port GPIOA
#define STEP_J1_Pin GPIO_PIN_6
#define STEP_J1_GPIO_Port GPIOA
#define DIR_J1_Pin GPIO_PIN_1
#define DIR_J1_GPIO_Port GPIOB
#define DIR_J2_Pin GPIO_PIN_2
#define DIR_J2_GPIO_Port GPIOB
#define DIR_J3_Pin GPIO_PIN_11
#define DIR_J3_GPIO_Port GPIOF
#define DIR_J4_Pin GPIO_PIN_12
#define DIR_J4_GPIO_Port GPIOF
#define EN_J1_Pin GPIO_PIN_13
#define EN_J1_GPIO_Port GPIOF
#define EN_J2_Pin GPIO_PIN_14
#define EN_J2_GPIO_Port GPIOF
#define EN_J3_Pin GPIO_PIN_15
#define EN_J3_GPIO_Port GPIOF
#define EN_J4_Pin GPIO_PIN_0
#define EN_J4_GPIO_Port GPIOG
#define Encoder_J1_A_Pin GPIO_PIN_9
#define Encoder_J1_A_GPIO_Port GPIOE
#define Encoder_J1_B_Pin GPIO_PIN_11
#define Encoder_J1_B_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define Encoder_J4_A_Pin GPIO_PIN_12
#define Encoder_J4_A_GPIO_Port GPIOD
#define Encoder_J4_B_Pin GPIO_PIN_13
#define Encoder_J4_B_GPIO_Port GPIOD
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define INC_J3_A_Pin GPIO_PIN_6
#define INC_J3_A_GPIO_Port GPIOC
#define INC_J3_B_Pin GPIO_PIN_7
#define INC_J3_B_GPIO_Port GPIOC
#define INC_J4_A_Pin GPIO_PIN_8
#define INC_J4_A_GPIO_Port GPIOC
#define INC_J4_B_Pin GPIO_PIN_9
#define INC_J4_B_GPIO_Port GPIOC
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define Dynamixel_data_Pin GPIO_PIN_13
#define Dynamixel_data_GPIO_Port GPIOA
#define Encoder_J2_A_Pin GPIO_PIN_15
#define Encoder_J2_A_GPIO_Port GPIOA
#define Dynamixel_Tx_Pin GPIO_PIN_10
#define Dynamixel_Tx_GPIO_Port GPIOC
#define Dynamixel_Rx_Pin GPIO_PIN_11
#define Dynamixel_Rx_GPIO_Port GPIOC
#define Chessboard_Encoder_MOSI_Pin GPIO_PIN_7
#define Chessboard_Encoder_MOSI_GPIO_Port GPIOD
#define Chessboard_Encoder_MISO_Pin GPIO_PIN_9
#define Chessboard_Encoder_MISO_GPIO_Port GPIOG
#define Chessboard_Encoder_SCK_Pin GPIO_PIN_11
#define Chessboard_Encoder_SCK_GPIO_Port GPIOG
#define Encoder_J2_B_Pin GPIO_PIN_3
#define Encoder_J2_B_GPIO_Port GPIOB
#define Encoder_J3_A_Pin GPIO_PIN_4
#define Encoder_J3_A_GPIO_Port GPIOB
#define Encoder_J3_B_Pin GPIO_PIN_5
#define Encoder_J3_B_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define STEPJ4_Pin GPIO_PIN_8
#define STEPJ4_GPIO_Port GPIOB
#define STEP_J4_Pin GPIO_PIN_9
#define STEP_J4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
