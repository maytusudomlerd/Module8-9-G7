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
#define Proximity_1_Pin GPIO_PIN_2
#define Proximity_1_GPIO_Port GPIOE
#define Proximity_2_Pin GPIO_PIN_3
#define Proximity_2_GPIO_Port GPIOE
#define Proximity_3_Pin GPIO_PIN_4
#define Proximity_3_GPIO_Port GPIOE
#define Proximity_4_Pin GPIO_PIN_5
#define Proximity_4_GPIO_Port GPIOE
#define PUL_J3_Pin GPIO_PIN_6
#define PUL_J3_GPIO_Port GPIOF
#define PUL_J4_Pin GPIO_PIN_7
#define PUL_J4_GPIO_Port GPIOF
#define PUL_J1_Pin GPIO_PIN_8
#define PUL_J1_GPIO_Port GPIOF
#define PUL_J2_Pin GPIO_PIN_9
#define PUL_J2_GPIO_Port GPIOF
#define DIR_J1_Pin GPIO_PIN_1
#define DIR_J1_GPIO_Port GPIOB
#define DIR_J2_Pin GPIO_PIN_2
#define DIR_J2_GPIO_Port GPIOB
#define DIR_J3_Pin GPIO_PIN_11
#define DIR_J3_GPIO_Port GPIOF
#define DIR_J4_Pin GPIO_PIN_12
#define DIR_J4_GPIO_Port GPIOF
#define ENA_J1_Pin GPIO_PIN_13
#define ENA_J1_GPIO_Port GPIOF
#define ENA_J2_Pin GPIO_PIN_14
#define ENA_J2_GPIO_Port GPIOF
#define ENA_J3_Pin GPIO_PIN_15
#define ENA_J3_GPIO_Port GPIOF
#define ENA_J4_Pin GPIO_PIN_0
#define ENA_J4_GPIO_Port GPIOG
#define ST_LINK_TX_Pin GPIO_PIN_8
#define ST_LINK_TX_GPIO_Port GPIOD
#define ST_LINK_Rx_Pin GPIO_PIN_9
#define ST_LINK_Rx_GPIO_Port GPIOD
#define Dynamixel_Data_Pin GPIO_PIN_15
#define Dynamixel_Data_GPIO_Port GPIOA
#define Dynamixel_Tx_Pin GPIO_PIN_10
#define Dynamixel_Tx_GPIO_Port GPIOC
#define Dynamixel_Rx_Pin GPIO_PIN_11
#define Dynamixel_Rx_GPIO_Port GPIOC
#define Chessboard_ENC_MOSI_Pin GPIO_PIN_7
#define Chessboard_ENC_MOSI_GPIO_Port GPIOD
#define Chessboard_ENC_MISO_Pin GPIO_PIN_9
#define Chessboard_ENC_MISO_GPIO_Port GPIOG
#define Chessboard_ENC_SCK_Pin GPIO_PIN_11
#define Chessboard_ENC_SCK_GPIO_Port GPIOG
#define PC_Rx_Pin GPIO_PIN_3
#define PC_Rx_GPIO_Port GPIOB
#define PC_TX_Pin GPIO_PIN_4
#define PC_TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
