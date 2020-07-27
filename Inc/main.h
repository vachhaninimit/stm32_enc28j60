/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define LC2_C_Pin GPIO_PIN_13
#define LC2_C_GPIO_Port GPIOC

#define LC2_D_Pin GPIO_PIN_14
#define LC2_D_GPIO_Port GPIOC

#define LC4_D_Pin GPIO_PIN_0
#define LC4_D_GPIO_Port GPIOC

#define LC4_C_Pin GPIO_PIN_1
#define LC4_C_GPIO_Port GPIOC

#define LC5_D_Pin GPIO_PIN_2
#define LC5_D_GPIO_Port GPIOC
#define LC5_C_Pin GPIO_PIN_3
#define LC5_C_GPIO_Port GPIOC
#define LC6_D_Pin GPIO_PIN_0
#define LC6_D_GPIO_Port GPIOA
#define LC6_C_Pin GPIO_PIN_1
#define LC6_C_GPIO_Port GPIOA
#define ENC_1A_Pin GPIO_PIN_5
#define ENC_1A_GPIO_Port GPIOC
#define ENC_1B_Pin GPIO_PIN_0
#define ENC_1B_GPIO_Port GPIOB
#define RLY_FWD_Pin GPIO_PIN_1
#define RLY_FWD_GPIO_Port GPIOB
#define RLY_REV_Pin GPIO_PIN_2
#define RLY_REV_GPIO_Port GPIOB
#define RLY_ENC_Pin GPIO_PIN_10
#define RLY_ENC_GPIO_Port GPIOB
#define RLY_FS_Pin GPIO_PIN_11
#define RLY_FS_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_MISO_Pin GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define SPI2_RST_Pin GPIO_PIN_6
#define SPI2_RST_GPIO_Port GPIOC
#define DAC_SYNC_Pin GPIO_PIN_15
#define DAC_SYNC_GPIO_Port GPIOA
#define DAC_C_Pin GPIO_PIN_10
#define DAC_C_GPIO_Port GPIOC
#define DAC_D_Pin GPIO_PIN_11
#define DAC_D_GPIO_Port GPIOC
#define LC3_C_Pin GPIO_PIN_12
#define LC3_C_GPIO_Port GPIOC
#define LC3_D_Pin GPIO_PIN_2
#define LC3_D_GPIO_Port GPIOD
#define LC1_D_Pin GPIO_PIN_3
#define LC1_D_GPIO_Port GPIOB
#define LC1_C_Pin GPIO_PIN_4
#define LC1_C_GPIO_Port GPIOB

#define USART1_EN_Pin GPIO_PIN_5
#define USART1_EN_GPIO_Port GPIOB

#define USART1_TX_Pin GPIO_PIN_6
#define USART1_TX_GPIO_Port GPIOB

#define USART1_RX_Pin GPIO_PIN_7
#define USART1_RX_GPIO_Port GPIOB



/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
