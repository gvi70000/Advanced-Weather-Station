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
#include "stm32f3xx_hal.h"

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
#define IO1_Pin GPIO_PIN_13
#define IO1_GPIO_Port GPIOC
#define RX0_ESP_Pin GPIO_PIN_2
#define RX0_ESP_GPIO_Port GPIOA
#define TX0_ESP_Pin GPIO_PIN_3
#define TX0_ESP_GPIO_Port GPIOA
#define SET_ESP_MSG_Pin GPIO_PIN_4
#define SET_ESP_MSG_GPIO_Port GPIOA
#define GET_ESP_MSG_Pin GPIO_PIN_5
#define GET_ESP_MSG_GPIO_Port GPIOA
#define RST_HDC3020_Pin GPIO_PIN_6
#define RST_HDC3020_GPIO_Port GPIOA
#define INT1_HDC3020_Pin GPIO_PIN_7
#define INT1_HDC3020_GPIO_Port GPIOA
#define INT_AS3935_Pin GPIO_PIN_0
#define INT_AS3935_GPIO_Port GPIOB
#define BMP_INT_Pin GPIO_PIN_1
#define BMP_INT_GPIO_Port GPIOB
#define INT2_HDC3020_Pin GPIO_PIN_2
#define INT2_HDC3020_GPIO_Port GPIOB
#define GPS_RX_Pin GPIO_PIN_10
#define GPS_RX_GPIO_Port GPIOB
#define GPS_TX_Pin GPIO_PIN_11
#define GPS_TX_GPIO_Port GPIOB
#define INT_ES160_Pin GPIO_PIN_12
#define INT_ES160_GPIO_Port GPIOB
#define SYN_Pin GPIO_PIN_13
#define SYN_GPIO_Port GPIOB
#define INT_AS7331_Pin GPIO_PIN_14
#define INT_AS7331_GPIO_Port GPIOB
#define INT_TSL25911_Pin GPIO_PIN_15
#define INT_TSL25911_GPIO_Port GPIOB
#define INT_TCS34717_Pin GPIO_PIN_6
#define INT_TCS34717_GPIO_Port GPIOC
#define GPS_RST_Pin GPIO_PIN_12
#define GPS_RST_GPIO_Port GPIOA
#define IO3_Pin GPIO_PIN_15
#define IO3_GPIO_Port GPIOA
#define UST3_RX_Pin GPIO_PIN_10
#define UST3_RX_GPIO_Port GPIOC
#define UST3_TX_Pin GPIO_PIN_11
#define UST3_TX_GPIO_Port GPIOC
#define UST2_RX_Pin GPIO_PIN_12
#define UST2_RX_GPIO_Port GPIOC
#define UST2_TX_Pin GPIO_PIN_2
#define UST2_TX_GPIO_Port GPIOD
#define IO2_Pin GPIO_PIN_5
#define IO2_GPIO_Port GPIOB
#define UST1_RX_Pin GPIO_PIN_6
#define UST1_RX_GPIO_Port GPIOB
#define UST1_TX_Pin GPIO_PIN_7
#define UST1_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
