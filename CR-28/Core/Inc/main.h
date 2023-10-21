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
#define LSE_In_Pin GPIO_PIN_14
#define LSE_In_GPIO_Port GPIOC
#define LSE_Out_Pin GPIO_PIN_15
#define LSE_Out_GPIO_Port GPIOC
#define CAN3_RX_Pin GPIO_PIN_6
#define CAN3_RX_GPIO_Port GPIOF
#define CAN3_TX_Pin GPIO_PIN_7
#define CAN3_TX_GPIO_Port GPIOF
#define HSE_In_Pin GPIO_PIN_0
#define HSE_In_GPIO_Port GPIOH
#define HSE_Out_Pin GPIO_PIN_1
#define HSE_Out_GPIO_Port GPIOH
#define ADC_10B_Pin GPIO_PIN_0
#define ADC_10B_GPIO_Port GPIOC
#define ADC_11B_Pin GPIO_PIN_1
#define ADC_11B_GPIO_Port GPIOC
#define ADC_16B_Pin GPIO_PIN_0
#define ADC_16B_GPIO_Port GPIOA
#define ADC_17B_Pin GPIO_PIN_1
#define ADC_17B_GPIO_Port GPIOA
#define ADC_14B_Pin GPIO_PIN_2
#define ADC_14B_GPIO_Port GPIOA
#define ADC_15B_Pin GPIO_PIN_3
#define ADC_15B_GPIO_Port GPIOA
#define ADC_18B_Pin GPIO_PIN_4
#define ADC_18B_GPIO_Port GPIOA
#define ADC_19B_Pin GPIO_PIN_5
#define ADC_19B_GPIO_Port GPIOA
#define ADC_3B_Pin GPIO_PIN_6
#define ADC_3B_GPIO_Port GPIOA
#define ADC_7B_Pin GPIO_PIN_7
#define ADC_7B_GPIO_Port GPIOA
#define ADC_4B_Pin GPIO_PIN_4
#define ADC_4B_GPIO_Port GPIOC
#define ADC_8B_Pin GPIO_PIN_5
#define ADC_8B_GPIO_Port GPIOC
#define ADC_9B_Pin GPIO_PIN_0
#define ADC_9B_GPIO_Port GPIOB
#define ADC_5B_Pin GPIO_PIN_1
#define ADC_5B_GPIO_Port GPIOB
#define ADC_2B_Pin GPIO_PIN_11
#define ADC_2B_GPIO_Port GPIOF
#define GPIO_PE10_Pin GPIO_PIN_10
#define GPIO_PE10_GPIO_Port GPIOE
#define GPIO_PE11_Pin GPIO_PIN_11
#define GPIO_PE11_GPIO_Port GPIOE
#define GPIO_PE12_Pin GPIO_PIN_12
#define GPIO_PE12_GPIO_Port GPIOE
#define GPIO_PE13_Pin GPIO_PIN_13
#define GPIO_PE13_GPIO_Port GPIOE
#define GPIO_PE14_Pin GPIO_PIN_14
#define GPIO_PE14_GPIO_Port GPIOE
#define CAN2_RX_Pin GPIO_PIN_12
#define CAN2_RX_GPIO_Port GPIOB
#define CAN2_TX_Pin GPIO_PIN_13
#define CAN2_TX_GPIO_Port GPIOB
#define PA_11_Pin GPIO_PIN_11
#define PA_11_GPIO_Port GPIOA
#define PA_12_Pin GPIO_PIN_12
#define PA_12_GPIO_Port GPIOA
#define NRESET_LoRa_Pin GPIO_PIN_12
#define NRESET_LoRa_GPIO_Port GPIOC
#define GPIO_LED_Pin GPIO_PIN_10
#define GPIO_LED_GPIO_Port GPIOG
#define I2C_SCL_Pin GPIO_PIN_6
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_7
#define I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
