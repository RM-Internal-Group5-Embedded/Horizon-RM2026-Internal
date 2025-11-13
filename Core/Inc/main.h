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
#include "stm32g4xx_hal.h"

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
#define sensor_1_Pin GPIO_PIN_13
#define sensor_1_GPIO_Port GPIOC
#define sensor_2_Pin GPIO_PIN_14
#define sensor_2_GPIO_Port GPIOC
#define BTN_1_Pin GPIO_PIN_15
#define BTN_1_GPIO_Port GPIOC
#define jc24b_pd_Pin GPIO_PIN_0
#define jc24b_pd_GPIO_Port GPIOA
#define jc24b_set_Pin GPIO_PIN_1
#define jc24b_set_GPIO_Port GPIOA
#define MPU6500_CS_Pin GPIO_PIN_4
#define MPU6500_CS_GPIO_Port GPIOA
#define MPU6500_SCLK_Pin GPIO_PIN_5
#define MPU6500_SCLK_GPIO_Port GPIOA
#define MPU6500_MISO_Pin GPIO_PIN_6
#define MPU6500_MISO_GPIO_Port GPIOA
#define MPU6500_MOSI_Pin GPIO_PIN_7
#define MPU6500_MOSI_GPIO_Port GPIOA
#define sensor_3_Pin GPIO_PIN_1
#define sensor_3_GPIO_Port GPIOB
#define sensor_4_Pin GPIO_PIN_2
#define sensor_4_GPIO_Port GPIOB
#define jga_left_backward_Pin GPIO_PIN_10
#define jga_left_backward_GPIO_Port GPIOB
#define jga_right_backward_Pin GPIO_PIN_11
#define jga_right_backward_GPIO_Port GPIOB
#define jga_left_forward_Pin GPIO_PIN_12
#define jga_left_forward_GPIO_Port GPIOB
#define jga_right_forward_Pin GPIO_PIN_13
#define jga_right_forward_GPIO_Port GPIOB
#define jga_left_speed_Pin GPIO_PIN_14
#define jga_left_speed_GPIO_Port GPIOB
#define jga_right_speed_Pin GPIO_PIN_15
#define jga_right_speed_GPIO_Port GPIOB
#define servo_0_Pin GPIO_PIN_15
#define servo_0_GPIO_Port GPIOA
#define servo_1_Pin GPIO_PIN_3
#define servo_1_GPIO_Port GPIOB
#define jga_right_encoder_A_Pin GPIO_PIN_4
#define jga_right_encoder_A_GPIO_Port GPIOB
#define jga_right_encoder_B_Pin GPIO_PIN_5
#define jga_right_encoder_B_GPIO_Port GPIOB
#define jga_left_encoder_A_Pin GPIO_PIN_6
#define jga_left_encoder_A_GPIO_Port GPIOB
#define jga_left_encoder_B_Pin GPIO_PIN_7
#define jga_left_encoder_B_GPIO_Port GPIOB
#define ws2812_Pin GPIO_PIN_9
#define ws2812_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
