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
#include "stm32f7xx_hal.h"

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
#define TFT_BL_Pin GPIO_PIN_3
#define TFT_BL_GPIO_Port GPIOE
#define TFT_CS_Pin GPIO_PIN_4
#define TFT_CS_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define MOTOR5_B_Pin GPIO_PIN_4
#define MOTOR5_B_GPIO_Port GPIOF
#define MOTOR5_A_Pin GPIO_PIN_5
#define MOTOR5_A_GPIO_Port GPIOF
#define TFT_DC_Pin GPIO_PIN_7
#define TFT_DC_GPIO_Port GPIOF
#define TFT_RST_Pin GPIO_PIN_8
#define TFT_RST_GPIO_Port GPIOF
#define MOTOR6_B_Pin GPIO_PIN_10
#define MOTOR6_B_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define Switch_UP2_Pin GPIO_PIN_0
#define Switch_UP2_GPIO_Port GPIOC
#define Switch_BALL1_Pin GPIO_PIN_1
#define Switch_BALL1_GPIO_Port GPIOC
#define TIM2_CH1_ENCODER2_A_Pin GPIO_PIN_0
#define TIM2_CH1_ENCODER2_A_GPIO_Port GPIOA
#define TIM2_CH2_ENCODER2_B_Pin GPIO_PIN_1
#define TIM2_CH2_ENCODER2_B_GPIO_Port GPIOA
#define TIM5_CH3_BRUSHLESS1_Pin GPIO_PIN_2
#define TIM5_CH3_BRUSHLESS1_GPIO_Port GPIOA
#define TIM5_CH4_BRUSHLESS2_Pin GPIO_PIN_3
#define TIM5_CH4_BRUSHLESS2_GPIO_Port GPIOA
#define Switch_BALL2_Pin GPIO_PIN_4
#define Switch_BALL2_GPIO_Port GPIOA
#define TIM3_CH1_MOTOR4_PWM_Pin GPIO_PIN_6
#define TIM3_CH1_MOTOR4_PWM_GPIO_Port GPIOA
#define TIM3_CH2_MOTOR3_PWM_Pin GPIO_PIN_7
#define TIM3_CH2_MOTOR3_PWM_GPIO_Port GPIOA
#define MOTOR4_B_Pin GPIO_PIN_4
#define MOTOR4_B_GPIO_Port GPIOC
#define GREEN_USER_LED_Pin GPIO_PIN_0
#define GREEN_USER_LED_GPIO_Port GPIOB
#define MOTOR3_B_Pin GPIO_PIN_1
#define MOTOR3_B_GPIO_Port GPIOB
#define MOTOR3_A_Pin GPIO_PIN_2
#define MOTOR3_A_GPIO_Port GPIOB
#define RELAY_4_Pin GPIO_PIN_14
#define RELAY_4_GPIO_Port GPIOF
#define RELAY_1_Pin GPIO_PIN_7
#define RELAY_1_GPIO_Port GPIOE
#define MOTOR6_A_Pin GPIO_PIN_8
#define MOTOR6_A_GPIO_Port GPIOE
#define RELAY_5_Pin GPIO_PIN_9
#define RELAY_5_GPIO_Port GPIOE
#define RELAY_6_Pin GPIO_PIN_11
#define RELAY_6_GPIO_Port GPIOE
#define MOTOR2_B_Pin GPIO_PIN_11
#define MOTOR2_B_GPIO_Port GPIOB
#define MOTOR2_A_Pin GPIO_PIN_12
#define MOTOR2_A_GPIO_Port GPIOB
#define MOTOR4_A_Pin GPIO_PIN_13
#define MOTOR4_A_GPIO_Port GPIOB
#define TIM12_CH1_MOTOR6_PWM_Pin GPIO_PIN_14
#define TIM12_CH1_MOTOR6_PWM_GPIO_Port GPIOB
#define TIM12_CH2_MOTOR5_PWM_Pin GPIO_PIN_15
#define TIM12_CH2_MOTOR5_PWM_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define TIM4_CH1_ENCODER4_A_Pin GPIO_PIN_12
#define TIM4_CH1_ENCODER4_A_GPIO_Port GPIOD
#define TIM4_CH2_ENCODER4_B_Pin GPIO_PIN_13
#define TIM4_CH2_ENCODER4_B_GPIO_Port GPIOD
#define RELAY_2_Pin GPIO_PIN_14
#define RELAY_2_GPIO_Port GPIOD
#define RELAY_3_Pin GPIO_PIN_15
#define RELAY_3_GPIO_Port GPIOD
#define Switch_UP1_Pin GPIO_PIN_2
#define Switch_UP1_GPIO_Port GPIOG
#define TIM8_CH1__ENCODER3_A_Pin GPIO_PIN_6
#define TIM8_CH1__ENCODER3_A_GPIO_Port GPIOC
#define TIM8_CH2__ENCODER3_B_Pin GPIO_PIN_7
#define TIM8_CH2__ENCODER3_B_GPIO_Port GPIOC
#define TIM3_CH3_MOTOR2_PWM_Pin GPIO_PIN_8
#define TIM3_CH3_MOTOR2_PWM_GPIO_Port GPIOC
#define TIM3_CH4_MOTOR1_PWM_Pin GPIO_PIN_9
#define TIM3_CH4_MOTOR1_PWM_GPIO_Port GPIOC
#define TIM1_CH1_ENCODER1_A_Pin GPIO_PIN_8
#define TIM1_CH1_ENCODER1_A_GPIO_Port GPIOA
#define TIM1_CH2_ENCODER1_B_Pin GPIO_PIN_9
#define TIM1_CH2_ENCODER1_B_GPIO_Port GPIOA
#define SPI_CHIPSELECT_Pin GPIO_PIN_10
#define SPI_CHIPSELECT_GPIO_Port GPIOA
#define MOTOR1_B_Pin GPIO_PIN_11
#define MOTOR1_B_GPIO_Port GPIOA
#define MOTOR1_A_Pin GPIO_PIN_12
#define MOTOR1_A_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Switch_DOWN2_Pin GPIO_PIN_11
#define Switch_DOWN2_GPIO_Port GPIOC
#define Switch_DOWN1_Pin GPIO_PIN_2
#define Switch_DOWN1_GPIO_Port GPIOD
#define SW0_Pin GPIO_PIN_3
#define SW0_GPIO_Port GPIOB
#define blue_user_led_Pin GPIO_PIN_7
#define blue_user_led_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
