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
#include "stm32f4xx_hal.h"

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
#define TRIG_HC_3_Pin GPIO_PIN_3
#define TRIG_HC_3_GPIO_Port GPIOE
#define TRIG_HC_4_Pin GPIO_PIN_4
#define TRIG_HC_4_GPIO_Port GPIOE
#define ECHO_HC_3_Pin GPIO_PIN_5
#define ECHO_HC_3_GPIO_Port GPIOE
#define ECHO_HC_4_Pin GPIO_PIN_6
#define ECHO_HC_4_GPIO_Port GPIOE
#define JET_SPI_MISO_Pin GPIO_PIN_2
#define JET_SPI_MISO_GPIO_Port GPIOC
#define JET_SPI_MOSI_Pin GPIO_PIN_3
#define JET_SPI_MOSI_GPIO_Port GPIOC
#define Buzzer_Pin GPIO_PIN_4
#define Buzzer_GPIO_Port GPIOA
#define Battery_ADC_Pin GPIO_PIN_5
#define Battery_ADC_GPIO_Port GPIOA
#define JET_SPI_SCK_Pin GPIO_PIN_10
#define JET_SPI_SCK_GPIO_Port GPIOB
#define MOTOR1_PWM_Pin GPIO_PIN_6
#define MOTOR1_PWM_GPIO_Port GPIOC
#define MOTOR2_PWM_Pin GPIO_PIN_7
#define MOTOR2_PWM_GPIO_Port GPIOC
#define BUS_SDA_Pin GPIO_PIN_9
#define BUS_SDA_GPIO_Port GPIOC
#define BUS_SCL_Pin GPIO_PIN_8
#define BUS_SCL_GPIO_Port GPIOA
#define USB2Serial_TX_Pin GPIO_PIN_9
#define USB2Serial_TX_GPIO_Port GPIOA
#define USB2Serial_RX_Pin GPIO_PIN_10
#define USB2Serial_RX_GPIO_Port GPIOA
#define ENCODER2_A_Pin GPIO_PIN_15
#define ENCODER2_A_GPIO_Port GPIOA
#define MOTOR2_B_Pin GPIO_PIN_0
#define MOTOR2_B_GPIO_Port GPIOD
#define MOTOR2_A_Pin GPIO_PIN_1
#define MOTOR2_A_GPIO_Port GPIOD
#define MOTOR1_B_Pin GPIO_PIN_3
#define MOTOR1_B_GPIO_Port GPIOD
#define MOTOR1_A_Pin GPIO_PIN_4
#define MOTOR1_A_GPIO_Port GPIOD
#define ENCODER2_B_Pin GPIO_PIN_3
#define ENCODER2_B_GPIO_Port GPIOB
#define ENCODER1_A_Pin GPIO_PIN_4
#define ENCODER1_A_GPIO_Port GPIOB
#define ENCODER1_B_Pin GPIO_PIN_5
#define ENCODER1_B_GPIO_Port GPIOB
#define JET_I2C_SCL_Pin GPIO_PIN_6
#define JET_I2C_SCL_GPIO_Port GPIOB
#define JET_I2C_SDA_Pin GPIO_PIN_7
#define JET_I2C_SDA_GPIO_Port GPIOB
#define ECHO_HC_1_Pin GPIO_PIN_8
#define ECHO_HC_1_GPIO_Port GPIOB
#define ECHO_HC_2_Pin GPIO_PIN_9
#define ECHO_HC_2_GPIO_Port GPIOB
#define TRIG_HC_1_Pin GPIO_PIN_0
#define TRIG_HC_1_GPIO_Port GPIOE
#define TRIG_HC_2_Pin GPIO_PIN_1
#define TRIG_HC_2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */


//=========================================== Conversions ========================================

// imu values

#define rad2deg 57.2958

// pid values

#define Speed2PWM_Rate 75/218
#define Tick2RMP_Rate 6000/48960

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
