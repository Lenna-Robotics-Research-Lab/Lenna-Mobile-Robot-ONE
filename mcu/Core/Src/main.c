/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "eth.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include "ultrasonic.h"
#include "utilities.h"
#include "motion.h"
#include "pid.h"
#include "math.h"
#include "odometry.h"
#include "mcu_config.h"
#include "packet_handler.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t msgBuffer[32] = " Lenna Robotics Research Lab. \r\n";

uint8_t min_len_packet = 8;
uint8_t max_len_packet = 32;
uint8_t protocol_data[144] = {0};
uint8_t txBuffer[144]; //= {0xFF, 0xFF, 0x05, 0xA0, 0xAB, 0xCD, 0xB4, 0xFC};

bool flag_uart_cb = 0;
bool flag_remain_packet = 1;

uint8_t total_pkt_length = 0;
uint8_t remain_pkt_length = 0;

unsigned short temp_crc = 0;


char MSG[128];


uint8_t input_speed ;// step given by MATLAB code
uint16_t left_enc_temp = 0, right_enc_temp = 0 , right_enc_diff = 0, left_enc_diff = 0;
int8_t dir_right,dir_left;
uint16_t encoder_tick[2] = {0};
float angular_speed_left,angular_speed_right;
uint16_t tst;

uint8_t flag_tx = 0, pid_tim_flag = 0, dir_flag = 0;

// ####################   ODOMETRY  ###################
const imu_cfgType imu =
{
	&hi2c3
};

const encoder_cfgType enc_right =
{
	&htim3,
	48960,
	0.1225		// 6000/48960
};

const encoder_cfgType enc_left =
{
	&htim2,
	48960,
	0.1225		// 6000/48960
};

odom_cfgType odom =
{
	imu,
	enc_right,
	enc_left
};



// ####################   Motor struct Value Setting   ###################
const motor_cfgType motor_right =
{
	MOTOR_PORT,
	MOTOR1_A_PIN,
	MOTOR_PORT,
	MOTOR1_B_PIN,
	&htim8,
	TIM_CHANNEL_1,
	1000,
	//1
};
const motor_cfgType motor_left =
{
	MOTOR_PORT,
	MOTOR2_A_PIN,
	MOTOR_PORT,
	MOTOR2_B_PIN,
	&htim8,
	TIM_CHANNEL_2,
	1000,
	//-1
};

// ####################   Ultra-Sonic struct Value Setting   ###################

const diffDrive_cfgType diff_robot =
{
	motor_right,
	motor_left,
	65,
	200
};

const ultrasonic_cfgType us_front =
{
	US1_TRIG_PORT,
	US1_TRIG_PIN,
	&htim4,
	TIM4,
	TIM_CHANNEL_3,
	168,
	1
};

// ####################   PID struct Value Setting   ###################

// definitions concerning PID gain values are made in the main.h header file
pid_cfgType pid_motor_left =
{
	Proportional_Gain_LEFT_MOTOR,
	Integral_Gain_LEFT_MOTOR,
	Derivative_Gain_LEFT_MOTOR,
	Sampling_Time,
	Lower_Saturation_Limit,
	Upper_Saturation_Limit,
	0,
	0,
	0,
	0,
	0,
	1,
	0,
	0
};

pid_cfgType pid_motor_right =
{
	Proportional_Gain_RIGHT_MOTOR,
	Integral_Gain_RIGHT_MOTOR,
	Derivative_Gain_RIGHT_MOTOR,
	Sampling_Time,
	Lower_Saturation_Limit,
	Upper_Saturation_Limit,
	0,
	0,
	0,
	0,
	0,
	1,
	0,
	0
};

// ####################   Packet struct Value Setting   ###################

packet_cfgType rx_packet=
{
	&huart2,
	3,
	144,
};


int16_t val_x;
int16_t val_y;
int16_t val_z;
float val_heading;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ####################   UART Tx -> printf   ####################
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ETH_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
//  HAL_TIM_Base_Init(&htim5);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_I2C_Init(&hi2c3);

//  printf("Lenna Robotics Research Lab. \r\n");
//  HAL_Delay(1000);

// #################### Initializations   ####################

  LRL_PID_Init(&pid_motor_left,  1);
  LRL_PID_Init(&pid_motor_right, 1);

  LRL_Encoder_Init(&odom);

  LRL_MPU6050_Init(&odom);

  LRL_HMC5883L_Init(&odom);

  HAL_UART_Transmit(&huart1, msgBuffer, 32, 100);

//  HAL_Delay(1000);
  LRL_handShake(&rx_packet);

//  LRL_RX_Init(&rx_packet);
  LRL_Packet_Init(&rx_packet);

//  HAL_UART_Receive_IT(&huart2, rxBuffer, min_len_packet);

  txBuffer[0] = 0xFF;
  txBuffer[1] = 0xFF;

  // ####################   memory allocation    ####################
int16_t motor_speed_left, motor_speed_right;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  LRL_rxPacket(&rx_packet);
//	  if(rx_packet.rx_dataValid)
//	  {

	    motor_speed_left = rx_packet.data.left_velocity;
	    motor_speed_right = rx_packet.data.right_velocity;

//	  }
	  if(pid_tim_flag == 1)
	  {
//		LRL_MPU6050_ReadAll(&odom);
//		LRL_MPU6050_ComplementaryFilter(&odom);
//		LRL_Encoder_ReadAngularSpeed(&odom);
//		sprintf(MSG,"readings are : %d\t %d\t\n\r",odom.vel.left,odom.vel.right);
//		HAL_UART_Transmit(&huart1, &MSG, sizeof(MSG), 10);
		LRL_Encoder_ReadAngularSpeed(&odom);

		sprintf(MSG,"data is : %d\t %d\t %d\t %d\t \r\n",motor_speed_left, motor_speed_right, odom.vel.left,odom.vel.right);
		HAL_UART_Transmit_IT(&huart1, MSG, sizeof(MSG));

		LRL_PID_Update(&pid_motor_left, odom.vel.left, motor_speed_left);
		LRL_PID_Update(&pid_motor_right, odom.vel.right,motor_speed_right);
		LRL_Motion_Control(diff_robot, pid_motor_left.Control_Signal,pid_motor_right.Control_Signal);

		pid_tim_flag = 0;
	  }
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// CRC

// ####################   Ultra Sonic Callback   ####################

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//	// TIMER Input Capture Callback
//	LRL_US_TMR_IC_ISR(htim, us_front);
//}

// ####################   UART Receive Callback   ####################

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	if (huart == &huart2)
//	{
		HAL_GPIO_WritePin(BLINK_LED_PORT, BLINK_LED_PIN, GPIO_PIN_SET);
		rx_packet.rx_byteReady = 1;
//	}
//	else
//	{
//		HAL_UART_Receive_IT(&huart1,&input_speed, 1);
//		flag_tx = 1;
//	}

}

// ####################   Timer To Creat 0.01 Delay Callback   ####################

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if(htim == &htim5)
	{
		pid_tim_flag = 1;
	}

}

// ####################   I2C Callback   ####################

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
if(hi2c == &hi2c3)
	{
////		LRL_IMU_Read(&imu);
//		HAL_GPIO_WritePin(BLINK_LED_PORT, BLINK_LED_PIN, 1);
//	LRL_GYRO_Read(&imu);
	}
}
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//	//
//}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
//{
//	// TIMER Overflow Callback
//	LRL_US_TMR_OVF_ISR(htim, us_front);
//}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
