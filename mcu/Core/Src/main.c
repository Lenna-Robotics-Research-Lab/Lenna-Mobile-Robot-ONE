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
#include "mcu_layout.h"
#include "ultrasonic.h"
#include "utilities.h"
#include "motion.h"
#include "pid.h"
#include "imu.h"
#include "hmc5883l.h"
//#include "mpu6050.h"


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
char MSG[128];

uint8_t input_speed ;// step given by MATLAB code
uint16_t left_enc_temp = 0, right_enc_temp = 0 , right_enc_diff = 0, left_enc_diff = 0;
uint16_t encoder_tick[2] = {0};
float angular_speed_left,angular_speed_right;
uint16_t tst;

uint8_t flag_tx = 0, pid_tim_flag = 0, dir_flag = 0;



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

// ####################   IMU struct Value Setting   ###################

imu_cfgType gy80=
{
	&hi2c3,
	0,
	0,
	0
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
  LRL_Delay_Init();			// TIMER Initialization for Delay us
  LRL_US_Init(us_front); 	// TIMER Initialization for Ultrasonics

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
//  HAL_TIM_Base_Init(&htim5);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_I2C_Init(&hi2c3);

  //printf("Lenna Robotics Research Lab. \r\n");
  // HAL_Delay(1000);

// ####################   Encoder Initialization   ####################
  TIM2->CNT = 0;
  TIM3->CNT = 0;
  encoder_tick[0] = (TIM2->CNT);
  encoder_tick[1] = (TIM3->CNT);

// ####################   MATLAB Communication Initialization   ####################

 /*
  // Initializing the MATLAB communication and identification
  // Receiving info from MATLAB SIMULINK for system identification and commands to run speed
  HAL_UART_Receive_IT(&huart1,&input_speed, 1); // getting the speed

 */

  LRL_PID_Init(&pid_motor_left,  1);
  LRL_PID_Init(&pid_motor_right, 1);
  LRL_MPU_Init(&gy80);

  LRL_MPU_Bypass(&gy80);

  HMC5883L_init(&hi2c3);
//uint8_t ident = 0;
  //uint8_t tstt[3];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//		HAL_I2C_Master_Transmit(&hi2c3, HMC5883L_ADDRESS >> 1, 0x3D, 1, DELAY_TIMEOUT);
//		HAL_I2C_Mem_Read(&hi2c3, HMC5883L_ADDRESS >> 1, 10, 1, &ident, 1, DELAY_TIMEOUT);
//	  HAL_I2C_Mem_Read(&hi2c3, 0x3D, 10, 1, &val_x, 1, 100);
//	  sprintf(MSG,"ident: %d\n\r", ident);

//	  HAL_Delay(10);

// ####################   imu setup  ####################
//	  LRL_ACCEL_Read(&gy80);
//	  uint8_t data[6];
//	  int16_t mytst[3];

//	  MPU6050_Read_All(&hi2c3, &gy80);
//	  LRL_Read_Gyro(&gy80);
//	  LRL_Read_Accel(&gy80);
//
//	  static uint32_t prev_time = 0;
//	  uint32_t curr_time = HAL_GetTick();
//	  float dt = (curr_time - prev_time) / 1000.0f;
//	  prev_time = curr_time;
//
//	  LRL_Complementary_Filter(&gy80);
//	  HAL_I2C_Mem_Read(&hi2c3, 0xD0, 0x75, 1, &data[0], 1,10);
//	  HAL_I2C_Mem_Read(&hi2c3, GYRO_ADDR_R, 0x29, 1, &data[1], 1,10);
//	  mytst[0] = ((data[1]<<8)|data[0]);
//	  mytst[1] = ((data[3]<<8)|data[2]);
//	  mytst[2] = ((data[5]<<8)|data[4]);


//	  HAL_I2C_Mem_Read(&hi2c3,0xD3,0x0F,1,&myimu,1,100);
//	  LRL_GY80_Init(&hi2c3,tstt);
	  HMC5883L_readHeading(&val_x, &val_y, &val_z, &val_heading);

//      sprintf(MSG,"the speed is : %3.2f\t %3.2f\t %3.2f\n\r", gy80.roll, gy80.pitch, gy80.yaw);
	  sprintf(MSG,"magnetometer heading: %4.2f\n\r", val_heading);
//	  sprintf(MSG,"the speed is : %d\n\r", data[0]);
	  HAL_UART_Transmit(&huart1,MSG, 64,100);
//	  HAL_Delay(1);

// ####################   Motor Test Scenarios   ####################
//	  LRL_Motion_Control(diff_robot, -100, 100);
//	  if(input_speed >= 50 && input_speed <= 100)
//	  {
//		  LRL_Motor_Speed(motor_left, -1*input_speed);
//	  }
//	  else
//	  {
//		  HAL_GPIO_TogglePin(BLINK_LED_PORT, BLINK_LED_PIN);
//		  HAL_Delay(100);
//		  LRL_Motor_Speed(motor_left, 0);
//	  }

//	  LRL_Motor_Speed(motor_right, input_speed);

// ####################   Encoder Reading   ####################
/*
	  if(pid_tim_flag == 1)
	  {
		  encoder_tick[0] = (TIM2->CNT); // Left Motor Encoder
		  encoder_tick[1] = (TIM3->CNT); // Right Motor Encoder

		  // Reading the Encoder for the right Motor
//		  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
//		  {
//			  if(encoder_tick[1] - right_enc_temp >= 0)
//			  {
////				  right_enc_diff = encoder_tick[1] - right_enc_temp;
//				  right_enc_diff = (48960 + encoder_tick[1]) - right_enc_temp;
//			  }
//			  else
//			  {
////				  right_enc_diff = (48960 - right_enc_temp) + encoder_tick[1];
//				  right_enc_diff = -(encoder_tick[1] - right_enc_temp);
//			  }
//			  right_enc_temp = encoder_tick[1];

//		  }
////		  else
//		  {

		  ;
		  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3) == 0)
		  {
			  if(encoder_tick[1] - right_enc_temp >= 0)
			  {
				  right_enc_diff = encoder_tick[1] - right_enc_temp;
			  }
			  else
			  {
				  right_enc_diff = (48960 - right_enc_temp) + encoder_tick[1];
			  }
			  right_enc_temp = encoder_tick[1];
		  }
		  else
		  {
			  if(right_enc_temp - encoder_tick[1] >= 0)
			  {
				  right_enc_diff = -(encoder_tick[1] - right_enc_temp);
			  }
			  else
			  {
				  right_enc_diff = (48960 - encoder_tick[1]) + right_enc_temp;
			  }
			  right_enc_temp = encoder_tick[1];
		  }
//		  }


			  // Reading the Encoder for the left Motor

		  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) == 0)
		  {
			  if(encoder_tick[0] - left_enc_temp >= 0)
			  {
				  left_enc_diff = encoder_tick[0] - left_enc_temp;
			  }
			  else
			  {
				  left_enc_diff = (48960 - left_enc_temp) + encoder_tick[0];
			  }
			  left_enc_temp = encoder_tick[0];
		  }
		  else
		  {
			  if(left_enc_temp - encoder_tick[0] >= 0)
			  {
				  left_enc_diff = -(encoder_tick[0] - left_enc_temp);
			  }
			  else
			  {
				  left_enc_diff = (48960 - encoder_tick[0]) + left_enc_temp;
			  }
			  left_enc_temp = encoder_tick[0];
		  }

*/
// ####################   PID control   ####################

/*


// The transmission of the encoder tick to angular velocity is (6000 / 48960)

		  angular_speed_left = left_enc_diff * Tick2RMP_Rate ;//  *Speed2PWM_Rate;
		  angular_speed_right = right_enc_diff * Tick2RMP_Rate // * Speed2PWM_Rate;

		  LRL_PID_Update(&pid_motor_left,angular_speed_left,120);
//		  LRL_Motor_Speed(motor_left, pid_motor_left.Control_Signal);

		  LRL_PID_Update(&pid_motor_right,angular_speed_right,120);
//		  LRL_Motor_Speed(motor_right, pid_motor_right.Control_Signal);
		  pid_tim_flag = 0;
//		  HAL_GPIO_WritePin(BLINK_LED_PORT, BLINK_LED_PIN, 1);
		  LRL_Motor_Speed(motor_left, pid_motor_left.Control_Signal);
		  LRL_Motor_Speed(motor_right, pid_motor_right.Control_Signal);
		  sprintf(MSG,"the speed is : %5.1f\t%d\t%5.1f\t%5.1f\r\n ",angular_speed_left,pid_motor_left.Control_Signal,pid_motor_left.Prev_Error,pid_motor_left.Integrator_Amount);
		  HAL_UART_Transmit_IT(&huart1,MSG, 64);
	  }
*/

// ####################   Transmit Speed for MATLAB Identification   ####################
	  /*
	  // Sending the speed that has been read from encoder only if there is a receiving data
	  if(flag_tx == 1){
		  HAL_UART_Transmit(&huart1,(uint8_t *)&right_enc_diff, sizeof(right_enc_diff),10);
		  flag_tx = 0;
	  }

	  // Pay great attention to the delayed time hence the wrong data can be received in MATLAB
	  // Since the Data is read roughly every 0.01 second this amount of delay is necessary for correct transmission
	  HAL_Delay(10);

	  */



//	  sprintf(MSG, "encoder ticks: %04d\t%04d\r\n", encoder_tick[0], encoder_tick[1]);
//	  printf(MSG);


//	LRL_US_Trig(us_front);	// Trigging the Sensor for 15us
//
//	sprintf(MSG, "Distance:\t %05.1f \r\n", LRL_US_Read(us_front));
//	printf(MSG);
//
	  //HAL_Delay(1000);


//	  	sprintf(MSG, "Distance:\t %d \r\n", i);
//	  	printf(i);

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

// ####################   Ultra Sonic Callback   ####################

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	// TIMER Input Capture Callback
	LRL_US_TMR_IC_ISR(htim, us_front);
}

// ####################   UART Receive Callback   ####################

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Receive_IT(&huart1,&input_speed, 1);
	flag_tx = 1;
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
////		LRL_IMU_Read(&gy80);
		HAL_GPIO_WritePin(BLINK_LED_PORT, BLINK_LED_PIN, 1);
//	LRL_GYRO_Read(&gy80);
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
