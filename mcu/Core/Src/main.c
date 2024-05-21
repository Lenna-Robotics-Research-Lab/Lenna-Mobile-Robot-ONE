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
uint8_t msgBuffer[32] = " Lenna Robotics Research Lab. \r\n";

uint8_t min_len_packet = 8;
uint8_t max_len_packet = 32;
uint8_t rxBuffer[144] = {0};
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

imu_cfgType imu=
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
unsigned short updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
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

//  printf("Lenna Robotics Research Lab. \r\n");
//  HAL_Delay(1000);

// ####################   Encoder Initialization   ####################
  TIM2->CNT = 0;
  TIM3->CNT = 0;
  encoder_tick[0] = (TIM2->CNT);
  encoder_tick[1] = (TIM3->CNT);

  LRL_PID_Init(&pid_motor_left,  1);
  LRL_PID_Init(&pid_motor_right, 1);
  LRL_MPU_Bypass_Enable(&imu, 0);
  LRL_MPU_Init(&imu);

  LRL_MPU_Bypass_Enable(&imu,1);
  LRL_HMC5883L_Init(&hi2c3);


  HAL_UART_Transmit(&huart1, msgBuffer, 32, 100);
  HAL_Delay(1000);

  HAL_UART_Receive_IT(&huart2, rxBuffer, min_len_packet);

  txBuffer[0] = 0xFF;
  txBuffer[1] = 0xFF;
  // ####################   memory allocation    ####################
  float motor_speed_right = 0;
  float motor_speed_left = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(flag_uart_cb)
		{
			flag_uart_cb = 0;

			total_pkt_length = rxBuffer[2] + 3;
			remain_pkt_length = total_pkt_length - min_len_packet;

			if(remain_pkt_length)
			{
				HAL_UART_Receive(&huart2, &rxBuffer[8], remain_pkt_length, 1);
			}

			temp_crc = updateCRC(0, &rxBuffer, total_pkt_length-2);

			if (rxBuffer[3] == 0x01)
			{
				motor_speed_left = (float)((rxBuffer[4] << 8) | rxBuffer[5]);
				motor_speed_right = (float)((rxBuffer[6] << 8) | rxBuffer[7]);
			}


//			HAL_UART_Transmit(&huart1, rxBuffer, total_pkt_length, 0xFF);

//			HAL_UART_Transmit_IT(&huart2, txBuffer, 8);

			memset(rxBuffer, 0, max_len_packet*sizeof(rxBuffer[0]));
			HAL_UART_Receive_IT(&huart2, rxBuffer, min_len_packet);
		}


		if(pid_tim_flag == 1)
		{
		  encoder_tick[0] = (TIM2->CNT); 	// Left Motor Encoder
		  encoder_tick[1] = (TIM3->CNT); 	// Right Motor Encoder

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


		// PID
		  angular_speed_left = left_enc_diff * Tick2RMP_Rate ;
		  angular_speed_right = right_enc_diff * Tick2RMP_Rate;

		  LRL_PID_Update(&pid_motor_left, angular_speed_left, motor_speed_left);
		  LRL_PID_Update(&pid_motor_right, angular_speed_right, motor_speed_right);

		  LRL_Motion_Control(diff_robot, pid_motor_left.Control_Signal, pid_motor_right.Control_Signal);
//		  motor_speed_right = (float*)realloc(motor_speed_right,sizeof(float));
//		  motor_speed_left = (float*)realloc(motor_speed_left,sizeof(float));

		  pid_tim_flag = 0;

		  LRL_MPU_Bypass_Enable(&imu , 0);

		  LRL_MPU_Read_All(&imu);

		  LRL_MPU_Bypass_Enable(&imu , 1);
		  LRL_HMC5883L_ReadHeading(&val_x, &val_y, &val_z, &val_heading);

		  uint16_t tmp_heading = (uint16_t)val_heading;
		  uint16_t tmp_angular_left = (uint16_t) angular_speed_left;
		  uint16_t tmp_angular_right = (uint16_t) angular_speed_right;
		  uint16_t tmp_acc_x = (uint16_t)(imu.final_accel_x * 10000);
		  uint16_t tmp_acc_y = (uint16_t)(imu.final_accel_y * 10000);
		  int16_t tmp_gyr_x = (int16_t)(imu.final_gyro_x * 100);
		  uint16_t tmp_CRC;

		  txBuffer[2] = (uint8_t)(tmp_heading >> 8);
		  txBuffer[3] = (uint8_t)(tmp_heading & 0x00FF);

		  txBuffer[4] = (uint8_t)(tmp_angular_left >> 8);
		  txBuffer[5] = (uint8_t)(tmp_angular_left & 0x00FF);

		  txBuffer[6] = (uint8_t)(tmp_angular_right >> 8);
		  txBuffer[7] = (uint8_t)(tmp_angular_right & 0x00FF);

		  txBuffer[8] = (uint8_t)(tmp_acc_x>> 8);
		  txBuffer[9] = (uint8_t)(tmp_acc_x & 0x00FF);

		  txBuffer[10] = (uint8_t)(tmp_acc_y>> 8);
		  txBuffer[11] = (uint8_t)(tmp_acc_y & 0x00FF);

		  txBuffer[12] = (uint8_t)(tmp_gyr_x>> 8);
		  txBuffer[13] = (uint8_t)(tmp_gyr_x & 0x00FF);

		  tmp_CRC = updateCRC(0, &txBuffer, 14);

		  txBuffer[14] = (uint8_t)(tmp_CRC >> 8);
		  txBuffer[15] = (uint8_t)(tmp_CRC & 0x00FF);

		  HAL_UART_Transmit_IT(&huart2, txBuffer, 16);

		  sprintf(MSG,"something is : %d\t %d\t %d\t %d\t %d\t %d\t\n\r", tmp_heading, tmp_angular_left,tmp_angular_right,tmp_acc_x,tmp_acc_y,tmp_gyr_x);

		  HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 5);

//		sprintf(MSG, "speed L:%6.2f \t R:%6.2f \r\n", angular_speed_left, angular_speed_right);
//		HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 0xFF);
	}
//		sprintf(MSG, "speed L:%5.1f \t R:%5.1f \r\n", angular_speed_left, angular_speed_right);

//		HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 0xFF);
	//  LRL_MPU_Bypass_Enable(&imu);


//	  LRL_HMC5883L_ReadHeading(&val_x, &val_y, &val_z, &val_heading);
//	  sprintf(MSG,"heading is : %05.2f\n\r",val_heading);
//	  HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 0xFF);
//	  HAL_Delay(100);
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
unsigned short updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
  uint16_t i, j;
  static const uint16_t crc_table[256] = { 0x0000,
    0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
    0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
    0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
    0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
    0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
    0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
    0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
    0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
    0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
    0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
    0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
    0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
    0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
    0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
    0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
    0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
    0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
    0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
    0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
    0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
    0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
    0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
    0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
    0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
    0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
    0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
    0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
    0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
    0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
    0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
    0x820D, 0x8207, 0x0202 };

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}

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
		flag_uart_cb = 1;
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
		HAL_GPIO_WritePin(BLINK_LED_PORT, BLINK_LED_PIN, 1);
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
