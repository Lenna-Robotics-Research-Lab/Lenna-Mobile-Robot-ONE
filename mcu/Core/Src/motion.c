/*
 * motion.c
 *
 *  Created on: Nov 19, 2023
 *  Author: Lenna Robotics Research Laboratory
 *      	Autonomous Systems Research Branch
 *			Iran University of Science and Technology
 *  GitHub:	github.com/Lenna-Robotics-Research-Lab
 */

#include "motion.h"
#include "stdbool.h"
#include "stdlib.h"

//void LRL_Motion_Init(motor_cfgType motor_left, motor_cfgType motor_right)
//{
//	// Starting the Timer PWM
//	HAL_TIM_PWM_Start(motor_left.TIM_PWM_Handle, motor_left.TIM_PWM_Channel);
//	HAL_TIM_PWM_Start(motor_right.TIM_PWM_Handle, motor_right.TIM_PWM_Channel);
//
//	// Starting the Timer Encoder Mode
//	HAL_TIM_Encoder_Start(motor_left.TIM_ENC_Handle, TIM_CHANNEL_ALL);
//	HAL_TIM_Encoder_Start(motor_right.TIM_ENC_Handle, TIM_CHANNEL_ALL);
//}

void LRL_Motor_Speed(motor_cfgType motor, int8_t duty_cycle)
{
	bool dir;
	uint32_t motor_pwm;

	dir = (duty_cycle >> 7) & 0x01;	// if MSB is 1 -> neg
	duty_cycle = abs(duty_cycle);

	motor_pwm = (uint32_t) ((motor.MAX_ARR * duty_cycle) / 100);

    HAL_GPIO_WritePin(motor.MOTOR_1_GPIO, motor.MOTOR_1_PIN, !dir);
    HAL_GPIO_WritePin(motor.MOTOR_2_GPIO, motor.MOTOR_2_PIN, dir);

    if (motor.TIM_PWM_Channel == TIM_CHANNEL_1)
    	motor.TIM_PWM_Handle->Instance->CCR1 = motor_pwm;
    else if (motor.TIM_PWM_Channel == TIM_CHANNEL_2)
    	motor.TIM_PWM_Handle->Instance->CCR2 = motor_pwm;
    else if (motor.TIM_PWM_Channel == TIM_CHANNEL_3)
    	motor.TIM_PWM_Handle->Instance->CCR3 = motor_pwm;
    else
    	motor.TIM_PWM_Handle->Instance->CCR4 = motor_pwm;
}

void LRL_Motion_Control(diffDrive_cfgType diffRobot, int8_t duty_cycle_left, int8_t duty_cycle_right)
{
	LRL_Motor_Speed(diffRobot.MOTOR_LEFT, duty_cycle_left);
	LRL_Motor_Speed(diffRobot.MOTOR_RIGHT, duty_cycle_right);
}
