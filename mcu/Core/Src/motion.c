/**
 * @file motion.c
 * @brief This module provides functions for controlling the motion of a differential drive robot.
 *
 * It includes functions for controlling individual motor speed and direction,
 * and for executing pre-defined motion tests.
 *
 * @author Lenna Robotics Research Laboratory, Autonomous Systems Research Branch, Iran University of Science and Technology
 * @date November 19, 2023
 * @version 1.0
 * @link https://github.com/Lenna-Robotics-Research-Lab
 */

#include "motion.h"
#include "stdbool.h"
#include "stdlib.h"
#include "main.h"


/**
 * @brief Sets the speed and direction of a single motor using PWM.
 * @param motor The motor configuration structure.
 * @param duty_cycle The desired duty cycle for the motor, from -100 to 100.
 *
 * A positive duty cycle commands forward motion, while a negative value commands
 * backward motion. The magnitude determines the speed.
 */
void LRL_Motion_MotorSpeed(motor_cfgType motor, int8_t duty_cycle)
{
	bool dir;
	uint32_t motor_pwm;

	// Determine direction from the sign of the duty cycle.
	dir = (duty_cycle >> 7) & 0x01;
	// Use the absolute value for PWM calculation.
	duty_cycle = abs(duty_cycle);

	// Convert duty cycle percentage to a PWM value based on the timer's ARR.
	motor_pwm = (uint32_t) ((motor.MAX_ARR * duty_cycle) / 100);

	// Set GPIO pins to control the motor driver direction.
	HAL_GPIO_WritePin(motor.MOTOR_1_GPIO, motor.MOTOR_1_PIN, !dir);
	HAL_GPIO_WritePin(motor.MOTOR_2_GPIO, motor.MOTOR_2_PIN, dir);

	// Update the appropriate PWM timer channel's CCR register.
	if (motor.TIM_PWM_Channel == TIM_CHANNEL_1)
		motor.TIM_PWM_Handle->Instance->CCR1 = motor_pwm;
	else if (motor.TIM_PWM_Channel == TIM_CHANNEL_2)
		motor.TIM_PWM_Handle->Instance->CCR2 = motor_pwm;
	else if (motor.TIM_PWM_Channel == TIM_CHANNEL_3)
		motor.TIM_PWM_Handle->Instance->CCR3 = motor_pwm;
	else
		motor.TIM_PWM_Handle->Instance->CCR4 = motor_pwm;
}

/**
 * @brief Controls both motors of a differential drive robot.
 * @param diffRobot The differential robot configuration structure.
 * @param duty_cycle_left The duty cycle for the left motor (-100 to 100).
 * @param duty_cycle_right The duty cycle for the right motor (-100 to 100).
 *
 * This function provides a high-level interface to control the left and right
 * motors independently for various robot movements.
 */
void LRL_Motion_Control(diffDrive_cfgType diffRobot, int8_t duty_cycle_left, int8_t duty_cycle_right)
{
	LRL_Motion_MotorSpeed(diffRobot.MOTOR_LEFT, duty_cycle_left);
	LRL_Motion_MotorSpeed(diffRobot.MOTOR_RIGHT, duty_cycle_right);
}

/**
 * @brief Executes a pre-defined test sequence for the motors.
 * @param diffRobot The differential robot configuration structure.
 *
 * This function is useful for debugging and verifying motor and encoder
 * functionality by running a series of timed movements.
 */
void LRL_Motion_MotorTest(diffDrive_cfgType diffRobot)
{
	LRL_Motion_Control(diffRobot, 60, 60);	// Forward
	HAL_Delay(3000);
	LRL_Motion_Control(diffRobot, 60, 0);	// Pivot left
	HAL_Delay(3000);
	LRL_Motion_Control(diffRobot, 0, 60);	// Pivot right
	HAL_Delay(3000);
	LRL_Motion_Control(diffRobot, -60, -60);	// Backward
	HAL_Delay(3000);
	LRL_Motion_Control(diffRobot, -60, 0);	// Pivot left (backward)
	HAL_Delay(3000);
	LRL_Motion_Control(diffRobot, 0, -60);	// Pivot right (backward)
	HAL_Delay(3000);
	LRL_Motion_Control(diffRobot, 0, 0);	// Stop
}
