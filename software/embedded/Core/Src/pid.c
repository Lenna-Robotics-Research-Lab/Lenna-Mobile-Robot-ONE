/**
 * @file pid.c
 * @brief This module provides a PID (Proportional-Integral-Derivative) controller implementation.
 *
 * It includes functions for initializing the controller, updating the control
 * signal based on sensor feedback, and handling anti-windup and saturation.
 *
 * @author Lenna Robotics Research Laboratory, Autonomous Systems Research Branch, Iran University of Science and Technology
 * @date October 11, 2025
 * @version 1.0
 * @link https://github.com/Lenna-Robotics-Research-Lab
 */

#include <mcu_config.h>
#include "pid.h"
#include "odometry.h"
#include "main.h"


/**
 * @brief Initializes the PID controller parameters.
 * @param pid_cfg Pointer to the PID configuration structure.
 * @param AntiWindup Enables or disables the anti-windup feature (1 for enabled, 0 for disabled).
 *
 * This function resets all PID state variables and sets the initial configuration,
 * including the anti-windup flag.
 */
void LRL_PID_Init(pid_cfgType *pid, uint8_t AntiWindup)
{
	// Reset PID state variables.
	pid->Anti_windup_EN 			= AntiWindup;
	pid->Prev_Measurement_r 		= 0.0f;
	pid->Prev_Measurement_l			= 0.0f;
	pid->Differentiator_Amount_r 	= 0;
	pid->Differentiator_Amount_l	= 0;
	pid->Integrator_Amount_r 		= 0;
	pid->Integrator_Amount_l		= 0;
	pid->Prev_Error_r 				= 0.0f;
	pid->Prev_Error_l				= 0.0f;
	pid->Control_Signal_r 			= 0;
	pid->Control_Signal_l			= 0;
	pid->Kp_r						= Proportional_Gain_RIGHT_MOTOR;
	pid->Kp_l						= Proportional_Gain_LEFT_MOTOR;
	pid->Ki_r						= Integral_Gain_RIGHT_MOTOR;
	pid->Ki_l						= Integral_Gain_LEFT_MOTOR;
	pid->Kd_r						= Derivative_Gain_RIGHT_MOTOR;
	pid->Kd_l						= Derivative_Gain_LEFT_MOTOR;
}


/**
 * @brief Updates the PID control signal.
 * @param pid_cfg Pointer to the PID configuration structure.
 * @param measurement The current measured value (e.g., motor speed).
 * @param set_point The desired set point.
 *
 * This function calculates the error and then computes the proportional, integral,
 * and derivative terms to generate a new control signal. It also handles
 * direction, anti-windup, and output saturation.
 */
void LRL_PID_Update(pid_cfgType *pid, odom_cfgType *odom, int16_t left_setpoint, int16_t right_setpoint)
{

	int8_t _dir_r, _dir_l;
	int16_t _vel_r, _vel_l;

	LRL_Odometry_ReadAngularSpeed(odom);

	_vel_l = odom->vel.left;
	_vel_r = odom->vel.right;

	// Determine the direction of the control signal.
	_dir_r = right_setpoint / abs(right_setpoint);
	_dir_l = left_setpoint / abs(left_setpoint);

	// Use absolute values for error calculation to handle direction separately.

//	_vel_r 			= abs(_vel_r);
//	right_setpoint 	= abs(right_setpoint);
//
//	_vel_l 			= abs(_vel_l);
//	left_setpoint 	= abs(left_setpoint);

	// Calculate the current error and scale it.
	pid->Error_r = (right_setpoint - _vel_r) * Speed2PWM_Rate;
	pid->Error_l = (left_setpoint - _vel_l) * Speed2PWM_Rate;

	// Calculate the integral term using the trapezoidal rule.
	pid->Integrator_Amount_r += (pid->Ts * (pid->Ki_r * (pid->Error_r + pid->Prev_Error_r)));
	pid->Integrator_Amount_l += (pid->Ts * (pid->Ki_l * (pid->Error_l + pid->Prev_Error_l)));

	// Derivative term is currently set to zero.
	pid->Differentiator_Amount_r = 0;
	pid->Differentiator_Amount_l = 0;

	// Calculate the control signal as the sum of PID terms.
	pid->Control_Signal_r = (pid->Kp_r * pid->Error_r) + pid->Integrator_Amount_r + pid->Differentiator_Amount_r;
	pid->Control_Signal_l = (pid->Kp_l * pid->Error_l) + pid->Integrator_Amount_l + pid->Differentiator_Amount_l;

	// Anti-windup implementation.
	if(pid->Anti_windup_EN == 1)
	{
		// If the control signal is within limits, update the anti-windup amount.
		if(pid->Control_Signal_r <= Upper_Saturation_Limit)
		{
			pid->Wind_Up_Amount_r = pid->Integrator_Amount_r;
		}
		else
		{
			// If saturated, re-calculate the control signal using the last non-saturated integral value.
			pid->Control_Signal_r = (pid->Kp_r * pid->Error_r) + pid->Wind_Up_Amount_r + pid->Differentiator_Amount_r;
		}
	}

	if(pid->Anti_windup_EN == 1)
	{
		// If the control signal is within limits, update the anti-windup amount.
		if(pid->Control_Signal_l <= Upper_Saturation_Limit)
		{
			pid->Wind_Up_Amount_l = pid->Integrator_Amount_l;
		}
		else
		{
			// If saturated, re-calculate the control signal using the last non-saturated integral value.
			pid->Control_Signal_l = (pid->Kp_l * pid->Error_l) + pid->Wind_Up_Amount_l + pid->Differentiator_Amount_l;
		}
	}

	// Apply output saturation limits.
	if(pid->Control_Signal_r > pid->Upper_Limit_Saturation)
	{
		pid->Control_Signal_r = pid->Upper_Limit_Saturation;
	}
	else if(pid->Control_Signal_r < pid->Lower_Limit_Saturation)
	{
		pid->Control_Signal_r = pid->Lower_Limit_Saturation;
	}

	if(pid->Control_Signal_l > pid->Upper_Limit_Saturation)
	{
		pid->Control_Signal_l = pid->Upper_Limit_Saturation;
	}
	else if(pid->Control_Signal_l < pid->Lower_Limit_Saturation)
	{
		pid->Control_Signal_l = pid->Lower_Limit_Saturation;
	}

	// Apply the determined direction to the final control signal.
	pid->Control_Signal_r = pid->Control_Signal_r * _dir_r;
	pid->Control_Signal_l = pid->Control_Signal_l * _dir_l;

	// Update previous values for the next iteration.
	pid->Prev_Measurement_r = _vel_r;
	pid->Prev_Error_r = pid->Error_r;

	pid->Prev_Measurement_l = _vel_l;
	pid->Prev_Error_l = pid->Error_l;
}
