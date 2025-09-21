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
#include "main.h"


/**
 * @brief Initializes the PID controller parameters.
 * @param pid_cfg Pointer to the PID configuration structure.
 * @param AntiWindup Enables or disables the anti-windup feature (1 for enabled, 0 for disabled).
 *
 * This function resets all PID state variables and sets the initial configuration,
 * including the anti-windup flag.
 */
void LRL_PID_Init(pid_cfgType *pid_cfg,uint8_t AntiWindup)
{
	// Reset PID state variables.
	pid_cfg->Anti_windup_EN = AntiWindup;
	pid_cfg->Prev_Measurement = 0.0f;
	pid_cfg->Integrator_Amount = 0;
	pid_cfg->Prev_Error = 0.0f;
	pid_cfg->Control_Signal = 0;
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
void LRL_PID_Update(pid_cfgType *pid_cfg, int16_t measurement, int16_t set_point)
{
	int8_t _dir;

	// Determine the direction of the control signal.
	_dir = set_point / abs(set_point);

	// Use absolute values for error calculation to handle direction separately.
	measurement = abs(measurement);
	set_point = abs(set_point);

	// Calculate the current error and scale it.
	pid_cfg->Error = set_point - measurement;
	pid_cfg->Error = pid_cfg->Error * Speed2PWM_Rate;

	// Calculate the integral term using the trapezoidal rule.
	pid_cfg->Integrator_Amount += (pid_cfg->Ts * (pid_cfg->Ki * (pid_cfg->Error + pid_cfg->Prev_Error)));

	// Derivative term is currently set to zero.
	pid_cfg->Differentiator_Amount = 0;

	// Calculate the control signal as the sum of PID terms.
	pid_cfg->Control_Signal = (pid_cfg->Kp * pid_cfg->Error) + pid_cfg->Integrator_Amount + pid_cfg->Differentiator_Amount;

	// Anti-windup implementation.
	if(pid_cfg->Anti_windup_EN == 1)
	{
		// If the control signal is within limits, update the anti-windup amount.
		if(pid_cfg->Control_Signal <= Upper_Saturation_Limit)
		{
			pid_cfg->Wind_Up_Amount = pid_cfg->Integrator_Amount;
		}
		else
		{
			// If saturated, re-calculate the control signal using the last non-saturated integral value.
			pid_cfg->Control_Signal = (pid_cfg->Kp * pid_cfg->Error) + pid_cfg->Wind_Up_Amount + pid_cfg->Differentiator_Amount;
		}
	}

	// Apply output saturation limits.
	if(pid_cfg->Control_Signal > pid_cfg->Upper_Limit_Saturation)
	{
		pid_cfg->Control_Signal = pid_cfg->Upper_Limit_Saturation;
	}
	else if(pid_cfg->Control_Signal < pid_cfg->Lower_Limit_Saturation)
	{
		pid_cfg->Control_Signal = pid_cfg->Lower_Limit_Saturation;
	}

	// Apply the determined direction to the final control signal.
	pid_cfg->Control_Signal = pid_cfg->Control_Signal * _dir;

	// Update previous values for the next iteration.
	pid_cfg->Prev_Measurement = measurement;
	pid_cfg->Prev_Error = pid_cfg->Error;
}
