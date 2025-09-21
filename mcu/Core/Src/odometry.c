/**
 * @file odometry.c
 * @brief This module provides odometry functionality for differential drive robots.
 *
 * It initializes encoder values, calculates angular velocity from encoder ticks,
 * and converts tick differences into wheel speeds and incremental distances.
 *
 * @author Lenna Robotics Research Laboratory, Autonomous Systems Research Branch, Iran University of Science and Technology
 * @date June 1, 2024
 * @version 1.0
 * @link https://github.com/Lenna-Robotics-Research-Lab
 */

#include "odometry.h"
#include "math.h"
#include "mcu_config.h"
#include "stdio.h"


/**
 * @brief Initializes the odometry system.
 *
 * This function resets the hardware encoder counters and clears previous tick values
 * to ensure consistent measurements from system startup.
 *
 * @param odom Pointer to the odometry configuration structure.
 */
void LRL_Odometry_Init(odom_cfgType * odom)
{
	// Reset hardware encoder counters to zero.
	__HAL_TIM_SET_COUNTER(odom->enc_right.htim, 0);
	__HAL_TIM_SET_COUNTER(odom->enc_left.htim, 0);

	// Reset software tick and previous tick values.
	odom->enc_right.tick = 0;
	odom->enc_left.tick = 0;
	odom->enc_right.tick_prev = 0;
	odom->enc_left.tick_prev = 0;
}


/**
 * @brief Reads and updates wheel angular speeds and distances from encoders.
 *
 * This function performs the core odometry calculations:
 * - Reads current encoder tick counts and their counting direction.
 * - Computes the tick difference since the last update, correctly handling timer overflows.
 * - Converts the tick difference into incremental wheel velocities (RPM) and distances.
 * - Applies a simple noise filter to ignore small movements.
 * - Updates the previous tick values for the next calculation cycle.
 *
 * @param odom Pointer to the odometry configuration structure.
 */
void LRL_Odometry_ReadAngularSpeed(odom_cfgType * odom)
{
	// Temporary variables for distance calculation.
	float _temp_dist_right = 0, _temp_dist_left = 0;

	// Read current tick counts from the hardware timers.
	odom->enc_right.tick = __HAL_TIM_GET_COUNTER(odom->enc_right.htim);
	odom->enc_left.tick  = __HAL_TIM_GET_COUNTER(odom->enc_left.htim);

	// Direction indicators: +1 for forward, -1 for backward.
	int _dir_r, _dir_l;
	// Raw direction count from the hardware timer: 0 = up, 1 = down.
	uint8_t _dir_count_r, _dir_count_l;

	// Determine the counting direction from hardware timers.
	_dir_count_r = __HAL_TIM_IS_TIM_COUNTING_DOWN(odom->enc_right.htim);
	_dir_count_l = __HAL_TIM_IS_TIM_COUNTING_DOWN(odom->enc_left.htim);

	/* ------------------- Right wheel velocity calculation ------------------- */
	if(_dir_count_r == 0) // Timer is counting up (forward motion).
	{
		if(odom->enc_right.tick - odom->enc_right.tick_prev >= 0)
			odom->vel.right = odom->enc_right.tick - odom->enc_right.tick_prev;
		else
			// Handle timer overflow by calculating the difference.
			odom->vel.right = (odom->enc_right.MAX_ARR - odom->enc_right.tick_prev) + odom->enc_right.tick;

		_dir_r = 1; // Set direction to forward.
	}
	else // Timer is counting down (backward motion).
	{
		if(odom->enc_right.tick_prev - odom->enc_right.tick >= 0)
			odom->vel.right = -(odom->enc_right.tick - odom->enc_right.tick_prev);
		else
			// Handle timer underflow by calculating the difference.
			odom->vel.right = (odom->enc_right.MAX_ARR - odom->enc_right.tick) + odom->enc_right.tick_prev;

		_dir_r = -1; // Set direction to backward.
	}

	/* ------------------- Left wheel velocity calculation ------------------- */
	if(_dir_count_l == 0) // Timer is counting up (forward motion).
	{
		if(odom->enc_left.tick - odom->enc_left.tick_prev >= 0)
			odom->vel.left = odom->enc_left.tick - odom->enc_left.tick_prev;
		else
			// Handle timer overflow by calculating the difference.
			odom->vel.left = (odom->enc_left.MAX_ARR - odom->enc_left.tick_prev) + odom->enc_left.tick;

		_dir_l = -1; // Left wheel convention: forward motion = -1.
	}
	else // Timer is counting down (backward motion).
	{
		if(odom->enc_left.tick_prev - odom->enc_left.tick >= 0)
			odom->vel.left = -(odom->enc_left.tick - odom->enc_left.tick_prev);
		else
			// Handle timer underflow by calculating the difference.
			odom->vel.left = (odom->enc_left.MAX_ARR - odom->enc_left.tick) + odom->enc_left.tick_prev;

		_dir_l = 1;
	}

	// Apply a simple noise filter to ignore small tick velocities.
	if(odom->vel.right < 9)
		odom->vel.right = 0;
	if(odom->vel.left < 9)
		odom->vel.left = 0;

	/* ------------------- Distance calculation ------------------- */
	// Convert the tick differences into incremental arc lengths for each wheel.
	// Formula: distance = (ticks_diff / ticks_per_revolution) * circumference
	_temp_dist_right += _dir_r * odom->vel.right * (2 * M_PI * odom->diff_robot.WHEEL_RADIUS) / odom->enc_right.MAX_ARR;
	_temp_dist_left  += _dir_l * odom->vel.left  * (2 * M_PI * odom->diff_robot.WHEEL_RADIUS) / odom->enc_left.MAX_ARR;

	// Store incremental distances as integers (in millimeters).
	odom->dist.right = (int16_t)_temp_dist_right;
	odom->dist.left  = (int16_t)_temp_dist_left;

	/* ------------------- Velocity conversion ------------------- */
	// Convert tick differences into RPM using a calibrated factor.
	odom->vel.right = _dir_r * odom->vel.right * odom->enc_right.TICK2RPM;
	odom->vel.left  = _dir_l * odom->vel.left  * odom->enc_left.TICK2RPM;

	// Save the current ticks to be used as 'previous' ticks in the next cycle.
	odom->enc_right.tick_prev = odom->enc_right.tick;
	odom->enc_left.tick_prev  = odom->enc_left.tick;
}
