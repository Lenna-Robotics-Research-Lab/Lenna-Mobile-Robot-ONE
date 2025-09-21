/*
 * odometry.c
 *
 *  Created on: Jun 1, 2024
 *      Author: Lenna Robotics Research Laboratory
 *      		Autonomous Systems Research Branch
 *				Iran University of Science and Technology
 *		GitHub:	github.com/Lenna-Robotics-Research-Lab
 *
 *  Description:
 *  This module provides odometry functionality for differential drive robots.
 *  It initializes encoder values, calculates angular velocity from encoder ticks,
 *  and converts tick differences into wheel speeds and incremental distances.
 */

#include "odometry.h"
#include "math.h"
#include "mcu_config.h"
#include "stdio.h"


/**
 * @brief Initialize the odometry system
 *
 * Resets encoder counters and clears previous tick values
 * to ensure consistent measurements at system startup.
 *
 * @param odom Pointer to the odometry configuration structure
 */
void LRL_Odometry_Init(odom_cfgType * odom)
{
	// Reset hardware encoder counters
	__HAL_TIM_SET_COUNTER(odom->enc_right.htim, 0);
	__HAL_TIM_SET_COUNTER(odom->enc_left.htim, 0);

	// Reset tick values and previous tick values in software
	odom->enc_right.tick = 0;
	odom->enc_left.tick = 0;
	odom->enc_right.tick_prev = 0;
	odom->enc_left.tick_prev = 0;
}


/**
 * @brief Read and update wheel angular speeds from encoders
 *
 * This function:
 *   - Reads encoder tick counts and direction
 *   - Computes tick differences since last update
 *   - Converts tick differences to wheel velocities (RPM)
 *   - Computes incremental wheel displacements (distance traveled)
 *   - Updates previous tick values for the next cycle
 *
 * @param odom Pointer to the odometry configuration structure
 */
void LRL_Odometry_ReadAngularSpeed(odom_cfgType * odom)
{
	// Temporary storage for wheel displacements
	float _temp_dist_right = 0, _temp_dist_left = 0;

	// Read current encoder ticks from hardware
	odom->enc_right.tick = __HAL_TIM_GET_COUNTER(odom->enc_right.htim);
	odom->enc_left.tick  = __HAL_TIM_GET_COUNTER(odom->enc_left.htim);

	int _dir_r, _dir_l;                   // Direction indicators: +1 (forward), -1 (backward)
	uint8_t _dir_count_r, _dir_count_l;   // Raw direction count: 0 = up, 1 = down

	// Check counting direction from hardware timers
	_dir_count_r = __HAL_TIM_IS_TIM_COUNTING_DOWN(odom->enc_right.htim);
	_dir_count_l = __HAL_TIM_IS_TIM_COUNTING_DOWN(odom->enc_left.htim);

	/* ------------------- Right wheel velocity calculation ------------------- */
	if(_dir_count_r == 0)  // Timer counting up → forward motion
	{
		if(odom->enc_right.tick - odom->enc_right.tick_prev >= 0)
			odom->vel.right = odom->enc_right.tick - odom->enc_right.tick_prev;
		else
			odom->vel.right = (odom->enc_right.MAX_ARR - odom->enc_right.tick_prev) + odom->enc_right.tick;

		_dir_r = 1; // forward
	}
	else  // Timer counting down → backward motion
	{
		if(odom->enc_right.tick_prev - odom->enc_right.tick >= 0)
			odom->vel.right = -(odom->enc_right.tick - odom->enc_right.tick_prev);
		else
			odom->vel.right = (odom->enc_right.MAX_ARR - odom->enc_right.tick) + odom->enc_right.tick_prev;

		_dir_r = -1; // backward
	}

	/* ------------------- Left wheel velocity calculation ------------------- */
	if(_dir_count_l == 0)  // Timer counting up → forward motion
	{
		if(odom->enc_left.tick - odom->enc_left.tick_prev >= 0)
			odom->vel.left = odom->enc_left.tick - odom->enc_left.tick_prev;
		else
			odom->vel.left = (odom->enc_left.MAX_ARR - odom->enc_left.tick_prev) + odom->enc_left.tick;

		_dir_l = -1; // Note: left wheel convention → forward motion = -1
	}
	else  // Timer counting down → backward motion
	{
		if(odom->enc_left.tick_prev - odom->enc_left.tick >= 0)
			odom->vel.left = -(odom->enc_left.tick - odom->enc_left.tick_prev);
		else
			odom->vel.left = (odom->enc_left.MAX_ARR - odom->enc_left.tick) + odom->enc_left.tick_prev;

		_dir_l = 1;
	}

	// Apply simple noise filter: ignore very small velocities
	if(odom->vel.right < 9)
		odom->vel.right = 0;
	if(odom->vel.left < 9)
		odom->vel.left = 0;

	/* ------------------- Distance calculation ------------------- */
	// Convert tick differences into incremental arc lengths (per wheel)
	_temp_dist_right += _dir_r * odom->vel.right * (2 * M_PI * odom->diff_robot.WHEEL_RADIUS) / odom->enc_right.MAX_ARR;
	_temp_dist_left  += _dir_l * odom->vel.left  * (2 * M_PI * odom->diff_robot.WHEEL_RADIUS) / odom->enc_left.MAX_ARR;

	// Store incremental distances as integers
	odom->dist.right = (int16_t)_temp_dist_right;
	odom->dist.left  = (int16_t)_temp_dist_left;

	/* ------------------- Velocity conversion ------------------- */
	// Convert tick counts to RPM (using calibration factor TICK2RPM)
	odom->vel.right = _dir_r * odom->vel.right * odom->enc_right.TICK2RPM;
	odom->vel.left  = _dir_l * odom->vel.left  * odom->enc_left.TICK2RPM;

	// Save current ticks for next iteration
	odom->enc_right.tick_prev = odom->enc_right.tick;
	odom->enc_left.tick_prev  = odom->enc_left.tick;
}
