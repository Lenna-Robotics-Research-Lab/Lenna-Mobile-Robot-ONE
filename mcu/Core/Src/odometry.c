/*
 * odometry.c
 *
 *  Created on: Jun 1, 2024
 *      Author: Lenna Robotics Research Laboratory
 *      		Autonomous Systems Research Branch
 *				Iran University of Science and Technology
 *		GitHub:	github.com/Lenna-Robotics-Research-Lab
 */

#include "odometry.h"
#include "math.h"
#include "mcu_config.h"
#include "stdio.h"


// #########################################################
// ####################  MOTOR ENCODER  ####################
// #########################################################

void LRL_Odometry_Init(odom_cfgType * odom)
{
	__HAL_TIM_SET_COUNTER(odom->enc_right.htim, 0);
	__HAL_TIM_SET_COUNTER(odom->enc_left.htim, 0);

	odom->enc_right.tick = 0;
	odom->enc_left.tick = 0;
	odom->enc_right.tick_prev = 0;
	odom->enc_left.tick_prev = 0;
}

void LRL_Odometry_ReadAngularSpeed(odom_cfgType * odom)
{
	float _temp_dist_right = 0, _temp_dist_left = 0; // this is for the problem with motor position

	odom->enc_right.tick = __HAL_TIM_GET_COUNTER(odom->enc_right.htim);
	odom->enc_left.tick = __HAL_TIM_GET_COUNTER(odom->enc_left.htim);

	int _dir_r,_dir_l;

	uint8_t _dir_count_r, _dir_count_l;	// if 0 -> up; else down

	_dir_count_r = __HAL_TIM_IS_TIM_COUNTING_DOWN(odom->enc_right.htim);
	_dir_count_l = __HAL_TIM_IS_TIM_COUNTING_DOWN(odom->enc_left.htim);


	if(_dir_count_r == 0)
	{
	  if(odom->enc_right.tick - odom->enc_right.tick_prev >= 0)
	  {
		  odom->vel.right = odom->enc_right.tick - odom->enc_right.tick_prev;
	  }
	  else
	  {
		  odom->vel.right = (odom->enc_right.MAX_ARR - odom->enc_right.tick_prev) + odom->enc_right.tick;
	  }
	  _dir_r = 1;
	}
	else
	{
	  if(odom->enc_right.tick_prev - odom->enc_right.tick >= 0)
	  {
		  odom->vel.right = -(odom->enc_right.tick - odom->enc_right.tick_prev);
	  }
	  else
	  {
		  odom->vel.right = (odom->enc_right.MAX_ARR - odom->enc_right.tick) + odom->enc_right.tick_prev;
	  }
	  _dir_r = -1;
	}

	if(_dir_count_l == 0)
	{
	  if(odom->enc_left.tick - odom->enc_left.tick_prev >= 0)
	  {
		  odom->vel.left = odom->enc_left.tick - odom->enc_left.tick_prev;
	  }
	  else
	  {
		  odom->vel.left = (odom->enc_left.MAX_ARR - odom->enc_left.tick_prev) + odom->enc_left.tick;
	  }
	  _dir_l = -1;
	}
	else
	{
	  if(odom->enc_left.tick_prev - odom->enc_left.tick >= 0)
	  {
		  odom->vel.left = -(odom->enc_left.tick - odom->enc_left.tick_prev);
	  }
	  else
	  {
		  odom->vel.left = (odom->enc_left.MAX_ARR - odom->enc_left.tick) + odom->enc_left.tick_prev;
	  }
	  _dir_l = 1;
	}

	if(odom->vel.right < 9)
	{
		odom->vel.right = 0;
	}
	if(odom->vel.left < 9)
	{
		odom->vel.left = 0;
	}


	_temp_dist_right += _dir_r * odom->vel.right*(2*M_PI*odom->diff_robot.WHEEL_RADIUS) / odom->enc_right.MAX_ARR;
	_temp_dist_left += _dir_l * odom->vel.left*(2*M_PI*odom->diff_robot.WHEEL_RADIUS) / odom->enc_left.MAX_ARR ;
	odom->dist.right = (int16_t)_temp_dist_right;
	odom->dist.left  = (int16_t)_temp_dist_left;


	odom->vel.right = _dir_r * odom->vel.right * odom->enc_right.TICK2RPM;
	odom->vel.left = _dir_l * odom->vel.left * odom->enc_left.TICK2RPM;

	odom->enc_right.tick_prev = odom->enc_right.tick;
	odom->enc_left.tick_prev = odom->enc_left.tick;
}
