/*
 * odometry.h
 *
 *  Created on: Jun 1, 2024
 *      Author: Lenna Robotics Research Laboratory
 *      		Autonomous Systems Research Branch
 *				Iran University of Science and Technology
 *		GitHub:	github.com/Lenna-Robotics-Research-Lab
 */

#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_

#include "i2c.h"
#include "tim.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "motion.h"



// ############################################################
// ####################  ODOMETRY STRUCTS  ####################
// ############################################################


typedef struct
{
	int16_t	right;
	int16_t	left;
} wheel_velocity;

typedef struct
{
	int16_t right;
	int16_t left;
} wheel_position;

typedef struct
{
	TIM_HandleTypeDef * htim;

	uint16_t 	MAX_ARR;
	float 		TICK2RPM;

	uint16_t 	tick;
	uint16_t 	tick_prev;
} encoder_cfgType;



typedef struct
{
//	imu_cfgType 		imu;
	encoder_cfgType		enc_right;
	encoder_cfgType		enc_left;
	diffDrive_cfgType	diff_robot;


//	linear_position 	pose;
//	angular_position 	angle;
//	accelerometer 		accel;
//	gyroscope 			gyro;
//	magnetometer 		mag;

	wheel_velocity 		vel;
	wheel_position 		dist;
} odom_cfgType;

// ##############################################################
// ####################  FUNCTION PROTOTYPE  ####################
// ##############################################################

void LRL_Odometry_Init(odom_cfgType * odom);
void LRL_Odometry_ReadAngularSpeed(odom_cfgType * odom);

#endif /* INC_ODOMETRY_H_ */
