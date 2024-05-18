/*
 * motion.h
 *
 *  Created on: Nov 19, 2023
 *  Author: Lenna Robotics Research Laboratory
 *      	Autonomous Systems Research Branch
 *			Iran University of Science and Technology
 *	GitHub:	github.com/Lenna-Robotics-Research-Lab
 */

#ifndef INC_MOTION_H_
#define INC_MOTION_H_

/* Private Includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"


/* Variable Definitions ------------------------------------------------------*/
typedef struct
{
	GPIO_TypeDef * 		MOTOR_1_GPIO;
	uint16_t       		MOTOR_1_PIN;
	GPIO_TypeDef * 		MOTOR_2_GPIO;
	uint16_t       		MOTOR_2_PIN;
	TIM_HandleTypeDef * TIM_PWM_Handle;
	uint32_t 			TIM_PWM_Channel;
	uint32_t 			MAX_ARR;
}motor_cfgType;

typedef struct
{
	motor_cfgType 	MOTOR_RIGHT;	// Right Motor Config Parameters
	motor_cfgType 	MOTOR_LEFT;		// Left Motor Config Parameters
	uint16_t		WHEEL_RADIUS;	// R Radius of Differential Robot's Wheels
	uint16_t		WHEEL_DISTANC;	// L Distance Between the Center of the Wheels
}diffDrive_cfgType;

void LRL_Motion_Init(motor_cfgType, motor_cfgType);
void LRL_Motor_Speed(motor_cfgType, int8_t);
void LRL_Motion_Control(diffDrive_cfgType, int8_t, int8_t);

#endif /* INC_MOTION_H_ */
