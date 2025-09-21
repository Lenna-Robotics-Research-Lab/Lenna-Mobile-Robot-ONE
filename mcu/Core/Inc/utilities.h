/*
 * utilities.h
 *
 *  Created on: Oct 18, 2023
 *      Author: Lenna Robotics Research Laboratory
 *      		Autonomous Systems Research Branch
 *				Iran University of Science and Technology
 *		GitHub:	github.com/Lenna-Robotics-Research-Lab
 *
 *	Overview:	Generating Time Delay in MicroSeconds(us)
 *
 *	NOTE:		this module is developed based on TIM1!
 *				for other timers some modifications might be needed.
 */

#ifndef INC_UTILITIES_H_
#define INC_UTILITIES_H_

#include "stm32f4xx_hal.h"
#include "tim.h"

#define DELAY_TIM_INISTANCE	TIM1
#define DELAY_TIM_HANDLE	htim1

void LRL_Delay_Init();
void LRL_Delay_Us(volatile uint16_t);

#endif /* INC_UTILITIES_H_ */
