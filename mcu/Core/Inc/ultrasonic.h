/*
 * ultrasonic.h
 *
 *  Created on: Oct 18, 2023
 *      Author: Lenna Robotics Research Laboratory
 *      		Autonomous Systems Research Branch
 *				Iran University of Science and Technology
 *		GitHub:	github.com/Lenna-Robotics-Research-Lab
 *
 *	Overview:	Initializing and Reading a HC-SRF04 Ultrasonic Sensor
 *
 *	NOTE		this module is only developed and tested for a SINGLE
 *				ultrasonic sensor connected to the microcontroller!
 *				the script for multiple readings shall be added later
 */

#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_


/* Private Includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"


/* Variable Definitions ------------------------------------------------------*/
typedef struct
{
	GPIO_TypeDef * 		TRIG_GPIO;
	uint16_t       		TRIG_PIN;
	TIM_HandleTypeDef * TIM_Handle;
	TIM_TypeDef  * 		TIM_Instance;
	uint32_t       		IC_TIM_CH;
	uint32_t       		TIM_CLK_MHz;
	uint32_t	   		TIM_PSC;
}ultrasonic_cfgType;


/* Function Prototypes -------------------------------------------------------*/
void LRL_US_Init(ultrasonic_cfgType);
void LRL_US_Trig(ultrasonic_cfgType);
void LRL_US_TMR_OVF_ISR(TIM_HandleTypeDef *, ultrasonic_cfgType);
void LRL_US_TMR_IC_ISR(TIM_HandleTypeDef *, ultrasonic_cfgType);
float LRL_US_Read(ultrasonic_cfgType);


#endif /* INC_ULTRASONIC_H_ */
