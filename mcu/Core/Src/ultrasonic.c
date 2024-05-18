/*
 * ultrasonic.c
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


/* Private Includes ----------------------------------------------------------*/
#include "ultrasonic.h"
#include "utilities.h"
#include "stdbool.h"


/* Variable Definitions ------------------------------------------------------*/
typedef struct
{
	bool	 FIRST_CAPTURED;	// Captured the Start of the Echo 	(flag)
	uint16_t TMR_OVC;			// TIMER Over Flow Counter
	uint32_t TMR_ARR;			// TIMER Auto-Reload Register Value
	uint32_t T1;				// Start(Rise) of the Echo Signal 	(Start Time)
	uint32_t T2;				// End of the Received Signal	  	(End Time)
	uint32_t DIFF;				// Travel Time T2 - T1			  	(Time Difference)
	float    DISTANCE;			// Measured Distance in cm		  	(Distance)
}ultrasonic_info;

static ultrasonic_info us_info = {0};

/* LRL Ultrasonics Functions --------------------------------------------------*/
void LRL_US_Init(ultrasonic_cfgType us)
{
	/*
	 * TODO:
	 * Initialize the Ultrasonic Timer Parameters in this Function
	 * Instead of Using the STM32CubeMX Software.
	 *
	 * NOTE:
	 * One of the Most Important Settings to Initiate is to Activate
	 * the Input Capture Interrupt.
	 *
	 * WARNING:
	 * Always Remember to Start the Timer in Interrupt Mode!
	 */

	// Start the TIM generation
	HAL_TIM_Base_Start_IT(us.TIM_Handle);
	HAL_TIM_IC_Start_IT(us.TIM_Handle, us.IC_TIM_CH);
}

void LRL_US_Trig(ultrasonic_cfgType us)
{
	// 10~25us Delay is Required for the Trigger Signal
	HAL_GPIO_WritePin(us.TRIG_GPIO, us.TRIG_PIN, GPIO_PIN_SET);
	LRL_Delay_Us(15);
	HAL_GPIO_WritePin(us.TRIG_GPIO, us.TRIG_PIN, GPIO_PIN_RESET);

	// One Might Also Enable the Interrupt Mode After Triggering
	// and Disable It Eventually When It is Fully Captured.
}

void LRL_US_TMR_OVF_ISR(TIM_HandleTypeDef* htim, ultrasonic_cfgType us)
{
	if(htim->Instance == us.TIM_Instance)
	{
		us_info.TMR_OVC++;
	}
}

void LRL_US_TMR_IC_ISR(TIM_HandleTypeDef* htim, ultrasonic_cfgType us)
{
	if ((htim->Instance == us.TIM_Instance) && (htim->Channel == us.IC_TIM_CH))
	{
		if (!us_info.FIRST_CAPTURED)
		{
			us_info.T1 = HAL_TIM_ReadCapturedValue(htim, us.IC_TIM_CH);
			us_info.FIRST_CAPTURED = 1;		// the Echo Signal is Captured
			us_info.TMR_OVC = 0;			// Reset the Overflow Counter:
											// this counter is used to evaluate T2
											// and time difference if it overflows
											// the Counter Period (Max. ARR)

			// Reverse the Polarity for Capturing the Incoming Signal
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, us.IC_TIM_CH, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else
		{
			us_info.T2 = HAL_TIM_ReadCapturedValue(htim, us.IC_TIM_CH);

			us_info.TMR_ARR = us.TIM_Instance->ARR; 				// Check for the ARR Value
			us_info.T2 += (us_info.TMR_OVC * (us_info.TMR_ARR+1));	// Estimate T2 When It Overflows the ARR

			us_info.DIFF = us_info.T2 - us_info.T1;

			// the formula to calculate the distance:
			// High-Level Time * .034 / 2; where High-Level time is measured using:
			// High-Level Time = Time Difference / Timer Clock
			us_info.DISTANCE = ((us_info.DIFF * 0.017) / (us.TIM_CLK_MHz / us.TIM_PSC));

			us_info.FIRST_CAPTURED = 0;		// the Echo Signal is Fully Captured

			// Reverse the Polarity for Capturing the Incoming Signal
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, us.IC_TIM_CH, TIM_INPUTCHANNELPOLARITY_RISING);

			// one might disable the interrupt mode after capturing the time difference.
			// if so, REMEMBER to enable it when waiting for an echo; e.g. after triggering
		}
	}
}

float LRL_US_Read(ultrasonic_cfgType us)
{
	return us_info.DISTANCE;
}
