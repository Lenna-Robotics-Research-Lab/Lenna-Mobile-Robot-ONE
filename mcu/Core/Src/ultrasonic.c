/**
 * @file ultrasonic.c
 * @brief Implementation for a single HC-SRF04 ultrasonic sensor.
 *
 * @author Lenna Robotics Research Laboratory
 * @date October 18, 2023
 * @version 1.0
 * @link https://github.com/Lenna-Robotics-Research-Lab
 */

/* Private Includes ----------------------------------------------------------*/
#include "ultrasonic.h"
#include "utilities.h"
#include "stdbool.h"

/* Private Type Definitions --------------------------------------------------*/
/**
 * @struct ultrasonic_info
 * @brief Structure for internal state and measurement data.
 */
typedef struct
{
	bool	 FIRST_CAPTURED;	/**< Flag for the rising edge of the echo signal. */
	uint16_t TMR_OVC;			/**< Counter for timer overflows. */
	uint32_t TMR_ARR;			/**< Timer's Auto-Reload Register value. */
	uint32_t T1;				/**< Time value of the rising edge. */
	uint32_t T2;				/**< Time value of the falling edge. */
	uint32_t DIFF;				/**< Calculated time difference (T2 - T1). */
	float	 DISTANCE;			/**< Measured distance. */
}ultrasonic_info;

/* Private Variable Definitions ------------------------------------------------------*/
static ultrasonic_info us_info = {0};

/* LRL Ultrasonics Functions --------------------------------------------------*/
/**
 * @brief Initializes the timer and input capture for the sensor.
 * @param us The configuration structure.
 *
 * This starts the timer and input capture interrupts.
 */
void LRL_US_Init(ultrasonic_cfgType us)
{
	HAL_TIM_Base_Start_IT(us.TIM_Handle);
	HAL_TIM_IC_Start_IT(us.TIM_Handle, us.IC_TIM_CH);
}

/**
 * @brief Generates a trigger pulse for the sensor.
 * @param us The configuration structure.
 */
void LRL_US_Trig(ultrasonic_cfgType us)
{
	HAL_GPIO_WritePin(us.TRIG_GPIO, us.TRIG_PIN, GPIO_PIN_SET);
	LRL_Delay_Us(15);
	HAL_GPIO_WritePin(us.TRIG_GPIO, us.TRIG_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Timer overflow interrupt service routine (ISR) handler.
 * @param htim Pointer to the timer handle structure.
 * @param us The configuration structure.
 */
void LRL_US_TMR_OVF_ISR(TIM_HandleTypeDef* htim, ultrasonic_cfgType us)
{
	if(htim->Instance == us.TIM_Instance)
	{
		us_info.TMR_OVC++;
	}
}

/**
 * @brief Timer input capture interrupt service routine (ISR) handler.
 * @param htim Pointer to the timer handle structure.
 * @param us The configuration structure.
 */
void LRL_US_TMR_IC_ISR(TIM_HandleTypeDef* htim, ultrasonic_cfgType us)
{
	if ((htim->Instance == us.TIM_Instance) && (htim->Channel == us.IC_TIM_CH))
	{
		if (!us_info.FIRST_CAPTURED)
		{
			us_info.T1 = HAL_TIM_ReadCapturedValue(htim, us.IC_TIM_CH);
			us_info.FIRST_CAPTURED = 1;
			us_info.TMR_OVC = 0;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, us.IC_TIM_CH, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else
		{
			us_info.T2 = HAL_TIM_ReadCapturedValue(htim, us.IC_TIM_CH);

			us_info.TMR_ARR = us.TIM_Instance->ARR;
			us_info.T2 += (us_info.TMR_OVC * (us_info.TMR_ARR+1));

			us_info.DIFF = us_info.T2 - us_info.T1;
			us_info.DISTANCE = ((us_info.DIFF * 0.017) / (us.TIM_CLK_MHz / us.TIM_PSC));

			us_info.FIRST_CAPTURED = 0;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, us.IC_TIM_CH, TIM_INPUTCHANNELPOLARITY_RISING);
		}
	}
}

/**
 * @brief Returns the last measured distance.
 * @param us The configuration structure.
 * @return The distance in centimeters.
 */
float LRL_US_Read(ultrasonic_cfgType us)
{
	return us_info.DISTANCE;
}
