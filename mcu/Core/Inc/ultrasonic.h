/**
 * @file ultrasonic.h
 * @brief Header file for HC-SRF04 ultrasonic sensor module.
 *
 * @author Lenna Robotics Research Laboratory
 * @date October 18, 2023
 * @version 1.0
 * @link https://github.com/Lenna-Robotics-Research-Lab
 */

#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Type Definitions ----------------------------------------------------------*/
/**
 * @struct ultrasonic_cfgType
 * @brief Configuration structure for an ultrasonic sensor.
 *
 * This structure holds all the necessary hardware configuration details for the
 * HC-SRF04 sensor, including the GPIO pins for the trigger and the timer
 * settings for input capture.
 */
typedef struct
{
	GPIO_TypeDef * 		TRIG_GPIO;		/**< GPIO port for the trigger pin. */
	uint16_t	 		TRIG_PIN;		/**< GPIO pin number for the trigger. */
	TIM_HandleTypeDef * TIM_Handle;		/**< Pointer to the timer handle structure. */
	TIM_TypeDef  		* TIM_Instance;	/**< Pointer to the timer instance (e.g., TIM2). */
	uint32_t			IC_TIM_CH;		/**< Input Capture timer channel. */
	uint32_t			TIM_CLK_MHz;	/**< Timer clock frequency in MHz. */
	uint32_t			TIM_PSC;		/**< Timer prescaler value. */
} ultrasonic_cfgType;

/* Function Prototypes -------------------------------------------------------*/
/**
 * @brief Initializes the timer and input capture for the sensor.
 * @param us The configuration structure for the ultrasonic sensor.
 */
void LRL_US_Init(ultrasonic_cfgType us);

/**
 * @brief Generates a trigger pulse for the sensor.
 * @param us The configuration structure for the ultrasonic sensor.
 */
void LRL_US_Trig(ultrasonic_cfgType us);

/**
 * @brief Timer overflow interrupt service routine (ISR) handler.
 * @param htim Pointer to the timer handle structure.
 * @param us The configuration structure.
 */
void LRL_US_TMR_OVF_ISR(TIM_HandleTypeDef *htim, ultrasonic_cfgType us);

/**
 * @brief Timer input capture interrupt service routine (ISR) handler.
 * @param htim Pointer to the timer handle structure.
 * @param us The configuration structure.
 */
void LRL_US_TMR_IC_ISR(TIM_HandleTypeDef *htim, ultrasonic_cfgType us);

/**
 * @brief Returns the last measured distance from the sensor.
 * @param us The configuration structure (used for function signature consistency).
 * @return The distance in centimeters as a float value.
 */
float LRL_US_Read(ultrasonic_cfgType us);

#endif /* INC_ULTRASONIC_H_ */
