/**
 * @file motion.h
 * @brief Header file for the robot motion control module.
 *
 * This file defines the structures and function prototypes used to control
 * the motors and manage the motion of a differential drive robot.
 *
 * @author Lenna Robotics Research Laboratory, Autonomous Systems Research Branch, Iran University of Science and Technology
 * @date November 19, 2023
 * @version 1.0
 * @link https://github.com/Lenna-Robotics-Research-Lab
 */

#ifndef INC_MOTION_H_
#define INC_MOTION_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Type Definitions ----------------------------------------------------------*/

/**
 * @struct motor_cfgType
 * @brief Configuration structure for a single motor.
 *
 * This structure holds all the necessary hardware details to control a motor,
 * including its direction pins and the PWM timer settings.
 */
typedef struct
{
	GPIO_TypeDef * 			MOTOR_1_GPIO;	/**< GPIO port for the first direction pin. */
	uint16_t				MOTOR_1_PIN;	/**< GPIO pin number for the first direction pin. */
	GPIO_TypeDef * 			MOTOR_2_GPIO;	/**< GPIO port for the second direction pin. */
	uint16_t				MOTOR_2_PIN;	/**< GPIO pin number for the second direction pin. */
	TIM_HandleTypeDef * 	TIM_PWM_Handle;	/**< Pointer to the PWM timer handle. */
	uint32_t 				TIM_PWM_Channel;/**< The PWM timer channel. */
	uint32_t 				MAX_ARR;		/**< The timer's Auto-Reload Register (ARR) value. */
} motor_cfgType;

/**
 * @struct diffDrive_cfgType
 * @brief Configuration structure for a differential drive robot.
 *
 * This structure combines the individual motor configurations and defines
 * the robot's physical dimensions.
 */
typedef struct
{
	motor_cfgType 			MOTOR_RIGHT;	/**< Configuration for the right motor. */
	motor_cfgType 			MOTOR_LEFT;		/**< Configuration for the left motor. */
	float					WHEEL_RADIUS;	/**< The radius of the robot's wheels. */
	float					WHEEL_DISTANC;	/**< The distance between the center of the wheels. */
} diffDrive_cfgType;

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief Initializes the motion control system.
 *
 * @deprecated This function is commented out in `motion.c` and may be
 * unused. Consider removing it or uncommenting its implementation.
 */
void LRL_Motion_Init(motor_cfgType, motor_cfgType);

/**
 * @brief Sets the speed and direction of a single motor.
 * @param motor The motor configuration structure.
 * @param duty_cycle The desired duty cycle in a range from -100 to 100.
 */
void LRL_Motion_MotorSpeed(motor_cfgType, int8_t);

/**
 * @brief Controls both motors of a differential drive robot.
 * @param diffRobot The differential robot configuration structure.
 * @param duty_cycle_left The duty cycle for the left motor.
 * @param duty_cycle_right The duty cycle for the right motor.
 */
void LRL_Motion_Control(diffDrive_cfgType, int8_t, int8_t);

/**
 * @brief Executes a pre-defined test sequence for the motors.
 * @param diffRobot The differential robot configuration structure.
 */
void LRL_Motion_MotorTest(diffDrive_cfgType diffRobot);

#endif /* INC_MOTION_H_ */
