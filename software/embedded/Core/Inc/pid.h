/**
 * @file pid.h
 * @brief Header file for the PID (Proportional-Integral-Derivative) controller module.
 *
 * This file defines the structures and function prototypes for implementing a
 * PID controller with anti-windup and saturation limits. It includes
 * motor-specific gain constants for both left and right motors.
 *
 * @author Lenna Robotics Research Laboratory, Autonomous Systems Research Branch, Iran University of Science and Technology
 * @date March 24, 2024
 * @version 1.0
 * @link https://github.com/Lenna-Robotics-Research-Lab
 */

#ifndef INC_PID_H_
#define INC_PID_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "odometry.h"


/* Defines -------------------------------------------------------------------*/

/**
 * @name PID Gain Constants
 * @brief Tuned PID gain values for the left and right motors.
 *
 * These values were calculated using a motor identification process in MATLAB Simulink.
 */
///@{
#define Proportional_Gain_LEFT_MOTOR 1.0f
#define Integral_Gain_LEFT_MOTOR 3.0f
#define Derivative_Gain_LEFT_MOTOR 0.05f

#define Proportional_Gain_RIGHT_MOTOR 1.2f
#define Integral_Gain_RIGHT_MOTOR 3.0f
#define Derivative_Gain_RIGHT_MOTOR 0.01f
///@}

/**
 * @name PID Limits and Timing
 * @brief Constants for saturation limits and sampling time.
 */
///@{
#define Upper_Saturation_Limit 100		/**< Upper limit for the PID control signal. */
#define Lower_Saturation_Limit 0		/**< Lower limit for the PID control signal. */
#define Sampling_Time 0.01f				/**< The sampling period (Ts) in seconds. */
///@}


/* Type Definitions ----------------------------------------------------------*/

/**
 * @struct pid_cfgType
 * @brief Configuration and state data for a PID controller.
 */
typedef struct
{
	float 	Kp;						/**< Proportional gain. */
	float 	Ki;						/**< Integral gain. */
	float 	Kd;						/**< Derivative gain. */
	float 	Ts;						/**< Sampling time in seconds. */
	int8_t 	Lower_Limit_Saturation;	/**< Lower saturation limit for the control signal. */
	int8_t 	Upper_Limit_Saturation;	/**< Upper saturation limit for the control signal. */
	float 	Integrator_Amount;		/**< Accumulator for the integral term. */
	float 	Differentiator_Amount;	/**< The derivative term. */
	int16_t Prev_Measurement;		/**< Previous measurement for derivative calculation. */
	float 	Prev_Error;				/**< Previous error for integral calculation. */
	int8_t 	Control_Signal;			/**< The final calculated control signal. */
	uint8_t Anti_windup_EN;			/**< Flag to enable/disable anti-windup. */
	float 	Wind_Up_Amount;			/**< Integral term value during saturation for anti-windup. */
	float 	Error;					/**< Current error (set_point - measurement). */
	int8_t 	dir;						/**< Direction of the motor. */
} pid_cfgType2;

typedef struct
{
	float 	Kp_r;						/**< Proportional gain right. */
	float 	Kp_l;						/**< Proportional gain left. */
	float 	Ki_r;						/**< Integral gain right. */
	float	Ki_l;						/**< Integral gain left. */
	float 	Kd_r;						/**< Derivative gain right. */
	float	Kd_l;						/**< Derivative gain left.*/
	float 	Ts;							/**< Sampling time in seconds. */
	int8_t 	Lower_Limit_Saturation;		/**< Lower saturation limit for the control signal. */
	int8_t 	Upper_Limit_Saturation;		/**< Upper saturation limit for the control signal. */
	float 	Integrator_Amount_r;		/**< Accumulator for the integral term right. */
	float	Integrator_Amount_l;		/**< Accumulator for the integral term left. */
	float 	Differentiator_Amount_r;	/**< The derivative term right. */
	float 	Differentiator_Amount_l;	/**< The derivative term left. */
	int16_t Prev_Measurement_r;			/**< Previous measurement for derivative calculation right. */
	int16_t Prev_Measurement_l;			/**< Previous measurement for derivative calculation left. */
	float 	Prev_Error_r;				/**< Previous error for integral calculation right. */
	float 	Prev_Error_l;				/**< Previous error for integral calculation left. */
	int8_t 	Control_Signal_r;			/**< The final calculated control signal right. */
	int8_t 	Control_Signal_l;			/**< The final calculated control signal left. */
	uint8_t Anti_windup_EN;				/**< Flag to enable/disable anti-windup. */
	float 	Wind_Up_Amount_r;			/**< Integral term value during saturation for anti-windup right. */
	float 	Wind_Up_Amount_l;			/**< Integral term value during saturation for anti-windup right. */
	float 	Error_r;					/**< Current error (set_point - measurement) right. */
	float  	Error_l;					/**< Current error (set_point - measurement) left. */
	int8_t 	dir;						/**< Direction of the motor. */
} pid_cfgType;

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief Initializes the PID controller with default values.
 * @param pid_cfg Pointer to the PID configuration structure.
 * @param AntiWindup Flag to enable or disable anti-windup (1 = enabled, 0 = disabled).
 */
void LRL_PID_Init(pid_cfgType *pid_cfg,uint8_t AntiWindup);

/**
 * @brief Updates the PID controller and calculates the new control signal.
 * @param pid_cfg Pointer to the PID configuration structure.
 * @param measurement The current measured value (e.g., speed).
 * @param set_point The desired set point.
 */
void LRL_PID_Update(pid_cfgType *pid, odom_cfgType *odom, int16_t left_setpoint, int16_t right_setpoint);

#endif /* INC_PID_H_ */
