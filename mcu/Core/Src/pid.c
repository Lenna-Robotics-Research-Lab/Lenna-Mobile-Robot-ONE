/*
 *  Author: Lenna Robotics Research Laboratory
 *  	Autonomous Systems Research Branch
 *		Iran University of Science and Technology
 *	GitHub:	github.com/Lenna-Robotics-Research-Lab
 */

#include <mcu_config.h>
#include "pid.h"
#include "main.h"


void LRL_PID_Init(pid_cfgType *pid_cfg,uint8_t AntiWindup)
	{
	// Resetting the PID parameters
	pid_cfg->Anti_windup_EN = AntiWindup;
	pid_cfg->Prev_Measurement = 0.0f;
	pid_cfg->Integrator_Amount = 0;
	pid_cfg->Prev_Error = 0.0f;// initial error you can change it if by default you have an error
	pid_cfg->Control_Signal = 0;
	}

void LRL_PID_Update(pid_cfgType *pid_cfg, int16_t measurement, int16_t set_point)
	{
	int8_t _dir;

	_dir = set_point / abs(set_point);

	measurement = abs(measurement);
	set_point = abs(set_point);

	pid_cfg->Error = set_point - measurement;
	pid_cfg->Error = pid_cfg->Error * Speed2PWM_Rate;


	// Setting Values
//	float P = pid_cfg->Kp * pid_cfg->Error;
	pid_cfg->Integrator_Amount += (pid_cfg->Ts*(pid_cfg->Ki * (pid_cfg->Error + pid_cfg->Prev_Error)));
//	float I = pid_cfg->Integrator_Amount;
	pid_cfg->Differentiator_Amount = 0;//pid_cfg->Kd * (measurement - pid_cfg->Prev_Measurement)/(pid_cfg->Ts);
    /*
    // another way to use derivative term used by phils lab channel
   pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);
     */


	pid_cfg->Control_Signal = (pid_cfg->Kp * pid_cfg->Error) + pid_cfg->Integrator_Amount + pid_cfg->Differentiator_Amount;

	if(pid_cfg->Anti_windup_EN == 1)
	{

		if(pid_cfg->Control_Signal <= Upper_Saturation_Limit)
			{
			//pid_cfg->Integrator_Amount += (pid_cfg->Ts*(pid_cfg->Ki * (pid_cfg->Error + pid_cfg->Prev_Error)));
			pid_cfg->Wind_Up_Amount = pid_cfg->Integrator_Amount;
			}
		else
			{
			pid_cfg->Control_Signal = (pid_cfg->Kp * pid_cfg->Error) + pid_cfg->Wind_Up_Amount + pid_cfg->Differentiator_Amount;
			}
		}


	//pid_cfg->Control_Signal = (pid_cfg->Kp * pid_cfg->Error) + pid_cfg->Integrator_Amount + pid_cfg->Differentiator_Amount;

	if(pid_cfg->Control_Signal > pid_cfg->Upper_Limit_Saturation)
	  {
		pid_cfg->Control_Signal = pid_cfg->Upper_Limit_Saturation;
	  }
	else if(pid_cfg->Control_Signal < pid_cfg->Lower_Limit_Saturation)
	  {
		pid_cfg->Control_Signal = pid_cfg->Lower_Limit_Saturation;
	  }

	pid_cfg->Control_Signal = pid_cfg->Control_Signal * _dir;
	pid_cfg->Prev_Measurement = measurement;
	pid_cfg->Prev_Error = pid_cfg->Error;


	}
