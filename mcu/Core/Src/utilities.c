/*
 * utilities.c
 *
 *  Created on: Oct 18, 2023
 *      Author: Lenna Robotics Research Laboratory
 *      		Autonomous Systems Research Branch
 *				Iran University of Science and Technology
 *		GitHub:	github.com/Lenna-Robotics-Research-Lab
 */

#include "utilities.h"

void LRL_Delay_Init()
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	DELAY_TIM_HANDLE.Instance = DELAY_TIM_INISTANCE;
	DELAY_TIM_HANDLE.Init.Prescaler = (HAL_RCC_GetHCLKFreq() / 1000000)-1;
	DELAY_TIM_HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
	DELAY_TIM_HANDLE.Init.Period = 65535;
	DELAY_TIM_HANDLE.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	DELAY_TIM_HANDLE.Init.RepetitionCounter = 0;
	DELAY_TIM_HANDLE.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&DELAY_TIM_HANDLE) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&DELAY_TIM_HANDLE, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&DELAY_TIM_HANDLE, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void LRL_Delay_Us(volatile uint16_t delay_us)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < delay_us);
}
