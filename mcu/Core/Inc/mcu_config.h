/*
 * mcu_layout.h
 *
 *  Created on: Oct 11, 2023
 *      Author: Lenna Robotics Research Laboratory
 *      		Autonomous Systems Research Branch
 *				Iran University of Science and Technology
 *		GitHub:	github.com/Lenna-Robotics-Research-Lab
 */

#define BUZZER_PORT 	GPIOA
#define Buzzer_PIN 		GPIO_PIN_4

#define BLINK_LED_PORT 	GPIOD
#define BLINK_LED_PIN 	GPIO_PIN_10

//=========================================== BAT ADC =========================================

#define BAT_ADC_PORT 	GPIOA
#define BAT_ADC_PIN 	GPIO_PIN_5

//=========================================== MOTOR ===========================================

#define MOTOR1_EN_PORT 	GPIOC
#define MOTOR1_EN_PIN 	GPIO_PIN_6
#define MOTOR2_EN_PORT 	GPIOC
#define MOTOR2_EN_PIN 	GPIO_PIN_7

#define MOTOR_PORT 		GPIOD
#define MOTOR1_A_PIN 	GPIO_PIN_4
#define MOTOR1_B_PIN 	GPIO_PIN_3
#define MOTOR2_A_PIN 	GPIO_PIN_1
#define MOTOR2_B_PIN 	GPIO_PIN_0

//=========================================== ENCODER ===========================================

#define ENCODER1_A_PORT 	GPIOB
#define ENCODER1_B_PORT 	GPIOB
#define ENCODER2_B_PORT 	GPIOB
#define ENCODER2_A_PORT 	GPIOA
#define ENCODER1_A_PIN 		GPIO_PIN_4
#define ENCODER1_B_PIN 		GPIO_PIN_5
#define ENCODER2_A_PIN 		GPIO_PIN_15
#define ENCODER2_B_PIN 		GPIO_PIN_3

#define ENCODER_CYCLE 		48960
#define ENCODER1_COUNTER 	TIM3->CNT
#define ENCODER2_COUNTER 	TIM2->CNT
#define MOTOR1_PWM 			TIM8->CCR1
#define MOTOR2_PWM 			TIM8->CCR2

//=========================================== UltraSonic ========================================

#define US1_TRIG_PORT		GPIOE
#define US2_TRIG_PORT		GPIOE
#define US3_TRIG_PORT		GPIOE
#define US4_TRIG_PORT		GPIOE

#define US1_TRIG_PIN		GPIO_PIN_0
#define US2_TRIG_PIN		GPIO_PIN_1
#define US3_TRIG_PIN		GPIO_PIN_3
#define US4_TRIG_PIN		GPIO_PIN_4

#define ULTRASONIC_UNITS	4

