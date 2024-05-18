///*
// * imu.h
// *
// *  Created on: Apr 7, 2024
// *      Author: arian
// */
//
#ifndef INC_IMU_H_
#define INC_IMU_H_
//
//

#include "main.h"
#include "i2c.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"

#define WHO_AM_I 0x75
#define INT_PIN_CFG 0x37
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define TEMP_OUT_H 0x41
#define GYRO_CONFIG 0x1B
#define GYRO_XOUT_H 0x43

#define MPU_ADDR 0xD0

// Complementary filter constants
#define ALPHA 0.9f // Weighting factor for gyroscope

typedef struct
{
//
	I2C_HandleTypeDef *hi2c;
//	define accelerometer parameters
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    double 	final_accel_x; // it is the accel x data with gain
	double 	final_accel_y; // it is the accel y data with gain
	double 	final_accel_z; // it is the accel z data with gain

	float filtered_x;
	float filtered_y;
	float filtered_z;
//	define gyroscope parameters
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	float 	final_gyro_x;
	float	final_gyro_y;
	float	final_gyro_z;
	uint8_t mf;
	uint8_t mf2;

	float roll;
	float pitch;
	float yaw;

	float x;
	float y;
	float z;

} imu_cfgType;

typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} kalman_cfgType;

void LRL_Read_Accel(imu_cfgType *imu);
void LRL_Read_Gyro(imu_cfgType *imu);
void LRL_MPU_Init(imu_cfgType *imu);
//void LRL_Kalman_Filter(imu_cfgType *imu);
void LRL_Complementary_Filter(imu_cfgType *imu);
void LRL_MPU_Bypass(imu_cfgType *imu);
//
////=========================================== Gy-80 ========================================
//
////////=========================================== Accelerometer ========================================
////
////// The address for gy80 is 7bit 0x1b + 0 for write and 1 for read
////#define ACCEL_ADDR_W 0xA6
////#define ACCEL_ADDR_R 0xA7
////
////#define POWER_CTL 0x2D
////#define DATA_FORMAT 0x31
////
////#define DEVID 0x00 // register to check if sensor is all right
////
//////=========================================== Gyroscope ========================================
////
////#define GYRO_ADDR_R 0xD3
////#define GYRO_ADDR_W 0xD2
////
////#define WHO_AM_I 0x0F // register to check if sensor is all righ
////
////#define CTRL_REG1 0x20
////#define CTRL_REG4 0x23
////#define OUT_X_L 0x28
////
////typedef struct
////{
//////
////	I2C_HandleTypeDef *hi2c;
//////	define accelerometer parameters
////    int16_t accel_x;
////    int16_t accel_y;
////    int16_t accel_z;
////    float 	final_accel_x; // it is the accel x data with gain
////	float 	final_accel_y; // it is the accel y data with gain
////	float 	final_accel_z; // it is the accel z data with gain
//////	define gyroscope parameters
////	int16_t gyro_r;
////	int16_t gyro_p;
////	int16_t gyro_y;
////	float 	final_gyro_r;
////	float	final_gyro_p;
////	float	final_gyro_y;
////	uint8_t mf;
////	uint8_t mf2;
////
////
//////    float temprature;
//////
//////    double kalmanFilter_X_Angle;
//////    double kalmanFilter_Y_Angle;
////} imu_cfgType;
////
////// Kalman structure
////typedef struct
////{
////    double Q_angle;
////    double Q_bias;
////    double R_measure;
////    double angle;
////    double bias;
////    double P[2][2];
////} kalman_cfgType;
////
////void LRL_IMU_Init(imu_cfgType *imu);
////void LRL_ACCEL_Read(imu_cfgType *imu);
////void LRL_GYRO_Read(imu_cfgType *imu);
////
////
#endif /* INC_IMU_H_ */
