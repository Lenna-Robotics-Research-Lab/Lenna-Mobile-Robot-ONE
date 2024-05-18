///*
// * mpu6050.h
// *
// *  Created on: Apr 13, 2024
// *      Author: arian
// */
//
//#ifndef INC_MPU6050_H_
//#define INC_MPU6050_H_
//
//
//
//#endif /* INC_MPU6050_H_ */
//
//#include <stdint.h>
//#include "i2c.h"
//
//#define RAD_TO_DEG 57.295779513082320876798154814105
//
//
//
//// Setup MPU6050
//
//
//// MPU6050 structure
//typedef struct
//{
//
//    int16_t Accel_X_RAW;
//    int16_t Accel_Y_RAW;
//    int16_t Accel_Z_RAW;
//    double Ax;
//    double Ay;
//    double Az;
//
//    int16_t Gyro_X_RAW;
//    int16_t Gyro_Y_RAW;
//    int16_t Gyro_Z_RAW;
//    double Gx;
//    double Gy;
//    double Gz;
//
//    float Temperature;
//
//    double KalmanAngleX;
//    double KalmanAngleY;
//} MPU6050_t;
//
//// Kalman structure
//typedef struct
//{
//    double Q_angle;
//    double Q_bias;
//    double R_measure;
//    double angle;
//    double bias;
//    double P[2][2];
//} Kalman_t;
//
//void MPU6050_Init(I2C_HandleTypeDef *I2Cx);
//
//void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
//
//void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
//
//void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
//
//void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
//
//double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
