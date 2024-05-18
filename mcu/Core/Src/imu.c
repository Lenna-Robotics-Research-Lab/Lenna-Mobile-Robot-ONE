///*
// * imu.c
// *
// *  Created on: Apr 7, 2024
// *      Author: arian
// */
//
#include <math.h>
#include "imu.h"
#include "main.h"
#include "mcu_layout.h"
#include "i2c.h"
#include "usart.h"

const double Accel_Z_corrector = 14418.0;

void LRL_MPU_Init(imu_cfgType *imu)
{
    uint8_t check;
    uint8_t Data;

    // check WHO_AM_I Reg to see the address being correct

    HAL_I2C_Mem_Read(imu->hi2c, MPU_ADDR, WHO_AM_I, 1, &check, 1, 10);

    if (check == 0x68) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0x00;
        HAL_I2C_Mem_Write(imu->hi2c, MPU_ADDR, PWR_MGMT_1, 1, &Data, 1, 10);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(imu->hi2c, MPU_ADDR, SMPLRT_DIV, 1, &Data, 1, 10);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(imu->hi2c, MPU_ADDR, ACCEL_CONFIG, 1, &Data, 1, 10);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(imu->hi2c, MPU_ADDR, GYRO_CONFIG, 1, &Data, 1, 10);

        HAL_GPIO_WritePin(BLINK_LED_PORT, BLINK_LED_PIN, 1);
    }
}


void LRL_Read_Accel(imu_cfgType *imu)
{
    uint8_t data[6];
    int16_t accel_data_raw[3];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(imu->hi2c, MPU_ADDR, ACCEL_XOUT_H, 1, data, 6,10);

    accel_data_raw[0] = (int16_t)(data[0] << 8 | data[1]);
    accel_data_raw[1] = (int16_t)(data[2] << 8 | data[3]);
    accel_data_raw[2] = (int16_t)(data[4] << 8 | data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    imu->final_accel_x = accel_data_raw[0] / 16384.0;
    imu->final_accel_y = accel_data_raw[1] / 16384.0;
    imu->final_accel_z = accel_data_raw[2] / Accel_Z_corrector;
}

void LRL_Read_Gyro(imu_cfgType *imu)
{
	uint8_t data[6];
	int16_t gyro_data_raw[3];

	HAL_I2C_Mem_Read(imu->hi2c, MPU_ADDR, GYRO_XOUT_H, 1, data, 6,100);

	gyro_data_raw[0] = (int16_t)(data[0] << 8 | data[1]);
	gyro_data_raw[1] = (int16_t)(data[2] << 8 | data[3]);
	gyro_data_raw[2] = (int16_t)(data[4] << 8 | data[5]);

	imu->final_gyro_x = gyro_data_raw[0] / 131.0;
	imu->final_gyro_y = gyro_data_raw[1] / 131.0;
	imu->final_gyro_z = gyro_data_raw[2] / 131.0;
}

void LRL_MPU_Read_All(imu_cfgType *imu)
{
	LRL_Read_Accel(imu);
	LRL_Read_Gyro(imu);
}

void LRL_MPU_Bypass(imu_cfgType *imu)
{
    uint8_t Data;

    Data = 0x00;
    HAL_I2C_Mem_Write(imu->hi2c, MPU_ADDR, USER_CTRL, 1, &Data, 1, 10);
    Data = 0x02;
    HAL_I2C_Mem_Write(imu->hi2c, MPU_ADDR, INT_PIN_CFG, 1, &Data, 1, 10);
    Data = 0x00;
    HAL_I2C_Mem_Write(imu->hi2c, MPU_ADDR, PWR_MGMT_1, 1, &Data, 1, 10);
}

void LRL_Complementary_Filter(imu_cfgType *imu)
{
    static float prev_gyr_x = 0.0f, prev_gyr_y = 0.0f, prev_gyr_z = 0.0f;
    static float prev_acc_x = 0.0f, prev_acc_y = 0.0f, prev_acc_z = 0.0f;

    float dt = 0.001;

    // Low-pass filter accelerometer data
    imu->filtered_x = ALPHA * prev_acc_x + (1 - ALPHA) * imu->final_accel_x;
    imu->filtered_y = ALPHA * prev_acc_y + (1 - ALPHA) * imu->final_accel_y;
    imu->filtered_z = ALPHA * prev_acc_z + (1 - ALPHA) * imu->final_accel_z;

    // Normalize accelerometer data
    float acc_norm = sqrtf(imu->filtered_x * imu->filtered_x + imu->filtered_y * imu->filtered_y + imu->filtered_z * imu->filtered_z);
    imu->filtered_x /= acc_norm;
    imu->filtered_y /= acc_norm;
    imu->filtered_z /= acc_norm;

    // Update angle using accelerometer
    float acc_angle_x = atan2f(imu->filtered_y, imu->filtered_z) * (180.0f / M_PI);
    float acc_angle_y = atan2f(imu->filtered_x, imu->filtered_z) * (180.0f / M_PI);
    float acc_angle_z = atan2f(imu->filtered_y, imu->filtered_x) * (180.0f / M_PI);

    // Low-pass filter gyroscope data
    float gyr_x_filtered = ALPHA * prev_gyr_x + (1 - ALPHA) * imu->final_gyro_x;
    float gyr_y_filtered = ALPHA * prev_gyr_y + (1 - ALPHA) * imu->final_gyro_y;
    float gyr_z_filtered = ALPHA * prev_gyr_z + (1 - ALPHA) * imu->final_gyro_z;

    // Update angle using gyroscope
    imu->roll = imu->roll + gyr_x_filtered * dt;
    imu->pitch = imu->pitch - gyr_y_filtered * dt;
    imu->yaw = imu->yaw + gyr_z_filtered * dt;

    // Complementary filter
    imu->roll = ALPHA * imu->roll + (1 - ALPHA) * acc_angle_x;
    imu->pitch = (ALPHA * imu->pitch) - (1 - ALPHA) * acc_angle_y;
    imu->yaw = ALPHA * imu->yaw + (1 - ALPHA) * acc_angle_z;

    // Update previous values
    prev_acc_x = imu->filtered_x;
    prev_acc_y = imu->filtered_y;
    prev_acc_z = imu->filtered_z;
    prev_gyr_x = gyr_x_filtered;
    prev_gyr_y = gyr_y_filtered;
    prev_gyr_z = gyr_z_filtered;
}
// Kalman filter variables

// Kalman filter variables
//float q_angle = 0.001f;     // Process noise covariance for angle
//float q_bias = 0.003f;      // Process noise covariance for bias
//float r_measure = 0.03f;    // Measurement noise covariance
//
//float angle_hat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // Estimated quaternion [q0, q1, q2, q3]
//float bias_hat[3] = {0.0f, 0.0f, 0.0f};        // Estimated bias [x, y, z]
//float p_angle[4][4] = {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}}; // Covariance matrix for angle
//float p_bias[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

//void LRL_Kalman_Filter(imu_cfgType *imu)
//{
//    float q_acc[3];
//    float q_gyr[3];
//    float q_acc_norm, q_gyr_norm;
//    float q_acc_gyr[4];
//    float q_acc_gyr_norm;
//    float q_angles[4];
//
//    // Auxiliary variables to avoid reapeated operations
//    float q_gyr_x = imu->final_gyro_x * 0.5f;
//    float q_gyr_y = imu->final_gyro_y * 0.5f;
//    float q_gyr_z = imu->final_gyro_z * 0.5f;
//
//    // Normalize accelerometer measurements
//    q_acc_norm = sqrtf(imu->final_accel_x * imu->final_accel_x + imu->final_accel_y * imu->final_accel_y + imu->final_accel_z * imu->final_accel_z);
//    q_acc[0] = imu->final_accel_x / q_acc_norm;
//    q_acc[1] = imu->final_accel_y / q_acc_norm;
//    q_acc[2] = imu->final_accel_z / q_acc_norm;
//
//    // Compute quaternion from accelerometer and gyroscope data
//    q_acc_gyr[0] = q_acc[0] * bias_hat[2] - q_acc[1] * bias_hat[1] + q_acc[2] * bias_hat[0];
//    q_acc_gyr[1] = q_acc[0] * bias_hat[1] + q_acc[1] * bias_hat[2] - q_acc[2] * 0.0f; // Changed bias_hat[3] to 0.0f
//    q_acc_gyr[2] = -q_acc[0] * bias_hat[0] + q_acc[1] * 0.0f + q_acc[2] * bias_hat[2]; // Changed bias_hat[3] to 0.0f
//    q_acc_gyr[3] = sqrtf(1.0f - q_acc_gyr[0] * q_acc_gyr[0] - q_acc_gyr[1] * q_acc_gyr[1] - q_acc_gyr[2] * q_acc_gyr[2]);
//
//    q_acc_gyr_norm = sqrtf(q_acc_gyr[0] * q_acc_gyr[0] + q_acc_gyr[1] * q_acc_gyr[1] + q_acc_gyr[2] * q_acc_gyr[2] + q_acc_gyr[3] * q_acc_gyr[3]);
//    q_acc_gyr[0] /= q_acc_gyr_norm;
//    q_acc_gyr[1] /= q_acc_gyr_norm;
//    q_acc_gyr[2] /= q_acc_gyr_norm;
//    q_acc_gyr[3] /= q_acc_gyr_norm;
//
//    // Normalize gyroscope measurements
//    q_gyr_norm = sqrtf(imu->final_gyro_x * imu->final_gyro_x + imu->final_gyro_y * imu->final_gyro_y + imu->final_gyro_z * imu->final_gyro_z);
//    q_gyr[0] = -q_gyr_x / q_gyr_norm;
//    q_gyr[1] = -q_gyr_y / q_gyr_norm;
//    q_gyr[2] = -q_gyr_z / q_gyr_norm;
//
//    // Compute quaternion product
//    q_angles[0] = q_acc_gyr[0] * angle_hat[0] - q_acc_gyr[1] * angle_hat[1] - q_acc_gyr[2] * angle_hat[2] - q_acc_gyr[3] * angle_hat[3];
//    q_angles[1] = q_acc_gyr[0] * angle_hat[1] + q_acc_gyr[1] * angle_hat[0] + q_acc_gyr[2] * angle_hat[3] - q_acc_gyr[3] * angle_hat[2];
//    q_angles[2] = q_acc_gyr[0] * angle_hat[2] - q_acc_gyr[1] * angle_hat[3] + q_acc_gyr[2] * angle_hat[0] + q_acc_gyr[3] * angle_hat[1];
//    q_angles[3] = q_acc_gyr[0] * angle_hat[3] + q_acc_gyr[1] * angle_hat[2] - q_acc_gyr[2] * angle_hat[1] + q_acc_gyr[3] * angle_hat[0];
//
//    // Update angle estimates
//    angle_hat[0] += q_angles[0] * q_angle - q_gyr[0];
//    angle_hat[1] += q_angles[1] * q_angle - q_gyr[1];
//    angle_hat[2] += q_angles[2] * q_angle - q_gyr[2];
//    angle_hat[3] += q_angles[3] * q_angle;
//
//    // Normalize the quaternion
//    float angle_hat_norm = sqrtf(angle_hat[0] * angle_hat[0] + angle_hat[1] * angle_hat[1] + angle_hat[2] * angle_hat[2] + angle_hat[3] * angle_hat[3]);
//    angle_hat[0] /= angle_hat_norm;
//    angle_hat[1] /= angle_hat_norm;
//    angle_hat[2] /= angle_hat_norm;
//    angle_hat[3] /= angle_hat_norm;
//
//    // Update bias estimates
//    bias_hat[0] += q_bias * (q_acc[0] - q_angles[0]);
//    bias_hat[1] += q_bias * (q_acc[1] - q_angles[1]);
//    bias_hat[2] += q_bias * (q_acc[2] - q_angles[2]);
//
//    // Calculate roll, pitch, and yaw angles
//    imu->roll = atan2f(2.0f * (angle_hat[0] * angle_hat[1] + angle_hat[2] * angle_hat[3]), 1.0f - 2.0f * (angle_hat[1] * angle_hat[1] + angle_hat[2] * angle_hat[2])) * 180.0f / M_PI;
//    imu->pitch = asinf(2.0f * (angle_hat[0] * angle_hat[2] - angle_hat[3] * angle_hat[1])) * 180.0f / M_PI;
//    imu->yaw = atan2f(2.0f * (angle_hat[0] * angle_hat[3] + angle_hat[1] * angle_hat[2]), 1.0f - 2.0f * (angle_hat[2] * angle_hat[2] + angle_hat[3] * angle_hat[3])) * 180.0f / M_PI;
//
//    // Calculate linear acceleration
//    float q0q0 = q_acc_gyr[0] * q_acc_gyr[0];
//    float q0q1 = q_acc_gyr[0] * q_acc_gyr[1];
//    float q0q2 = q_acc_gyr[0] * q_acc_gyr[2];
//    float q0q3 = q_acc_gyr[0] * q_acc_gyr[3];
//    float q1q1 = q_acc_gyr[1] * q_acc_gyr[1];
//    float q1q2 = q_acc_gyr[1] * q_acc_gyr[2];
//    float q1q3 = q_acc_gyr[1] * q_acc_gyr[3];
//    float q2q2 = q_acc_gyr[2] * q_acc_gyr[2];
//    float q2q3 = q_acc_gyr[2] * q_acc_gyr[3];
//    float q3q3 = q_acc_gyr[3] * q_acc_gyr[3];
//
//    imu->x = (imu->final_accel_x - bias_hat[0]) / (q0q0 + q1q1 - q2q2 - q3q3);
//    imu->y = (imu->final_accel_y - bias_hat[1]) / (q0q0 - q1q1 + q2q2 - q3q3);
//    imu->z = (imu->final_accel_z - bias_hat[2]) / (q0q0 - q1q1 - q2q2 + q3q3);
//
//}
//

////=========================================== GY-80 ========================================
////
////// Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV).
////// Where Gyroscope Output Rate is 8KHz, To get the sample rate of 1KHz, we need to use the SMPLRT_DIV as ‘7’
////
////// Setup MPU6050
////
////const uint16_t i2c_timeout = 100;
////const double Accel_Z_corrector = 14418.0;
////uint8_t Data;
////
////
////uint32_t timer;
////
////kalman_cfgType KalmanX = {
////    .Q_angle = 0.001f,
////    .Q_bias = 0.003f,
////    .R_measure = 0.03f};
////
////kalman_cfgType KalmanY = {
////    .Q_angle = 0.001f,
////    .Q_bias = 0.003f,
////    .R_measure = 0.03f,
////};
////
////// ####################   Initialization of the GY-80   ####################
////
////void LRL_IMU_Init(imu_cfgType *imu)
////{
////	uint8_t data;
////	uint8_t check;
////
////	//check if device is all right
////	HAL_I2C_Mem_Read(imu->hi2c, ACCEL_ADDR_R, DEVID, 1, &check, 1, 100);
////	if(check == 0xE5)
////	{
////		data = 0x00;// first reset all the bits
////		HAL_I2C_Mem_Write(imu->hi2c, ACCEL_ADDR_W, POWER_CTL, 1, &data, 1, 100);
////
////		data = 16;// then set the sequence to wake up
////		HAL_I2C_Mem_Write(imu->hi2c, ACCEL_ADDR_W, POWER_CTL, 1, &data, 1, 100);
////
////		data = 0x08;// then set the sequence to wake up
////		HAL_I2C_Mem_Write(imu->hi2c, ACCEL_ADDR_W, POWER_CTL, 1, &data, 1, 100);
////
////		data = 0x01;
////		HAL_I2C_Mem_Write(imu->hi2c, ACCEL_ADDR_W, DATA_FORMAT, 1, &data, 1, 100);
////		//HAL_GPIO_WritePin(BLINK_LED_PORT, BLINK_LED_PIN, 1); // this line is to check the if data is all right
////	}
////	else
////	{
////		// !!!!!!! this will cast error in the future versions
////	}
////	HAL_I2C_Mem_Read(imu->hi2c, GYRO_ADDR_R , WHO_AM_I, 1, &check, 1, 100);
////
////	// gyro initialization
////	if(check == 0xD3)
////	{
////		// DR & BW 1010 equals to 400 Hz and 50 cut off PD = 1 and all axis are enabled with 1
////
////		data = 0x6F;
////		HAL_I2C_Mem_Write(imu->hi2c, GYRO_ADDR_W, CTRL_REG1, 1, &data, 1, 10);
////
////
////		data = 0x00;
////		HAL_I2C_Mem_Write(imu->hi2c, GYRO_ADDR_W, CTRL_REG4, 1, &data, 1, 10);
////
////		HAL_GPIO_WritePin(BLINK_LED_PORT, BLINK_LED_PIN, 1);
////	}
////	else
////	{
////		// !!!!!!! this will cast error in the future
////	}
////}
////
////
////void LRL_ACCEL_Read(imu_cfgType *imu)
////{
////	uint8_t data[6];
////	HAL_I2C_Mem_Read(imu->hi2c, ACCEL_ADDR_R, 0x32, 1, data, 6,10);
////	imu->accel_x = ((data[1]<<8)|data[0]);
////	imu->accel_y = ((data[3]<<8)|data[2]);
////	imu->accel_z = ((data[5]<<8)|data[4]);
////
////	imu->final_accel_x = imu->accel_x * 0.00376390;
////	imu->final_accel_y = imu->accel_y * 0.00376009;
////	imu->final_accel_z = imu->accel_z * 0.00349265;
////}
////
////void LRL_GYRO_Read(imu_cfgType *imu)
////{
////	uint8_t data[6];
////	uint8_t reg = 0xD3;
//////	HAL_I2C_Master_Transmit(imu->hi2c, 0xD2 , &reg ,1,10);
////	HAL_I2C_Mem_Read(imu->hi2c, 0xD3 , 0x28, 1, data, 6,10);
////	imu->gyro_r = (data[1]<<8)|data[0];
////	imu->gyro_p = (data[3]<<8)|data[2];
////	imu->gyro_y = (data[5]<<8)|data[4];
////
//////	HAL_GPIO_WritePin(BLINK_LED_PORT, BLINK_LED_PIN, 1);
////}
////
////
////
