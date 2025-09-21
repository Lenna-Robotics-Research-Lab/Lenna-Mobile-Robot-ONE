/*
 * imu.h
 *
 *  Created on: Sep 21, 2025
 *      Author: arian
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "i2c.h"
#include "tim.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "motion.h"

// #######################################################
// ######################  Structs 	######################
// #######################################################
typedef struct
{
	float	x;
	float	y;
	float	z;
} linear_position;

typedef struct
{
	int16_t	x; // roll
	int16_t	y; // pitch
	int16_t	z; // yaw
} angular_position;

typedef struct
{
	int16_t	x;
	int16_t	y;
	int16_t	z;

	int16_t	x_calibrated;
	int16_t	y_calibrated;
	int16_t	z_calibrated;
} accelerometer;

typedef struct
{
	int16_t	x;
	int16_t	y;
	int16_t	z;

	int16_t	x_calibrated;
	int16_t	y_calibrated;
	int16_t	z_calibrated;
} gyroscope;

typedef struct
{
	float	x;
	float	y;
	float	z;
	int16_t	heading;

	int16_t offset_heading;
} magnetometer;


typedef struct
{
	I2C_HandleTypeDef * hi2c;	// I2C Sensors e.g.  IMU, Magnetometer, etc.

	int16_t offset_gyro_x;
	int16_t offset_gyro_y;
	int16_t offset_gyro_z;

	int16_t offset_accel_x;
	int16_t offset_accel_y;
	int16_t offset_accel_z;

	float 	offset_calibration_x;
	float 	offset_calibration_y;
	float 	offset_calibration_z;

	float 	roll_temp;
	float 	pitch_temp;
	float	yaw_temp;
} imu_cfgType;

typedef struct
{
	imu_cfgType 		imu;
	diffDrive_cfgType	diff_robot;

	linear_position 	pose;
	angular_position 	angle;
	accelerometer 		accel;
	gyroscope 			gyro;
	magnetometer 		mag;

} imu_statetype;

// #################################################################
// ####################  MAGNETOMETER HMC5883L  ####################
// #################################################################
#define HMC5883L_DEFAULT_ADDRESS    0x1E
#define HMC5883L_ADDRESS            HMC5883L_DEFAULT_ADDRESS << 1
#define HMC5883L_ADDRESS_WRITE      0x3C
#define HMC5883L_ADDRESS_READ       0x3D

// Register Address List
#define HMC5883L_RA_CONFIG_A        0x00
#define HMC5883L_RA_CONFIG_B        0x01
#define HMC5883L_RA_MODE            0x02
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAX_L         0x04
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAZ_L         0x06
#define HMC5883L_RA_DATAY_H         0x07
#define HMC5883L_RA_DATAY_L         0x08
#define HMC5883L_RA_STATUS          0x09
#define HMC5883L_RA_ID_A            0x0A
#define HMC5883L_RA_ID_B            0x0B
#define HMC5883L_RA_ID_C            0x0C

#define HMC5883L_AVERAGING_1        0x00
#define HMC5883L_AVERAGING_2        0x01
#define HMC5883L_AVERAGING_4        0x02
#define HMC5883L_AVERAGING_8        0x03

#define HMC5883L_RATE_0P75          0x00
#define HMC5883L_RATE_1P5           0x01
#define HMC5883L_RATE_3             0x02
#define HMC5883L_RATE_7P5           0x03
#define HMC5883L_RATE_15            0x04
#define HMC5883L_RATE_30            0x05
#define HMC5883L_RATE_75            0x06

#define HMC5883L_BIAS_NORMAL        0x00
#define HMC5883L_BIAS_POSITIVE      0x01
#define HMC5883L_BIAS_NEGATIVE      0x02

#define HMC5883L_GAIN_1370          0x00
#define HMC5883L_GAIN_1090          0x01
#define HMC5883L_GAIN_820           0x02
#define HMC5883L_GAIN_660           0x03
#define HMC5883L_GAIN_440           0x04
#define HMC5883L_GAIN_390           0x05
#define HMC5883L_GAIN_330           0x06
#define HMC5883L_GAIN_220           0x07

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

#define MAGNETIC_DECLINATION		0.0881


// #######################################################
// ####################  IMU MPU6050  ####################
// #######################################################
#define WHO_AM_I 		0x75
#define INT_PIN_CFG 	0x37
#define USER_CTRL 		0x6A
#define PWR_MGMT_1 		0x6B
#define SMPLRT_DIV 		0x19
#define ACCEL_CONFIG 	0x1C
#define ACCEL_XOUT_H 	0x3B
#define TEMP_OUT_H 		0x41
#define GYRO_CONFIG 	0x1B
#define GYRO_XOUT_H 	0x43

#define MPU_ADDR 		0xD0

#define ACCEL_X_CORRECTOR 16384
#define ACCEL_Y_CORRECTOR 16384
#define ACCEL_Z_CORRECTOR 14418

#define GYRO_CORRECTOR 	131.0

// Complementary filter constants
#define ALPHA 			0.9f

#define DELAY_TIMEOUT	10
#define FLOAT_SCALING	1000


// ##############################################################
// ####################  FUNCTION PROTOTYPE  ####################
// ##############################################################

float LRL_IMU_MagSetDeclination(int16_t declination_degs , int16_t declination_mins, char declination_dir);
void LRL_IMU_MagInit(imu_statetype * imu);
void LRL_IMU_MagReadHeading(imu_statetype * imu);

void LRL_IMU_MPUInit(imu_statetype * imu);
void LRL_IMU_ReadAccel(imu_statetype * imu);
void _LRL_IMU_MPUBypassEn(imu_statetype * imu, uint8_t enable);
void LRL_IMU_ReadGyro(imu_statetype *imu);
void LRL_IMU_MPUReadAll(imu_statetype *imu);
void LRL_IMU_ComplementaryFilter(imu_statetype *imu, float dt);

#endif /* INC_IMU_H_ */
