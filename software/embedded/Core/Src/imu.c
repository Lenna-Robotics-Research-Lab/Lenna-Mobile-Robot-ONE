/**
 * @file imu.c
 * @brief This file contains functions for interfacing with the MPU-6050 IMU and HMC5883L magnetometer.
 *
 * It provides functionality for sensor initialization, data reading, and
 * sensor fusion using a complementary filter to determine device orientation.
 *
 * @author Lenna Robotics Research Laboratory, Autonomous Systems Research Branch, Iran University of Science and Technology
 * @date September 21, 2025
 * @version 1.0
 * @link https://github.com/Lenna-Robotics-Research-Lab
 */

#include "imu.h"
#include "math.h"
#include "mcu_config.h"
#include "stdio.h"

uint8_t i2c_reg_data;
uint8_t imu_buffer[6];
static float prev_gyr_x = 0.0f, prev_gyr_y = 0.0f, prev_gyr_z = 0.0f;
static float prev_acc_x = 0.0f, prev_acc_y = 0.0f, prev_acc_z = 0.0f;


/*
 ******************************************************************************
 * #################### HMC5883L MAGNETOMETER FUNCTIONS ####################
 ******************************************************************************
 */

/**
 * @brief Initializes the HMC5883L magnetometer.
 * @param imu Pointer to the IMU state structure.
 *
 * This function configures the magnetometer's registers, enables I2C bypass mode on the MPU-6050,
 * and performs a simple calibration by taking 100 readings to determine the initial heading offset.
 */
void LRL_IMU_MagInit(imu_statetype * imu)
{
	uint16_t _tmp_cal_mag = 0;

	// Enable I2C Bypass mode to access the magnetometer.
	_LRL_IMU_MPUBypassEn(imu, 1);
	// Write to CONFIG_A register (0x10 = 8-sample averaging, 15 Hz data output rate).
	HAL_I2C_Master_Transmit(imu->imu.hi2c, HMC5883L_ADDRESS, (uint8_t *)HMC5883L_ADDRESS_WRITE, 1, DELAY_TIMEOUT);
	i2c_reg_data = 0x10;
	HAL_I2C_Mem_Write(imu->imu.hi2c, HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_A, 1, &i2c_reg_data, 1, DELAY_TIMEOUT);

	// Write to CONFIG_B register (0xE0 = Gain of 1090 LSB/Gauss).
	HAL_I2C_Master_Transmit(imu->imu.hi2c, HMC5883L_ADDRESS, (uint8_t *)HMC5883L_ADDRESS_WRITE, 1, DELAY_TIMEOUT);
	i2c_reg_data = 0xE0;
	HAL_I2C_Mem_Write(imu->imu.hi2c, HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_B, 1, &i2c_reg_data, 1, DELAY_TIMEOUT);

	// Write to MODE register (HMC5883L_MODE_SINGLE = Single-measurement mode).
	HAL_I2C_Master_Transmit(imu->imu.hi2c, HMC5883L_ADDRESS, (uint8_t *)HMC5883L_ADDRESS_WRITE, 1, DELAY_TIMEOUT);
	HAL_I2C_Mem_Write(imu->imu.hi2c, HMC5883L_ADDRESS, HMC5883L_RA_MODE, 1, (uint8_t *)HMC5883L_MODE_SINGLE, 1, DELAY_TIMEOUT);

	HAL_Delay(10);

	// Calibrate heading offset by taking an average of 100 readings.
	imu->mag.offset_heading = 0;
	for(int i = 0; i<100; i++)
	{
		LRL_IMU_MagReadHeading(imu);
		_tmp_cal_mag += imu->mag.heading;
		HAL_Delay(10);
	}
	imu->mag.offset_heading = (_tmp_cal_mag / 100);
}

/**
 * @brief Sets the magnetic declination angle for the magnetometer.
 * @param declination_degs Degrees of declination.
 * @param declination_mins Minutes of declination.
 * @param declination_dir Direction of declination ('E' for East, 'W' for West).
 * @return The declination angle in radians.
 *
 * This function converts the given degrees and minutes of declination into a single
 * value in radians, which is used to correct the magnetic heading to true north.
 */
float LRL_IMU_MagSetDeclination(int16_t declination_degs , int16_t declination_mins, char declination_dir)
{
	int8_t _dir = 0;

	if (declination_dir == 'E' || declination_dir == 'e' || declination_dir == 1)
	{
		_dir = 1;
	}
	else if (declination_dir == 'W' || declination_dir == 'w' || declination_dir == -1)
	{
		_dir = -1;
	}

	return ((_dir)* ( declination_degs + (1/60 * declination_mins)) * (M_PI / 180));
}

/**
 * @brief Reads the magnetic heading from the HMC5883L sensor.
 * @param imu Pointer to the IMU state structure.
 *
 * This function reads the raw magnetic data, calculates the heading using atan2, and applies
 * both the magnetic declination and the pre-calibrated heading offset.
 */
void LRL_IMU_MagReadHeading(imu_statetype * imu)
{
	uint8_t _mag_buffer[6];
	float _mag_heading_temp;

	// Read 6 bytes of data starting from the X-axis high byte register.
	_LRL_IMU_MPUBypassEn(imu, 1);
	HAL_I2C_Master_Transmit(imu->imu.hi2c, HMC5883L_ADDRESS, (uint8_t *)HMC5883L_ADDRESS_READ, 1, DELAY_TIMEOUT);
	HAL_I2C_Mem_Read(imu->imu.hi2c, HMC5883L_ADDRESS, HMC5883L_RA_DATAX_H, 1, (uint8_t *)&_mag_buffer, 6, DELAY_TIMEOUT);

	// Combine the high and low bytes to get the full 16-bit values.
	// The HMC5883L data sheet shows the data in X, Z, Y order.
	imu->mag.x = (int16_t)((_mag_buffer[0] << 8) | _mag_buffer[1]);
	imu->mag.y = (int16_t)((_mag_buffer[4] << 8) | _mag_buffer[5]);
	imu->mag.z = (int16_t)((_mag_buffer[2] << 8) | _mag_buffer[3]);

	// Calculate heading using atan2 and correct for magnetic declination.
	_mag_heading_temp = atan2(imu->mag.x, imu->mag.y) + MAGNETIC_DECLINATION;

	// Convert radians to degrees.
	imu->mag.heading = _mag_heading_temp * 180/M_PI;

	// Ensure the heading is within the 0-360 degree range.
	if(imu->mag.heading < 0)
	imu->mag.heading += 360;

	if(imu->mag.heading > 360)
	imu->mag.heading -= 360;

	// Subtract the calibrated offset.
	imu->mag.heading -= imu->mag.offset_heading;
	if(imu->mag.heading < 0)
	imu->mag.heading += 360;
}

/*
 ******************************************************************************
 * #################### MPU-6050 IMU FUNCTIONS ####################
 ******************************************************************************
 */

/**
 * @brief Initializes the MPU-6050 IMU.
 * @param imu Pointer to the IMU state structure.
 *
 * This function configures the MPU-6050's power management, data rate, and
 * full-scale ranges for the accelerometer and gyroscope. It then performs a
 * calibration routine to find and store the zero-rate offsets.
 */
void LRL_IMU_MPUInit(imu_statetype * imu)
{
	float _tmp_cal_cf_x=0, _tmp_cal_cf_y=0, _tmp_cal_cf_z=0;
	float _tmp_cal_gy_x=0, _tmp_cal_gy_y=0, _tmp_cal_gy_z=0;
	float _tmp_cal_ac_x=0, _tmp_cal_ac_y=0, _tmp_cal_ac_z=0;
	uint8_t _imu_addr_check;

	// Disable I2C bypass mode to configure the MPU-6050.
	_LRL_IMU_MPUBypassEn(imu, 0);

	// Check if the MPU-6050 is available by reading the WHO_AM_I register.
	HAL_I2C_Mem_Read(imu->imu.hi2c, MPU_ADDR, WHO_AM_I, 1, &_imu_addr_check, 1, DELAY_TIMEOUT);

	if (_imu_addr_check == 0x68)
	{
		// Wake up the sensor by writing all 0s to the power management register.
		i2c_reg_data = 0x00;
		HAL_I2C_Mem_Write(imu->imu.hi2c, MPU_ADDR, PWR_MGMT_1, 1, &i2c_reg_data, 1, DELAY_TIMEOUT);

		// Set data rate to 1KHz by writing to SMPLRT_DIV register.
		i2c_reg_data = 0x07;
		HAL_I2C_Mem_Write(imu->imu.hi2c, MPU_ADDR, SMPLRT_DIV, 1, &i2c_reg_data, 1, DELAY_TIMEOUT);

		// Set accelerometer full-scale range to +/- 2g.
		i2c_reg_data = 0x00;
		HAL_I2C_Mem_Write(imu->imu.hi2c, MPU_ADDR, ACCEL_CONFIG, 1, &i2c_reg_data, 1, DELAY_TIMEOUT);

		// Set gyroscope full-scale range to +/- 250 deg/s.
		i2c_reg_data = 0x00;
		HAL_I2C_Mem_Write(imu->imu.hi2c, MPU_ADDR, GYRO_CONFIG, 1, &i2c_reg_data, 1, DELAY_TIMEOUT);
	}

	// Calibrate gyroscope and accelerometer offsets by taking 500 samples.
	imu->imu.offset_gyro_x = 0;
	imu->imu.offset_gyro_y = 0;
	imu->imu.offset_gyro_z = 0;

	imu->imu.offset_accel_x = 0;
	imu->imu.offset_accel_y = 0;
	imu->imu.offset_accel_z = 0;

	for(int i = 0; i<500; i++)
	{
		LRL_IMU_MPUReadAll(imu);
		_tmp_cal_gy_x += imu->gyro.x;
		_tmp_cal_gy_y += imu->gyro.y;
		_tmp_cal_gy_z += imu->gyro.z;

		_tmp_cal_ac_x += imu->accel.x;
		_tmp_cal_ac_y += imu->accel.y;
		_tmp_cal_ac_z += imu->accel.z;

		HAL_Delay(5);
	}

	imu->imu.offset_accel_x = (_tmp_cal_ac_x/500);
	imu->imu.offset_accel_y = (_tmp_cal_ac_y/500);
	imu->imu.offset_accel_z = (_tmp_cal_ac_z/500);

	imu->imu.offset_gyro_x = (_tmp_cal_gy_x/500);
	imu->imu.offset_gyro_y = (_tmp_cal_gy_y/500);
	imu->imu.offset_gyro_z = (_tmp_cal_gy_z/500);

	// Calibrate complementary filter offsets by taking 500 samples.
	imu->imu.offset_calibration_x = 0;
	imu->imu.offset_calibration_y = 0;
	imu->imu.offset_calibration_z = 0;

	for(int i = 0; i<500 ; i++)
	{
		LRL_IMU_MPUReadAll(imu);
		LRL_IMU_ComplementaryFilter(imu,0.01);

		_tmp_cal_cf_x += imu->angle.x;
		_tmp_cal_cf_y += imu->angle.y;
		_tmp_cal_cf_z += imu->angle.z;

		HAL_Delay(10);
	}

	imu->imu.offset_calibration_x = (_tmp_cal_cf_x/500);
	imu->imu.offset_calibration_y = (_tmp_cal_cf_y/500);
	imu->imu.offset_calibration_z = (_tmp_cal_cf_z/500);

	// Reset previous values for the filter to start clean.
	prev_acc_x = 0; prev_acc_y = 0; prev_acc_z = 0;
	prev_gyr_x = 0; prev_gyr_y = 0; prev_gyr_z = 0;
}

/**
 * @brief Reads accelerometer data from the MPU-6050.
 * @param imu Pointer to the IMU state structure.
 *
 * This function reads 6 bytes of raw data, converts them to signed 16-bit integers,
 * and scales them to 'g' units. It then applies the calibrated offsets.
 */
void LRL_IMU_ReadAccel(imu_statetype * imu)
{
	_LRL_IMU_MPUBypassEn(imu, 0);
	// Read 6 bytes of data from the accelerometer registers.
	HAL_I2C_Mem_Read(imu->imu.hi2c, MPU_ADDR, ACCEL_XOUT_H, 1, (uint8_t *)&imu_buffer, 6, DELAY_TIMEOUT);

	// Combine high and low bytes to get full 16-bit values.
	imu->accel.x = (int16_t)(imu_buffer[0] << 8 | imu_buffer[1]);
	imu->accel.y = (int16_t)(imu_buffer[2] << 8 | imu_buffer[3]);
	imu->accel.z = (int16_t)(imu_buffer[4] << 8 | imu_buffer[5]);

	// Scale the raw data to 'g' units based on the configured full-scale range (+/- 2g).
	imu->accel.x /= (ACCEL_X_CORRECTOR / FLOAT_SCALING);
	imu->accel.y /= (ACCEL_Y_CORRECTOR / FLOAT_SCALING);
	imu->accel.z /= (ACCEL_Z_CORRECTOR / FLOAT_SCALING);

	// Apply calibrated offsets.
	imu->accel.x_calibrated = imu->accel.x - imu->imu.offset_accel_x;
	imu->accel.y_calibrated = imu->accel.y - imu->imu.offset_accel_y;
	imu->accel.z_calibrated = imu->accel.z - imu->imu.offset_accel_z;
}

/**
 * @brief Reads gyroscope data from the MPU-6050.
 * @param imu Pointer to the IMU state structure.
 *
 * This function reads 6 bytes of raw data, converts them to signed 16-bit integers,
 * and scales them to 'degrees per second' units. It then applies the calibrated offsets.
 */
void LRL_IMU_ReadGyro(imu_statetype *imu)
{
	_LRL_IMU_MPUBypassEn(imu, 0);
	// Read 6 bytes of data from the gyroscope registers.
	HAL_I2C_Mem_Read(imu->imu.hi2c, MPU_ADDR, GYRO_XOUT_H, 1, (uint8_t *)imu_buffer, 6, DELAY_TIMEOUT);

	// Combine high and low bytes to get full 16-bit values.
	imu->gyro.x = (int16_t)(imu_buffer[0] << 8 | imu_buffer[1]);
	imu->gyro.y = (int16_t)(imu_buffer[2] << 8 | imu_buffer[3]);
	imu->gyro.z = (int16_t)(imu_buffer[4] << 8 | imu_buffer[5]);

	// Scale the raw data to 'deg/s' units based on the configured full-scale range (+/- 250 deg/s).
	imu->gyro.x /= (GYRO_CORRECTOR / FLOAT_SCALING);
	imu->gyro.y /= (GYRO_CORRECTOR / FLOAT_SCALING);
	imu->gyro.z /= (GYRO_CORRECTOR / FLOAT_SCALING);

	// Apply calibrated offsets.
	imu->gyro.x_calibrated = imu->gyro.x - imu->imu.offset_gyro_x;
	imu->gyro.y_calibrated = imu->gyro.y - imu->imu.offset_gyro_y;
	imu->gyro.z_calibrated = imu->gyro.z - imu->imu.offset_gyro_z;
}

/**
 * @brief Reads all IMU data (accelerometer and gyroscope).
 * @param imu Pointer to the IMU state structure.
 */
void LRL_IMU_MPUReadAll(imu_statetype *imu)
{
	_LRL_IMU_MPUBypassEn(imu, 0);
	LRL_IMU_ReadAccel(imu);
	LRL_IMU_ReadGyro(imu);
}

/**
 * @brief Enables or disables I2C Bypass mode on the MPU-6050.
 * @param imu Pointer to the IMU state structure.
 * @param enable 1 to enable, 0 to disable.
 *
 * I2C Bypass mode allows the STM32 to communicate directly with the HMC5883L
 * magnetometer, which is connected to the MPU-6050's auxiliary I2C bus.
 */
void _LRL_IMU_MPUBypassEn(imu_statetype * imu, uint8_t enable)
{
	i2c_reg_data = 0x00;
	HAL_I2C_Mem_Write(imu->imu.hi2c, MPU_ADDR, USER_CTRL, 1, &i2c_reg_data, 1, DELAY_TIMEOUT);
	i2c_reg_data = 0x02 * enable;
	HAL_I2C_Mem_Write(imu->imu.hi2c, MPU_ADDR, INT_PIN_CFG, 1, &i2c_reg_data, 1, DELAY_TIMEOUT);
	i2c_reg_data = 0x00;
	HAL_I2C_Mem_Write(imu->imu.hi2c, MPU_ADDR, PWR_MGMT_1, 1, &i2c_reg_data, 1, DELAY_TIMEOUT);
}

/**
 * @brief Implements a complementary filter for orientation estimation.
 * @param imu Pointer to the IMU state structure.
 * @param dt Time step in seconds.
 *
 * This function combines accelerometer and gyroscope data to provide a stable orientation.
 * It applies a low-pass filter to the accelerometer data and then a complementary filter
 * to combine it with the gyroscope data, compensating for drift.
 */
void LRL_IMU_ComplementaryFilter(imu_statetype *imu, float dt)
{
	static float _tmp_accx, _tmp_accy, _tmp_accz;
	_LRL_IMU_MPUBypassEn(imu, 0);

	// Low-pass filter accelerometer data.
	_tmp_accx = ALPHA * prev_acc_x + (1 - ALPHA) * imu->accel.x/FLOAT_SCALING;
	_tmp_accy = ALPHA * prev_acc_y + (1 - ALPHA) * imu->accel.y/FLOAT_SCALING;
	_tmp_accz = ALPHA * prev_acc_z + (1 - ALPHA) * imu->accel.z/FLOAT_SCALING;

	// Normalize accelerometer data to ensure it's a unit vector for angle calculation.
	float acc_norm = sqrtf(_tmp_accx * _tmp_accx + _tmp_accy * _tmp_accy + _tmp_accz * _tmp_accz);
	_tmp_accx /= acc_norm;
	_tmp_accy /= acc_norm;
	_tmp_accz /= acc_norm;

	// Calculate angles from accelerometer data.
	float acc_angle_x = atan2f(_tmp_accy, _tmp_accz) * (180.0f / M_PI);
	float acc_angle_y = atan2f(_tmp_accx, _tmp_accz) * (180.0f / M_PI);
	float acc_angle_z = atan2f(_tmp_accy, _tmp_accx) * (180.0f / M_PI);

	// Low-pass filter gyroscope data.
	float gyr_x_filtered = ALPHA * prev_gyr_x + (1 - ALPHA) * imu->gyro.x/FLOAT_SCALING;
	float gyr_y_filtered = ALPHA * prev_gyr_y + (1 - ALPHA) * imu->gyro.y/FLOAT_SCALING;
	float gyr_z_filtered = ALPHA * prev_gyr_z + (1 - ALPHA) * imu->gyro.z/FLOAT_SCALING;

	// Update angle using gyroscope integration.
	imu->imu.roll_temp += gyr_x_filtered * dt;
	imu->imu.pitch_temp -= gyr_y_filtered * dt;
	imu->imu.yaw_temp += gyr_z_filtered * dt;

	// Apply complementary filter to combine accelerometer and gyroscope data.
	imu->imu.roll_temp = (ALPHA * imu->imu.roll_temp + (1 - ALPHA) * acc_angle_x) ;
	imu->imu.pitch_temp = ((ALPHA * imu->imu.pitch_temp) - (1 - ALPHA) * acc_angle_y);
	imu->imu.yaw_temp = (ALPHA * imu->imu.yaw_temp + (1 - ALPHA) * acc_angle_z);

	// Apply calibrated offsets to the final angles.
	imu->angle.x = imu->imu.roll_temp - imu->imu.offset_calibration_x;
	imu->angle.y = imu->imu.pitch_temp - imu->imu.offset_calibration_y;
	imu->angle.z = imu->imu.yaw_temp - imu->imu.offset_calibration_z;

	// Update previous values for the next filter iteration.
	prev_acc_x = _tmp_accx;
	prev_acc_y = _tmp_accy;
	prev_acc_z = _tmp_accz;
	prev_gyr_x = gyr_x_filtered;
	prev_gyr_y = gyr_y_filtered;
	prev_gyr_z = gyr_z_filtered;
}
