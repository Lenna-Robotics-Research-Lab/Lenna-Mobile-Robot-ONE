/**
 * @file imu.h
 * @brief Header file for the IMU and magnetometer module.
 *
 * This file defines the data structures, register addresses, and function
 * prototypes for communicating with an MPU-6050 IMU and an HMC5883L magnetometer.
 *
 * @author Lenna Robotics Research Laboratory, Autonomous Systems Research Branch, Iran University of Science and Technology
 * @date September 21, 2025
 * @version 1.0
 * @link https://github.com/Lenna-Robotics-Research-Lab
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "tim.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "motion.h"

/* Type Definitions ----------------------------------------------------------*/

/**
 * @struct linear_position
 * @brief Represents linear position or acceleration data in three axes (x, y, z).
 */
typedef struct
{
	float	x;	/**< X-axis value. */
	float	y;	/**< Y-axis value. */
	float	z;	/**< Z-axis value. */
} linear_position;

/**
 * @struct angular_position
 * @brief Represents angular position data in three axes: roll, pitch, and yaw.
 */
typedef struct
{
	int16_t	x;	/**< Roll angle (x-axis rotation). */
	int16_t	y;	/**< Pitch angle (y-axis rotation). */
	int16_t	z;	/**< Yaw angle (z-axis rotation). */
} angular_position;

/**
 * @struct accelerometer
 * @brief Stores raw and calibrated accelerometer data.
 */
typedef struct
{
	int16_t	x;	/**< Raw x-axis accelerometer data. */
	int16_t	y;	/**< Raw y-axis accelerometer data. */
	int16_t	z;	/**< Raw z-axis accelerometer data. */

	int16_t	x_calibrated;	/**< Calibrated x-axis accelerometer data. */
	int16_t	y_calibrated;	/**< Calibrated y-axis accelerometer data. */
	int16_t	z_calibrated;	/**< Calibrated z-axis accelerometer data. */
} accelerometer;

/**
 * @struct gyroscope
 * @brief Stores raw and calibrated gyroscope data.
 */
typedef struct
{
	int16_t	x;	/**< Raw x-axis gyroscope data. */
	int16_t	y;	/**< Raw y-axis gyroscope data. */
	int16_t	z;	/**< Raw z-axis gyroscope data. */

	int16_t	x_calibrated;	/**< Calibrated x-axis gyroscope data. */
	int16_t	y_calibrated;	/**< Calibrated y-axis gyroscope data. */
	int16_t	z_calibrated;	/**< Calibrated z-axis gyroscope data. */
} gyroscope;

/**
 * @struct magnetometer
 * @brief Stores magnetometer data, heading, and calibration offset.
 */
typedef struct
{
	float	x;				/**< X-axis magnetic field strength. */
	float	y;				/**< Y-axis magnetic field strength. */
	float	z;				/**< Z-axis magnetic field strength. */
	int16_t	heading;		/**< Calculated magnetic heading in degrees. */
	int16_t offset_heading;	/**< Heading calibration offset. */
} magnetometer;

/**
 * @struct imu_cfgType
 * @brief Configuration and calibration data for the IMU and complementary filter.
 */
typedef struct
{
	I2C_HandleTypeDef * hi2c;			/**< Pointer to the I2C handle for communication. */

	int16_t offset_gyro_x;				/**< Gyroscope x-axis offset. */
	int16_t offset_gyro_y;				/**< Gyroscope y-axis offset. */
	int16_t offset_gyro_z;				/**< Gyroscope z-axis offset. */

	int16_t offset_accel_x;				/**< Accelerometer x-axis offset. */
	int16_t offset_accel_y;				/**< Accelerometer y-axis offset. */
	int16_t offset_accel_z;				/**< Accelerometer z-axis offset. */

	float 	offset_calibration_x;		/**< Complementary filter x-axis offset. */
	float 	offset_calibration_y;		/**< Complementary filter y-axis offset. */
	float 	offset_calibration_z;		/**< Complementary filter z-axis offset. */

	float 	roll_temp;					/**< Temporary roll angle for filter calculation. */
	float 	pitch_temp;					/**< Temporary pitch angle for filter calculation. */
	float	yaw_temp;					/**< Temporary yaw angle for filter calculation. */
} imu_cfgType;

/**
 * @struct imu_statetype
 * @brief Main state structure holding all IMU-related data and configurations.
 */
typedef struct
{
	imu_cfgType 		imu;			/**< IMU configuration and calibration data. */
	diffDrive_cfgType	diff_robot;		/**< Robot configuration data. */

	linear_position 	pose;			/**< Linear position data. */
	angular_position 	angle;			/**< Angular position data (roll, pitch, yaw). */
	accelerometer 		accel;			/**< Accelerometer data. */
	gyroscope 			gyro;			/**< Gyroscope data. */
	magnetometer 		mag;			/**< Magnetometer data. */
} imu_statetype;


/*
 ******************************************************************************
 * #################### HMC5883L MAGNETOMETER CONSTANTS ####################
 ******************************************************************************
 */
#define HMC5883L_DEFAULT_ADDRESS    0x1E			/**< Default I2C address. */
#define HMC5883L_ADDRESS            (0x1E << 1)		/**< 7-bit I2C address (shifted for HAL library). */
#define HMC5883L_ADDRESS_WRITE      0x3C			/**< I2C write address. */
#define HMC5883L_ADDRESS_READ       0x3D			/**< I2C read address. */

// HMC5883L Register Address List
#define HMC5883L_RA_CONFIG_A        0x00
#define HMC5883L_RA_CONFIG_B        0x01
#define HMC5883L_RA_MODE            0x02
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAX_L         0x04
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAZ_L         0x06
#define HMC5883L_RA_DATAY_H         0x07
#define HMC5883L_RA_DATAY_L         0x08
#define HMC5883L_RA_STATUS          0x09
#define HMC5883L_RA_ID_A            0x0A
#define HMC5883L_RA_ID_B            0x0B
#define HMC5883L_RA_ID_C            0x0C

// HMC5883L Measurement Mode Options
#define HMC5883L_MODE_CONTINUOUS    0x00	/**< Continuous-Measurement Mode. */
#define HMC5883L_MODE_SINGLE        0x01	/**< Single-Measurement Mode. */
#define HMC5883L_MODE_IDLE          0x02	/**< Idle Mode. */

// Other HMC5883L configuration options...
// (Add brief Doxygen comments for these if needed, otherwise group them for clarity)
#define HMC5883L_AVERAGING_1        0x00
#define HMC5883L_AVERAGING_2        0x01
#define HMC5883L_AVERAGING_4        0x02
#define HMC5883L_AVERAGING_8        0x03

#define HMC5883L_RATE_0P75          0x00
#define HMC5883L_RATE_1P5           0x01
#define HMC5883L_RATE_3             0x02
#define HMC5883L_RATE_7P5           0x03
#define HMC5883L_RATE_15            0x04
#define HMC5883L_RATE_30            0x05
#define HMC5883L_RATE_75            0x06

#define HMC5883L_BIAS_NORMAL        0x00
#define HMC5883L_BIAS_POSITIVE      0x01
#define HMC5883L_BIAS_NEGATIVE      0x02

#define HMC5883L_GAIN_1370          0x00
#define HMC5883L_GAIN_1090          0x01
#define HMC5883L_GAIN_820           0x02
#define HMC5883L_GAIN_660           0x03
#define HMC5883L_GAIN_440           0x04
#define HMC5883L_GAIN_390           0x05
#define HMC5883L_GAIN_330           0x06
#define HMC5883L_GAIN_220           0x07

/**
 * @brief Magnetic declination in radians for a specific location.
 *
 * This value is used to correct the magnetic heading to true north.
 */
#define MAGNETIC_DECLINATION		0.0881


/*
 ******************************************************************************
 * ####################### MPU-6050 IMU CONSTANTS ########################
 ******************************************************************************
 */
#define WHO_AM_I 		0x75		/**< Device ID register address. */
#define INT_PIN_CFG 	0x37		/**< Interrupt Pin/Bypass Enable Configuration register. */
#define USER_CTRL 		0x6A		/**< User Control register. */
#define PWR_MGMT_1 		0x6B		/**< Power Management 1 register. */
#define SMPLRT_DIV 		0x19		/**< Sample Rate Divider register. */
#define ACCEL_CONFIG 	0x1C		/**< Accelerometer Configuration register. */
#define ACCEL_XOUT_H 	0x3B		/**< Accelerometer X-axis High register. */
#define TEMP_OUT_H 		0x41		/**< Temperature Sensor High register. */
#define GYRO_CONFIG 	0x1B		/**< Gyroscope Configuration register. */
#define GYRO_XOUT_H 	0x43		/**< Gyroscope X-axis High register. */

#define MPU_ADDR 		0xD0		/**< MPU-6050 I2C address. */

// Scaling factors for converting raw sensor data to engineering units.
#define ACCEL_X_CORRECTOR 16384		/**< Accelerometer scaling factor for X-axis (+/- 2g). */
#define ACCEL_Y_CORRECTOR 16384		/**< Accelerometer scaling factor for Y-axis (+/- 2g). */
#define ACCEL_Z_CORRECTOR 14418		/**< Accelerometer scaling factor for Z-axis (+/- 2g). */

#define GYRO_CORRECTOR 	131.0		/**< Gyroscope scaling factor (+/- 250 deg/s). */

// Complementary filter constants
#define ALPHA 			0.9f		/**< Weighting factor for the complementary filter. */

#define DELAY_TIMEOUT	10			/**< Timeout for I2C communication operations. */
#define FLOAT_SCALING	1000		/**< Scaling factor for floating-point conversion. */


/* Function Prototypes -------------------------------------------------------*/
/**
 * @brief Sets the magnetic declination angle.
 * @param declination_degs Degrees of declination.
 * @param declination_mins Minutes of declination.
 * @param declination_dir Direction of declination ('E' or 'W').
 * @return The declination angle in radians.
 */
float LRL_IMU_MagSetDeclination(int16_t declination_degs , int16_t declination_mins, char declination_dir);

/**
 * @brief Initializes the HMC5883L magnetometer.
 * @param imu Pointer to the IMU state structure.
 */
void LRL_IMU_MagInit(imu_statetype * imu);

/**
 * @brief Reads the magnetic heading from the HMC5883L sensor.
 * @param imu Pointer to the IMU state structure.
 */
void LRL_IMU_MagReadHeading(imu_statetype * imu);

/**
 * @brief Initializes the MPU-6050 IMU.
 * @param imu Pointer to the IMU state structure.
 */
void LRL_IMU_MPUInit(imu_statetype * imu);

/**
 * @brief Reads accelerometer data from the MPU-6050.
 * @param imu Pointer to the IMU state structure.
 */
void LRL_IMU_ReadAccel(imu_statetype * imu);

/**
 * @brief Enables or disables I2C Bypass mode on the MPU-6050.
 * @param imu Pointer to the IMU state structure.
 * @param enable 1 to enable, 0 to disable.
 */
void _LRL_IMU_MPUBypassEn(imu_statetype * imu, uint8_t enable);

/**
 * @brief Reads gyroscope data from the MPU-6050.
 * @param imu Pointer to the IMU state structure.
 */
void LRL_IMU_ReadGyro(imu_statetype *imu);

/**
 * @brief Reads all IMU data (accelerometer and gyroscope).
 * @param imu Pointer to the IMU state structure.
 */
void LRL_IMU_MPUReadAll(imu_statetype *imu);

/**
 * @brief Implements a complementary filter for orientation estimation.
 * @param imu Pointer to the IMU state structure.
 * @param dt Time step in seconds.
 */
void LRL_IMU_ComplementaryFilter(imu_statetype *imu, float dt);


#endif /* INC_IMU_H_ */
