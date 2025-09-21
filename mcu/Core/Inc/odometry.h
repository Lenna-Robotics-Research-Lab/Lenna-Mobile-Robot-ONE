/*
 * odometry.h
 *
 *  Created on: Jun 1, 2024
 *      Author: Lenna Robotics Research Laboratory
 *              Autonomous Systems Research Branch
 *              Iran University of Science and Technology
 *      GitHub: github.com/Lenna-Robotics-Research-Lab
 *
 *  Description:
 *  Public API and data structures for differential-drive odometry.
 *  Provides encoder configuration, wheel state, and function prototypes
 *  to initialize and update odometry from timer-driven quadrature encoders.
 */

#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_

#include "i2c.h"
#include "tim.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "motion.h"

// ############################################################
// ####################  ODOMETRY STRUCTS  ####################
// ############################################################

/**
 * @brief Wheel angular velocities (right/left).
 *
 * Units:
 *  - Typically RPM after applying encoder calibration (see TICK2RPM).
 *  - Sign convention is determined by update routine (forward/backward).
 */
typedef struct
{
    int16_t right;   ///< Right wheel angular velocity (e.g., RPM, signed)
    int16_t left;    ///< Left  wheel angular velocity (e.g., RPM, signed)
} wheel_velocity;

/**
 * @brief Wheel incremental distances (right/left).
 *
 * Units:
 *  - Application-defined (e.g., ticks, mm, or discretized steps).
 *  - In this project, these are often converted from ticks each update.
 */
typedef struct
{
    int16_t right;   ///< Right wheel incremental distance (signed)
    int16_t left;    ///< Left  wheel incremental distance (signed)
} wheel_position;

/**
 * @brief Encoder configuration and runtime state for a single wheel.
 *
 * Members:
 *  - htim:      Hardware timer used in encoder mode.
 *  - MAX_ARR:   Timer auto-reload value (max count before wrap).
 *  - TICK2RPM:  Conversion factor from ticks/update to RPM.
 *  - tick:      Current raw timer count.
 *  - tick_prev: Previous raw timer count (for delta computation).
 */
typedef struct
{
    TIM_HandleTypeDef *htim;  ///< Pointer to the timer configured in encoder mode

    uint16_t MAX_ARR;         ///< Timer ARR (max count), used to handle wrap-around
    float    TICK2RPM;        ///< Scale factor: ticks per update → RPM

    uint16_t tick;            ///< Current encoder tick (timer counter)
    uint16_t tick_prev;       ///< Previous encoder tick (for delta calculation)
} encoder_cfgType;

/**
 * @brief Odometry configuration and state for a differential-drive robot.
 *
 * Contains:
 *  - Two encoders (right/left)
 *  - Robot geometry/config (diff_robot)
 *  - Current wheel velocities and incremental distances
 *
 * Optional IMU and pose/attitude fields are provided in comments for future use.
 */
typedef struct
{
    // imu_cfgType        imu;         ///< IMU configuration/state (optional)

    encoder_cfgType     enc_right;    ///< Right wheel encoder configuration/state
    encoder_cfgType     enc_left;     ///< Left  wheel encoder configuration/state
    diffDrive_cfgType   diff_robot;   ///< Differential-drive geometry/config (e.g., wheel radius, track width)

    // linear_position   pose;        ///< Robot pose in world frame (optional)
    // angular_position  angle;       ///< Robot orientation (optional)
    // accelerometer     accel;       ///< IMU accelerometer readings (optional)
    // gyroscope         gyro;        ///< IMU gyroscope readings (optional)
    // magnetometer      mag;         ///< IMU magnetometer readings (optional)

    wheel_velocity      vel;          ///< Computed wheel angular velocities (signed)
    wheel_position      dist;         ///< Computed incremental distances per update (signed)
} odom_cfgType;

// ##############################################################
// ####################  FUNCTION PROTOTYPES  ###################
// ##############################################################

/**
 * @brief Initialize odometry subsystem.
 *
 * Resets hardware timer counters and software tick history so that
 * subsequent updates produce correct deltas.
 *
 * @param odom Pointer to odometry configuration/state structure.
 */
void LRL_Odometry_Init(odom_cfgType *odom);

/**
 * @brief Read encoder counters, compute wheel speeds and distances.
 *
 * Steps performed:
 *  1) Read current timer counts and counting direction.
 *  2) Compute tick deltas with wrap-around handling.
 *  3) Convert ticks to wheel RPM using @ref encoder_cfgType::TICK2RPM.
 *  4) Convert ticks to incremental arc lengths using wheel radius.
 *  5) Update previous tick values for the next call.
 *
 * @param odom Pointer to odometry configuration/state structure.
 */
void LRL_Odometry_ReadAngularSpeed(odom_cfgType *odom);

#endif /* INC_ODOMETRY_H_ */
