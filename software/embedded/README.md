# Lenna Mobile Robot ONE - Embedded Software

## General Information

### Description

This embedded software is the low-level control system for the **Lenna Mobile Robot ONE**, a differential drive robot. It runs on an **STM32F407VGTX** microcontroller and is responsible for interfacing with all the onboard hardware, executing motion commands, and providing sensor feedback to a higher-level control system (e.g., a computer running ROS).

The core functionalities of this software include:
-   **Motor Control**: Precise control of the two DC motors using a PID (Proportional-Integral-Derivative) controller to achieve desired velocities.
-   **Odometry**: Reading data from wheel encoders to calculate the robot's position and orientation (pose).
-   **Sensor Fusion**: Interfacing with an Inertial Measurement Unit (IMU), specifically the MPU-6050 (accelerometer and gyroscope) and HMC5883L (magnetometer), and using a complementary filter to get a stable orientation estimate.
-   **Obstacle Avoidance**: Reading data from HC-SR04 ultrasonic sensors to detect obstacles in the robot's path.
-   **Communication**: Handling a custom serial communication protocol to receive velocity commands and send sensor data packets to a host computer.

---
### Dependencies

This project relies on the following libraries and modules:

-   **STM32F4xx HAL Library**: The primary dependency for this project. The Hardware Abstraction Layer (HAL) provided by STMicroelectronics is used for all low-level hardware interactions, including GPIO, I2C, UART, SPI, TIM (timers), and ADC peripherals.
-   **CMSIS (Cortex Microcontroller Software Interface Standard)**: This library is used for basic access to the ARM Cortex-M4 processor core features.
-   **C Standard Libraries**:
    -   `stdio.h`: For standard input/output functions.
    -   `stdlib.h`: For general utility functions.
    -   `stdbool.h`: For boolean type and values.
    -   `math.h`: For mathematical functions like `sqrtf` and `atan2f`.
    -   `string.h`: For memory manipulation functions like `memset`.

---
### Coding Convention

The code follows a structured and well-documented convention to ensure readability and maintainability.

-   **File and Module Structure**: Each distinct piece of functionality (e.g., motion, PID, odometry) is separated into its own module with a corresponding `.h` (header) and `.c` (source) file.
-   **Naming Convention**:
    -   **Functions**: All public functions are prefixed with `LRL_` (for Lenna Robotics Laboratory), followed by the module name, and then the function name in CamelCase (e.g., `LRL_Motion_MotorSpeed`).
    -   **Structures and Typedefs**: Custom data types generally end with `_cfgType` for configuration structures or `_statetype` for state-holding structures (e.g., `motor_cfgType`, `imu_statetype`).
    -   **Constants and Macros**: All constants and macros are defined in uppercase with underscores separating words (e.g., `MAX_PACKET_LENGTH`).
-   **Commenting**: The code is extensively commented using a **Doxygen-compatible** style. Each file and function has a header comment block explaining its purpose, author, date, parameters (`@param`), and return values (`@return`). This allows for automatic documentation generation.
-   **Code Style**:
    -   The code maintains a consistent indentation style.
    -   Function and variable names are descriptive.
    -   Private functions, intended for use only within their own module, are sometimes prefixed with an underscore (e.g., `_LRL_IMU_MPUBypassEn`).
-   **Configuration**: All hardware-related pin definitions and magic numbers are centralized in `mcu_config.h` and `main.h` to make it easier to adapt the code to different hardware configurations.

---
## Module Deep Dive

### Motion Control (`motion.c`/`motion.h`)

This module is responsible for the direct control of the robot's motors.

* **Key Data Structures**:
    * `motor_cfgType`: This structure holds the hardware configuration for a single motor, including GPIO pins for direction control and the timer handle and channel for PWM signal generation.
    * `diffDrive_cfgType`: This structure defines the entire differential drive robot, combining two `motor_cfgType` structures (one for the left motor and one for the right) and storing the robot's physical parameters, such as wheel radius and the distance between the wheels.

* **Main Functions**:
    * `LRL_Motion_MotorSpeed(motor_cfgType motor, int8_t duty_cycle)`: Sets the speed and direction of a single motor. A positive `duty_cycle` value drives the motor forward, while a negative value drives it backward. The magnitude of the `duty_cycle` (from -100 to 100) determines the motor's speed.
    * `LRL_Motion_Control(diffDrive_cfgType diffRobot, int8_t duty_cycle_left, int8_t duty_cycle_right)`: A higher-level function that controls both motors of the robot simultaneously, allowing for differential drive maneuvers like turning and moving in arcs.
    * `LRL_Motion_MotorTest(diffDrive_cfgType diffRobot)`: A utility function to run a pre-defined sequence of movements (e.g., forward, backward, pivot turns) to test the functionality of the motors and encoders.

---
### PID Controller (`pid.c`/`pid.h`)

This module implements a Proportional-Integral-Derivative (PID) controller for precise, closed-loop speed control of the motors.

* **Key Data Structures**:
    * `pid_cfgType`: This is the core structure for the PID controller. It stores the controller's gains (`Kp`, `Ki`, `Kd`), the sampling time (`Ts`), saturation limits for the output signal, and internal state variables like the accumulated integral amount and the previous measurement error. It also includes a flag to enable or disable an anti-windup feature.

* **Main Functions**:
    * `LRL_PID_Init(pid_cfgType *pid_cfg, uint8_t AntiWindup)`: Initializes the PID controller's state variables to zero and sets the anti-windup configuration.
    * `LRL_PID_Update(pid_cfgType *pid_cfg, int16_t measurement, int16_t set_point)`: This function is the heart of the PID controller. It calculates the error between the desired `set_point` and the current `measurement` (e.g., motor speed from odometry), computes the proportional, integral, and derivative terms, and generates a new control signal. It also handles output saturation and the anti-windup mechanism to prevent the integral term from accumulating excessively when the output is saturated.

---
### Odometry (`odometry.c`/`odometry.h`)

This module is responsible for tracking the robot's movement by reading data from the wheel encoders.

* **Key Data Structures**:
    * `encoder_cfgType`: Holds the configuration and state for a single wheel encoder, including a pointer to the hardware timer used in encoder mode, the timer's maximum count value (`MAX_ARR`), a conversion factor from ticks to RPM (`TICK2RPM`), and the current and previous tick counts.
    * `odom_cfgType`: The main odometry structure that combines two `encoder_cfgType` instances (for the right and left wheels), the robot's physical configuration (`diffDrive_cfgType`), and the calculated wheel velocities and incremental distances.

* **Main Functions**:
    * `LRL_Odometry_Init(odom_cfgType *odom)`: Initializes the odometry system by resetting the hardware encoder counters and clearing the software tick history.
    * `LRL_Odometry_ReadAngularSpeed(odom_cfgType *odom)`: This function performs the core odometry calculations. It reads the current raw tick counts from the encoder timers, calculates the difference in ticks since the last call (handling timer overflows), and converts this difference into physically meaningful units: angular velocity (in RPM) and the incremental distance traveled by each wheel.

---
### IMU and Magnetometer (`imu.c`/`imu.h`)

This module interfaces with the MPU-6050 Inertial Measurement Unit (IMU) and the HMC5883L magnetometer to determine the robot's orientation in space.

* **Key Data Structures**:
    * `imu_statetype`: The main state structure that holds all IMU-related data. This includes the I2C handle for communication, calibration offsets for the accelerometer, gyroscope, and magnetometer, and the latest sensor readings (raw and filtered).
    * The file also defines several helper structs like `linear_position`, `angular_position`, `accelerometer`, `gyroscope`, and `magnetometer` to organize the sensor data.

* **Main Functions**:
    * `LRL_IMU_MPUInit(imu_statetype * imu)`: Initializes the MPU-6050 sensor, configuring its power management, data rate, and measurement ranges. It also performs a calibration routine to determine the zero-rate offsets for the accelerometer and gyroscope.
    * `LRL_IMU_MagInit(imu_statetype * imu)`: Initializes the HMC5883L magnetometer, which is connected through the MPU-6050's auxiliary I2C bus. This function also calibrates the magnetometer's heading.
    * `LRL_IMU_MPUReadAll(imu_statetype *imu)`: A convenience function that reads the latest data from both the accelerometer and gyroscope.
    * `LRL_IMU_ComplementaryFilter(imu_statetype *imu, float dt)`: This is the sensor fusion algorithm. It combines the orientation data from the accelerometer (which is reliable in the long term but noisy) and the gyroscope (which is reliable in the short term but drifts over time) to produce a stable and accurate estimate of the robot's roll, pitch, and yaw angles.

---
### Packet Handler (`packet_handler.c`/`packet_handler.h`)

This module manages the serial communication protocol between the microcontroller and a host computer.

* **Key Data Structures**:
    * `packet_cfgType`: Stores the configuration and state for the packet handler, including a pointer to the UART handle, the minimum and maximum packet lengths, flags to indicate data validity, and a buffer for incoming and outgoing data.

* **Main Functions**:
    * `LRL_Packet_Handshake(packet_cfgType *packet)`: Establishes a connection with the host computer by repeatedly sending a signature and waiting for a specific acknowledgment. This ensures that both the robot and the host are ready for communication.
    * `LRL_Packet_TX(packet_cfgType *packet, odom_cfgType *odom, imu_statetype *imu)`: Assembles a data packet containing the latest sensor and odometry information (wheel velocities, distances, accelerometer data, gyroscope data, and orientation angles). It then calculates a CRC checksum for data integrity and transmits the packet via UART.
    * `LRL_Packet_RX(packet_cfgType *packet)`: Handles incoming data packets from the host. It reads the packet from the UART buffer, validates its integrity by checking the CRC, and if the packet is valid, it extracts the velocity commands for the left and right wheels.

---
### Ultrasonic Sensor (`ultrasonic.c`/`ultrasonic.h`)

This module provides an interface for the HC-SR04 ultrasonic distance sensors.

* **Key Data Structures**:
    * `ultrasonic_cfgType`: Holds the hardware configuration for a single ultrasonic sensor, including the GPIO pin for the trigger signal and the timer configuration (handle, instance, and channel) for measuring the echo pulse duration using the input capture feature.

* **Main Functions**:
    * `LRL_US_Init(ultrasonic_cfgType us)`: Initializes the timer and input capture interrupts for a given ultrasonic sensor.
    * `LRL_US_Trig(ultrasonic_cfgType us)`: Sends a short trigger pulse to the ultrasonic sensor to initiate a distance measurement.
    * `LRL_US_TMR_IC_ISR(TIM_HandleTypeDef *htim, ultrasonic_cfgType us)`: This is the interrupt service routine for the timer's input capture. It measures the time between the rising and falling edges of the echo pulse to calculate the distance to an object.
    * `LRL_US_Read(ultrasonic_cfgType us)`: Returns the last measured distance in centimeters.

---
### Utilities (`utilities.c`/`utilities.h`)

This module contains miscellaneous helper functions.

* **Main Functions**:
    * `LRL_Delay_Us(volatile uint16_t delay_us)`: Provides a blocking delay with microsecond-level precision, implemented using one of the microcontroller's hardware timers. This is useful for timing-critical operations like generating the trigger pulse for the ultrasonic sensor.
