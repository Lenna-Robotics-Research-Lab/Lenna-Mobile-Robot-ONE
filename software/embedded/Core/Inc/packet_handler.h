/**
 * @file packet_handler.h
 * @brief Header file for the communication packet handling module.
 *
 * This file defines the structures, constants, and function prototypes
 * required for serial communication, including packet formatting,
 * transmission, reception, and CRC validation.
 *
 * @author Lenna Robotics Research Laboratory, Autonomous Systems Research Branch, Iran University of Science and Technology
 * @date June 5, 2024
 * @version 1.0
 * @link https://github.com/Lenna-Robotics-Research-Lab
 */

#ifndef INC_PACKET_HANDLER_H_
#define INC_PACKET_HANDLER_H_

/* Includes ------------------------------------------------------------------*/
#include "imu.h"
#include "usart.h" // Assuming this is where UART_HandleTypeDef is defined

/* Defines -------------------------------------------------------------------*/

/**
 * @name Packet Length Constants
 * @brief Defines the minimum and maximum packet sizes for the protocol.
 */
///@{
#define MIN_PACKET_LENGTH	3	/**< Minimum expected packet length. */
#define MAX_PACKET_LENGTH	144	/**< Maximum buffer size for a packet. */
#define MAX_DATA_LENGTH		100
///@}

/* Type Definitions ----------------------------------------------------------*/

/**
 * @struct packet_data
 * @brief Stores velocity data from an incoming command packet.
 */
typedef struct
{
	int16_t right_velocity;	/**< Desired right wheel velocity. */
	int16_t left_velocity;	/**< Desired left wheel velocity. */
} packet_vel_data;

/**
 * @struct packet_cfgType
 * @brief Stores configuration and state data for the packet handler module.
 */
typedef struct
{
	UART_HandleTypeDef *huart;						/**< Pointer to the UART handle for communication. */
	uint8_t				min_pkt_lenght;				/**< Minimum packet length. */
	uint8_t				max_pkt_lenght;				/**< Maximum packet length. */
	uint8_t				dataValid;					/**< Flag to indicate if received data is valid (CRC passed). */
	uint8_t				byteReady;					/**< Flag to indicate a new byte has been received. */
	uint8_t 			buffer[MAX_PACKET_LENGTH];	/**< Buffer to store incoming and outgoing data. */
	uint8_t				ack;						/**< Acknowledgment byte for handshake. */
	packet_vel_data		vel_data;					/**< Parsed velocity data from a received packet. */
	uint8_t				data[MAX_DATA_LENGTH];						/**< data packet inside the buffer >*/
	unsigned short 		crc_val;
} packet_cfgType;

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief Initializes the packet handler.
 * @param packet Pointer to the packet configuration structure.
 */
void LRL_Packet_Init(packet_cfgType *packet);

/**
 * @brief Callback for the protocol.
 * @param packet Pointer to the packet configuration structure.
 */
void LRL_Protocol_RX(packet_cfgType *packet);

/**
 * @brief Performs a handshake to establish a connection.
 * @param packet Pointer to the packet configuration structure.
 */
void LRL_Packet_Handshake(packet_cfgType *packet);

/**
 * @brief Calculates and updates a CRC checksum.
 * @param crc_accum The initial CRC value.
 * @param data_blk_ptr Pointer to the data block.
 * @param data_blk_size Size of the data block.
 * @param crc_final A reference to store the final CRC value.
 */
void LRL_Packet_UpdateCRC(uint8_t *data_blk_ptr, uint16_t data_blk_size, unsigned short crc_final );

/**
 * @brief Handles an incoming data packet.
 * @param packet Pointer to the packet configuration structure.
 */
void LRL_Packet_RX(packet_cfgType *packet);

/**
 * @brief Assembles and transmits a data packet.
 * @param packet Pointer to the packet configuration structure.
 * @param odom Pointer to the odometry state structure.
 * @param imu Pointer to the IMU state structure.
 */
void LRL_Packet_TX(packet_cfgType *packet, odom_cfgType *odom, imu_statetype *imu);

#endif /* INC_PACKET_HANDLER_H_ */
