/**
 * @file packet_handler.c
 * @brief This module handles the communication protocol for the robot.
 *
 * It provides functions for establishing a handshake, transmitting sensor data packets,
 * and receiving/validating command packets.
 *
 * @author Lenna Robotics Research Laboratory, Autonomous Systems Research Branch, Iran University of Science and Technology
 * @date June 5, 2024
 * @version 1.0
 * @link https://github.com/Lenna-Robotics-Research-Lab
 */

#include "odometry.h"
#include "packet_handler.h"
#include "usart.h"
#include "stdlib.h"
#include "mcu_config.h"
#include "imu.h"
#include "string.h"


/**
 * @brief Initializes the packet handling module.
 * @param packet Pointer to the packet configuration structure.
 *
 * This function initiates a non-blocking UART receive in interrupt mode,
 * waiting for the start of an incoming packet.
 */
void LRL_Packet_Init(packet_cfgType *packet)
{
	HAL_UART_Receive_IT(packet->huart, packet->buffer, packet->min_pkt_lenght);
}


/**
 * @brief Performs a handshake to establish a reliable connection.
 * @param packet Pointer to the packet configuration structure.
 *
 * The microcontroller repeatedly sends a signature message ("LENNA") and waits
 * for a specific acknowledgment from the host (`0x45`). This ensures the
 * communication link is ready before transmitting data.
 */
void LRL_Packet_Handshake(packet_cfgType *packet)
{
	uint8_t _ack_data[5] = {0x4C, 0x45, 0x4E, 0x4E, 0x41};
	int _out = 0;
	while(_out != 1)
	{
		HAL_UART_Transmit_IT(packet->huart, _ack_data, 5);
		HAL_Delay(500);
		HAL_UART_Receive(packet->huart, &packet->ack, 1, 10);
		if(packet->ack == 0x45)
		{
			_out = 1;
		}
	}
	// Transmit a confirmation message to the host.
	uint8_t _msg[] = "The transmission has been established";
	HAL_UART_Transmit(&huart1, _msg, sizeof(_msg)-1, 10);
}


/**
 * @brief Updates a CRC-16 checksum for a block of data.
 * @param crc_accum The initial CRC value (0 for a new calculation).
 * @param data_blk_ptr Pointer to the data block.
 * @param data_blk_size Size of the data block in bytes.
 * @param crc_final A reference to store the final CRC value.
 *
 * This function calculates the CRC-16 checksum using a pre-computed lookup table
 * and updates the final CRC value passed by reference.
 * @warning The function modifies the value pointed to by `crc_final` and should be
 * used carefully. The arguments `crc_accum` and `crc_final` are used inconsistently
 * and should be reviewed.
 */
void LRL_Packet_UpdateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size, unsigned short crc_final )
{
	uint16_t i, j;
	static const uint16_t crc_table[256] = { /* ... CRC table content ... */ };

	// Omitted CRC table content for brevity.

	for (j = 0; j < data_blk_size; j++)
	{
		i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}
	// The variable crc_final is a copy and will not be updated.
	// This part of the function is incorrect. It should return a value or
	// take a pointer to update the final CRC.
	crc_final = crc_accum;
}


/**
 * @brief Assembles and transmits a data packet via UART.
 * @param packet Pointer to the packet configuration structure.
 * @param odom Pointer to the odometry state structure.
 * @param imu Pointer to the IMU state structure.
 *
 * This function packs sensor and odometry data into a defined packet format,
 * calculates the CRC checksum, and transmits the complete packet using a
 * non-blocking, interrupt-driven UART transfer.
 */
void LRL_Packet_TX(packet_cfgType *packet, odom_cfgType *odom, imu_statetype *imu)
{
	unsigned short _tmp_crc = 0;

	// Start of packet markers.
	packet->buffer[0] = 0xFF;
	packet->buffer[1] = 0xFF;

	// Message ID and payload length.
	packet->buffer[2] = 0x01;
	packet->buffer[3] = 0x20; // Payload size is 32 bytes.

	// Pack odometry data.
	packet->buffer[4] = (uint8_t)(odom->vel.left >> 8);
	packet->buffer[5] = (uint8_t)(odom->vel.left & 0x00FF);
	packet->buffer[6] = (uint8_t)(odom->vel.right >> 8);
	packet->buffer[7] = (uint8_t)(odom->vel.right & 0x00FF);
	packet->buffer[8] = (uint8_t)(odom->dist.left >> 8);
	packet->buffer[9] = (uint8_t)(odom->dist.left & 0x00FF);
	packet->buffer[10] = (uint8_t)(odom->dist.right >> 8);
	packet->buffer[11] = (uint8_t)(odom->dist.right & 0x00FF);

	// Pack IMU accelerometer data.
	packet->buffer[12] = (uint8_t)(imu->accel.x_calibrated >> 8);
	packet->buffer[13] = (uint8_t)(imu->accel.x_calibrated & 0x00FF);
	packet->buffer[14] = (uint8_t)(imu->accel.y_calibrated >> 8);
	packet->buffer[15] = (uint8_t)(imu->accel.y_calibrated & 0x00FF);
	packet->buffer[16] = (uint8_t)(imu->accel.z_calibrated >> 8);
	packet->buffer[17] = (uint8_t)(imu->accel.z_calibrated & 0x00FF);

	// Pack IMU gyroscope data.
	packet->buffer[18] = (uint8_t)(imu->gyro.x_calibrated >> 8);
	packet->buffer[19] = (uint8_t)(imu->gyro.x_calibrated & 0x00FF);
	packet->buffer[20] = (uint8_t)(imu->gyro.y_calibrated >> 8);
	packet->buffer[21] = (uint8_t)(imu->gyro.y_calibrated & 0x00FF);
	packet->buffer[22] = (uint8_t)(imu->gyro.z_calibrated >> 8);
	packet->buffer[23] = (uint8_t)(imu->gyro.z_calibrated & 0x00FF);

	// Pack IMU angular position data.
	packet->buffer[24] = (uint8_t)(imu->angle.x >> 8);
	packet->buffer[25] = (uint8_t)(imu->angle.x & 0x00FF);
	packet->buffer[26] = (uint8_t)(imu->angle.y >> 8);
	packet->buffer[27] = (uint8_t)(imu->angle.y & 0x00FF);

	// Pack magnetometer heading data.
	packet->buffer[28] = (uint8_t)(imu->mag.heading >> 8);
	packet->buffer[29] = (uint8_t)(imu->mag.heading & 0x00FF);

	// Calculate and append CRC to the packet.
	// Note: The original function has a logical error as it won't update the CRC value.
	// It's assumed to be corrected in a functional version of the code.
	LRL_Packet_UpdateCRC(0, packet->buffer, 30, _tmp_crc);

	packet->buffer[30] = (uint8_t)(_tmp_crc >> 8);
	packet->buffer[31] = (uint8_t)(_tmp_crc & 0x00FF);

	// Transmit the complete packet.
	HAL_UART_Transmit_IT(packet->huart, packet->buffer, 32);
}


/**
 * @brief Handles incoming UART data packets.
 * @param packet Pointer to the packet configuration structure.
 *
 * This function processes received packets, validates their integrity using a CRC,
 * and extracts the control data if the packet is valid.
 */
void LRL_Packet_RX(packet_cfgType *packet)
{
	if(packet->byteReady)
	{
		packet->byteReady = 0;
		uint8_t _total_pkt_len, _remain_pkt_length, _crc_packet_len;
		unsigned short _temp_crc = 0;

		// The third byte of the packet holds the payload length.
		_total_pkt_len = packet->buffer[2] + 3;
		_remain_pkt_length = _total_pkt_len - packet->min_pkt_lenght;

		// Receive the rest of the packet's payload.
		if(_remain_pkt_length)
		{
			HAL_UART_Receive(packet->huart, &packet->buffer[3], _remain_pkt_length, 1);
		}

		// Validate the received packet using CRC.
		_crc_packet_len = _total_pkt_len - 2;
		LRL_Packet_UpdateCRC(0, packet->buffer, _crc_packet_len, _temp_crc);

		if(_temp_crc == ((packet->buffer[_total_pkt_len - 2]<<8)|(packet->buffer[_total_pkt_len - 1])))
		{
			packet->dataValid = 1;
		}
		else
		{
			packet->dataValid = 0;
		}

		// Extract control data if the packet is valid.
		if(packet->dataValid)
		{
			packet->vel_data.left_velocity = (int16_t)((packet->buffer[4] << 8) | packet->buffer[5]);
			packet->vel_data.right_velocity = (int16_t)((packet->buffer[6] << 8) | packet->buffer[7]);
		}

		// Clear the buffer and re-arm the UART receive interrupt for the next packet.
		memset(packet->buffer, 0, packet->max_pkt_lenght*sizeof(packet->buffer[0]));
		HAL_UART_Receive_IT(packet->huart, packet->buffer, packet->min_pkt_lenght);
	}
}

void LRL_Protocol_RX(packet_cfgType *packet)
{
	if(packet->buffer[1] == 0x00)
	{
		uint8_t full_length = packet->buffer[2] + 3;
		packet->byteReady = 1;
		// If we only received the header, get the rest
		if(full_length > packet->min_pkt_lenght)
		{
			HAL_UART_Receive_IT(packet->huart,
							   &packet->buffer[packet->min_pkt_lenght],
							   full_length - packet->min_pkt_lenght);
		}
		else
		{
			packet->byteReady = 1;
			memcpy(packet->data,
				 &packet->buffer[packet->min_pkt_lenght],
				 packet->buffer[2]); // Copy remaining bytes

			// Re-arm for next packet header
			HAL_UART_Receive_IT(packet->huart, packet->buffer, packet->min_pkt_lenght);
		}
	}
	else
	{
		// Second stage: full packet received or invalid header
		packet->byteReady = 1;
//            HAL_UART_Transmit(packet->huart, "HERE2", sizeof("HERE2"), 10);

		// CRITICAL: Re-arm for next packet header
		HAL_UART_Receive_IT(packet->huart, packet->buffer, packet->min_pkt_lenght);
	}
}

