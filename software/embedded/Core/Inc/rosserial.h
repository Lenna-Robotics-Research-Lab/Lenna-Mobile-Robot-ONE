/*
 * rosserial.h
 *
 *  Created on: Jan 27, 2026
 *      Author: arian
 */

#ifndef INC_ROSSERIAL_H_
#define INC_ROSSERIAL_H_

#include "usart.h"
#include "pid.h"
#include "imu.h"
#include "odometry.h"

#define PACKET_HEADER_LENGTH	5	/**< Minimum expected packet length. */
#define MAX_PACKET_LENGTH		255	/**< Maximum buffer size for a packet. */
#define MAX_DATA_LENGTH			248

/* Type Definitions ----------------------------------------------------------*/


/**
 * @struct rosserial_cfgType
 * @brief Stores configuration and state data for the packet handler module.
 */
typedef struct
{
	UART_HandleTypeDef *huart;						/**< Pointer to the UART handle for communication. */
	uint8_t				min_pkt_len;				/**< Minimum packet length. */
	uint8_t				max_pkt_len;				/**< Maximum packet length. */
	uint8_t				dataValid;					/**< Flag to indicate if received data is valid (checksum passed). */
	uint8_t				packetReceived;				/**< Flag to indicate a new packet has been received. */
	uint8_t				headerValid;				/**< Flag to indicate that the received header is valid. */
	uint16_t			data_len;					/**< Length of the packet after the header. */
	uint16_t			pkt_len;					/**< Length of the whole packet. */
	uint8_t 			rxbuffer[MAX_PACKET_LENGTH];	/**< Buffer to store incoming and outgoing data. */
	uint8_t				data[MAX_DATA_LENGTH];						/**< data packet inside the buffer >*/
	uint8_t 			txbuffer[MAX_PACKET_LENGTH];
} rosserial_cfgType;

/* Function Prototypes -------------------------------------------------------*/
void LRL_ROSSerial_Init(rosserial_cfgType *rosserial_handle, UART_HandleTypeDef *huart);

void LRL_ROSSerial_Rx(rosserial_cfgType *rosserial_handle);

void LRL_ROSSerial_Data_Handle(rosserial_cfgType *rosserial_handle);

void _LRL_ROSSerial_Function(rosserial_cfgType *rosserial_handle);

void LRL_ROSSerial_ReadAll(rosserial_cfgType *rosserial_handle, odom_cfgType *odom, imu_statetype *imu);

void LRL_ROSSerial_Query(rosserial_cfgType *rosserial_handle);

void LRL_ROSSerial_SetPID(rosserial_cfgType *rosserial_handle, pid_cfgType *pid_cfg);

void LRL_ROSSerial_getPID(rosserial_cfgType *rosserial_handle, pid_cfgType *pid_cfg);

#endif /* INC_ROSSERIAL_H_ */
