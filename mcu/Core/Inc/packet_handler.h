/*
 * packet_handler.h
 *
 *  Created on: Jun 5, 2024
 *      Author: Lenna Robotics Research Laboratory
 *      		Autonomous Systems Research Branch
 *				Iran University of Science and Technology
 *		GitHub:	github.com/Lenna-Robotics-Research-Lab
 */

#ifndef INC_PACKET_HANDLER_H_
#define INC_PACKET_HANDLER_H_

#define MIN_PACKET_LENGTH	3
#define MAX_PACKET_LENGTH	144

typedef struct
{
	int16_t right_velocity;
	int16_t left_velocity;
} packet_data;

typedef struct
{
	UART_HandleTypeDef *huart;
	uint8_t				min_pkt_lenght;
	uint8_t				max_pkt_lenght;
	uint8_t				rx_dataValid;
	uint8_t				rx_byteReady;
	uint8_t 			buffer[144];
	uint8_t				ack;
	packet_data			data;
} packet_cfgType;


void LRL_UpdateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size, unsigned short crc_final );
void LRL_Packet_Init(packet_cfgType *packet);
void LRL_rxPacket(packet_cfgType *packet);
void LRL_handShake(packet_cfgType *packet);



#endif /* INC_PACKET_HANDLER_H_ */
