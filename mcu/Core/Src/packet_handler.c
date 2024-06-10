/*
 * packet_handler.c
 *
 *  Created on: Jun 5, 2024
 *      Author: arian
 */
#include "odometry.h"
#include "packet_handler.h"
#include "usart.h"
#include "stdlib.h"

uint8_t _ack_data[10] = {0x4C, 0x45, 0x4E, 0x4E, 0x41};

void LRL_UpdateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size, unsigned short crc_final )
{
  uint16_t i, j;
  static const uint16_t crc_table[256] = { 0x0000,
    0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
    0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
    0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
    0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
    0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
    0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
    0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
    0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
    0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
    0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
    0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
    0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
    0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
    0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
    0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
    0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
    0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
    0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
    0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
    0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
    0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
    0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
    0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
    0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
    0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
    0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
    0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
    0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
    0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
    0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
    0x820D, 0x8207, 0x0202 };

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }
  crc_final = crc_accum;
}


void LRL_txPacket(packet_cfgType *packet,odom_cfgType *odom)
{
	unsigned short _tmp_crc;

	packet->buffer[0] = 0xFF;
	packet->buffer[1] = 0xFF;

	packet->buffer[2] = 0x01;
	packet->buffer[3] = 0x1A; //this is the size of the bytes

	packet->buffer[4] = (uint8_t)(odom->mag.heading >> 8);
	packet->buffer[5] = (uint8_t)(odom->mag.heading & 0x00FF);

	packet->buffer[6] = (uint8_t)(odom->vel.left >> 8);
	packet->buffer[7] = (uint8_t)(odom->vel.left & 0x00FF);

	packet->buffer[8] = (uint8_t)(odom->vel.right >> 8);
	packet->buffer[9] = (uint8_t)(odom->vel.right & 0x00FF);

	packet->buffer[10] = (uint8_t)(odom->accel.x >> 8);
	packet->buffer[11] = (uint8_t)(odom->accel.x & 0x00FF);

	packet->buffer[12] = (uint8_t)(odom->accel.y>> 8);
	packet->buffer[13] = (uint8_t)(odom->accel.y & 0x00FF);

	packet->buffer[14] = (uint8_t)(odom->gyro.x >> 8);
	packet->buffer[15] = (uint8_t)(odom->gyro.x & 0x00FF);

	packet->buffer[16] = (uint8_t)(odom->gyro.y >> 8);
	packet->buffer[17] = (uint8_t)(odom->gyro.y & 0x00FF);

	packet->buffer[18] = (uint8_t)(odom->gyro.z >> 8);
	packet->buffer[19] = (uint8_t)(odom->gyro.z & 0x00FF);

	packet->buffer[20] = (uint8_t)(odom->dist.left >> 8);
	packet->buffer[21] = (uint8_t)(odom->dist.left & 0x00FF);

	packet->buffer[22] = (uint8_t)(odom->dist.right >> 8);
	packet->buffer[23] = (uint8_t)(odom->dist.right & 0x00FF);

	LRL_UpdateCRC(0, &packet->buffer, 24,_tmp_crc);

	packet->buffer[24] = (uint8_t)(_tmp_crc >> 8);
	packet->buffer[25] = (uint8_t)(_tmp_crc & 0x00FF);

	HAL_UART_Transmit_IT(packet->huart, packet->buffer,26);
//	HAL_UART_Transmit(&huart1, _buffer, 26,10);

}


void LRL_Packet_Init(packet_cfgType *packet)
{
	HAL_UART_Receive_IT(packet->huart, packet->buffer, packet->min_pkt_lenght);
}

void LRL_rxPacket(packet_cfgType *packet)
{
	if(packet->rx_byteReady)
	{
		packet->rx_byteReady = 0;
		uint8_t total_pkt_len,remain_pkt_length;

		unsigned short temp_crc;
		// length of the package is sent in the third byte
		total_pkt_len = packet->buffer[2] + 3;

//		HAL_UART_Receive_IT(packet->huart, &packet->buffer[packet->min_pkt_lenght], packet->buffer[2]);
		remain_pkt_length = total_pkt_len - packet->min_pkt_lenght;

		if(remain_pkt_length)
		{
//			HAL_UART_Transmit(&huart1, "HELLO", sizeof("HELLO"), 10);
			HAL_UART_Receive(packet->huart, &packet->buffer[3], remain_pkt_length,1);
		}

		LRL_UpdateCRC(0, packet->buffer, total_pkt_len - 2 ,temp_crc);

		if(temp_crc == ((packet->buffer[total_pkt_len - 2]<<8)|(packet->buffer[total_pkt_len - 1])))
		{
			packet->rx_dataValid = 1;
		}
		else
		{
			packet->rx_dataValid = 0;
		}
	    packet->data.left_velocity = (int16_t)((packet->buffer[4] << 8) | packet->buffer[5]);
	    packet->data.right_velocity = (int16_t)((packet->buffer[6] << 8) | packet->buffer[7]);
//		HAL_UART_Transmit(&huart1, packet->buffer, 3+packet->buffer[2], 10);
		memset(packet->buffer, 0, packet->max_pkt_lenght*sizeof(packet->buffer[0]));
		HAL_UART_Receive_IT(packet->huart, packet->buffer, packet->min_pkt_lenght);

	}
}

void LRL_handShake(packet_cfgType *packet)
{
	int _out;
	while(_out != 1)
	{
		HAL_UART_Transmit(packet->huart,&_ack_data,5,10);
		HAL_UART_Receive(packet->huart, &packet->ack, 1,10);
		if(packet->ack)
		{
			_out = 1;
		}
		HAL_Delay(500);
	}
	HAL_UART_Transmit(&huart1,"Surprise motherfuckers",sizeof("Surprise motherfuckers"),10);

}

