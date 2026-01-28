/**
 * @file rosserial.c
 * @brief This module handles the communication protocol for the robot.
 *
 * this function provides the functionality of rosserial on LCB of the robot
 *
 * @author Lenna Robotics Research Laboratory, Autonomous Systems Research Branch, Iran University of Science and Technology
 * @date January 27, 2026
 * @version 1.0
 * @link https://github.com/Lenna-Robotics-Research-Lab
 */

/*-------------------------- Includes ---------------------------------------- */

#include "rosserial.h"
#include "usart.h"
#include "stdlib.h"
#include "string.h"
//#include "pid.h"
//#include "imu.h"
//#include "odometry.h"

/*-------------------------- Code Body ---------------------------------------- */


void LRL_ROSSerial_Init(rosserial_cfgType *rosserial_handle)
{
	HAL_UART_Receive_IT(rosserial_handle->huart, rosserial_handle->rxbuffer, rosserial_handle->min_pkt_len);
}


void LRL_ROSSerial_Rx(rosserial_cfgType *rosserial_handle)
{
	if(!rosserial_handle->packetReceived)
	{
		if(rosserial_handle->rxbuffer[1] == 0xFE)
		{
			/*
			 * version control for rosserial for ROS noetic or melodic.
			 * this also would not allow invalid all zero packets pass
			 */

			uint8_t 	_checksum_calc = 0;
			uint16_t	_temp;

			_checksum_calc = 255 - ((rosserial_handle->rxbuffer[3]) + (rosserial_handle->rxbuffer[2]) % 256);

			if(_checksum_calc == rosserial_handle->rxbuffer[4])
			{
				_temp	= ((uint16_t)(rosserial_handle->rxbuffer[3]<<8)) | (uint16_t)(rosserial_handle->rxbuffer[2]);
				rosserial_handle->data_len	= _temp + 3; // this is the full packet length - the received initial 5
				rosserial_handle->pkt_len 	= _temp + 8;

				HAL_UART_Receive_IT(rosserial_handle->huart,
								   &rosserial_handle->rxbuffer[rosserial_handle->min_pkt_len],
								   rosserial_handle->data_len);

				rosserial_handle->headerValid = 1;
			}
			else
			{
				/*
				 * this would deal with unauthorized checksum headers
				 */
				rosserial_handle->headerValid = 0;
			}
			rosserial_handle->packetReceived = 1;
		}
		else
		{
			// Second stage: full packet received or invalid header
		}
	}
	else
	{
		HAL_UART_Receive_IT(rosserial_handle->huart, rosserial_handle->rxbuffer, rosserial_handle->min_pkt_len);
	}
}

void LRL_ROSSerial_Data_Handle(rosserial_cfgType *rosserial_handle)
{
    if(rosserial_handle->packetReceived)
    {
//	    HAL_UART_Transmit(rosserial_handle->huart, "Stage 1", 7, 10);
	    if(rosserial_handle->headerValid)
	    {
	    	while(rosserial_handle->rxbuffer[rosserial_handle->pkt_len - 1] == 0)
	    	{

	    	}
//	    	HAL_UART_Transmit(rosserial_handle->huart, "Stage 2", 7, 10);
		    memcpy(rosserial_handle->data, &rosserial_handle->rxbuffer[rosserial_handle->min_pkt_len], rosserial_handle->data_len);

		    uint8_t _data_sum = 0, _data_checksum = 0;

		    for(int i = 0; i<rosserial_handle->data_len; i++)
		    {
			    _data_sum 	+= rosserial_handle->data[i];
		    }

		    _data_checksum 	=  255 - (_data_sum % 256);

		    if(_data_checksum == rosserial_handle->data[rosserial_handle->data_len])
		    {
//		  	   HAL_UART_Transmit(rosserial_handle->huart, "Stage 3", 7, 10);
		  	   rosserial_handle->dataValid = 1;
		  	   _LRL_ROSSerial_Function(rosserial_handle);
		  	   memset(rosserial_handle->rxbuffer, 0 , sizeof(rosserial_handle->rxbuffer));
		  	   memset(rosserial_handle->data, 0 , sizeof(rosserial_handle->data));
		    }
		    else
		    {
		    	//something for second leg of data not being valid
		    }


		    rosserial_handle->headerValid = 0;
	    }
	    else
	    {
	    	// something to deal with data not being valid
	    }
	    rosserial_handle->packetReceived = 0;

    }
}

void _LRL_ROSSerial_Function(rosserial_cfgType *rosserial_handle)
{
	uint8_t _id;

	/*
	 * in rosserial the functions are passed by two bytes of ID however here the number of functions
	 * are limited to just 4. We did not want to change the overall packet frame of rosserial packet
	 * hence we used the packeting as it was but we only use the low byte as the indicator of
	 * our function ID.
	 */
	_id = rosserial_handle->data[0];
	if(_id == 0x00)
	{
		LRL_ROSSerial_Query(rosserial_handle);
	}
	else
	{
		rosserial_handle->dataValid = 0;

	}
}

void LRL_ROSSerial_Query(rosserial_cfgType *rosserial_handle)
{
	HAL_UART_Transmit(rosserial_handle->huart, rosserial_handle->rxbuffer, rosserial_handle->pkt_len, 10);
	rosserial_handle->dataValid = 0;
}

/*
void LRL_ROSSerial_ReadAll(rosserial_cfgType *rosserial_handle, odom_cfgType *odom, imu_statetype *imu)
{
	// Start of packet markers.
	rosserial_handle->txbuffer[0] 	= 0xFF;
	rosserial_handle->txbuffer[1] 	= 0xFE;

	// Message data length.
	rosserial_handle->txbuffer[2] 	= 0x1A; // low byte
	rosserial_handle->txbuffer[3] 	= 0x00; // high byte

	rosserial_handle->txbuffer[4] 	= 0xFF - 0x1A; // checksum for header validity

	rosserial_handle->txbuffer[5] 	= 0x01; // ID low byte
	rosserial_handle->txbuffer[6] 	= 0x00; // ID high byte
	// Pack odometry data.
	rosserial_handle->txbuffer[7] 	= (uint8_t)(odom->vel.left >> 8);
	rosserial_handle->txbuffer[8] 	= (uint8_t)(odom->vel.left & 0x00FF);
	rosserial_handle->txbuffer[9] 	= (uint8_t)(odom->vel.right >> 8);
	rosserial_handle->txbuffer[10] 	= (uint8_t)(odom->vel.right & 0x00FF);
	rosserial_handle->txbuffer[11] 	= (uint8_t)(odom->dist.left >> 8);
	rosserial_handle->txbuffer[12] 	= (uint8_t)(odom->dist.left & 0x00FF);
	rosserial_handle->txbuffer[13] 	= (uint8_t)(odom->dist.right >> 8);
	rosserial_handle->txbuffer[14] 	= (uint8_t)(odom->dist.right & 0x00FF);

	// Pack IMU accelerometer data.
	rosserial_handle->txbuffer[15] 	= (uint8_t)(imu->accel.x_calibrated >> 8);
	rosserial_handle->txbuffer[16] 	= (uint8_t)(imu->accel.x_calibrated & 0x00FF);
	rosserial_handle->txbuffer[17] 	= (uint8_t)(imu->accel.y_calibrated >> 8);
	rosserial_handle->txbuffer[18] 	= (uint8_t)(imu->accel.y_calibrated & 0x00FF);
	rosserial_handle->txbuffer[19] 	= (uint8_t)(imu->accel.z_calibrated >> 8);
	rosserial_handle->txbuffer[20] 	= (uint8_t)(imu->accel.z_calibrated & 0x00FF);

	// Pack IMU gyroscope data.
	rosserial_handle->txbuffer[21] 	= (uint8_t)(imu->gyro.x_calibrated >> 8);
	rosserial_handle->txbuffer[22] 	= (uint8_t)(imu->gyro.x_calibrated & 0x00FF);
	rosserial_handle->txbuffer[23] 	= (uint8_t)(imu->gyro.y_calibrated >> 8);
	rosserial_handle->txbuffer[24] 	= (uint8_t)(imu->gyro.y_calibrated & 0x00FF);
	rosserial_handle->txbuffer[25] 	= (uint8_t)(imu->gyro.z_calibrated >> 8);
	rosserial_handle->txbuffer[26] 	= (uint8_t)(imu->gyro.z_calibrated & 0x00FF);

	// Pack IMU angular position data.
	rosserial_handle->txbuffer[27] 	= (uint8_t)(imu->angle.x >> 8);
	rosserial_handle->txbuffer[28] 	= (uint8_t)(imu->angle.x & 0x00FF);
	rosserial_handle->txbuffer[29] 	= (uint8_t)(imu->angle.y >> 8);
	rosserial_handle->txbuffer[30] 	= (uint8_t)(imu->angle.y & 0x00FF);

	// Pack magnetometer heading data.
	rosserial_handle->txbuffer[31] 	= (uint8_t)(imu->mag.heading >> 8);
	rosserial_handle->txbuffer[32] 	= (uint8_t)(imu->mag.heading & 0x00FF);

	uint8_t _checksum = 0;

	for(int i = 0; i < (0x02 + 0x1A) ; i++)
	{
		_checksum += rosserial_handle->txbuffer[i+5];
	}

	rosserial_handle->txbuffer[33] 	= _checksum;

	// Transmit the complete packet.
	HAL_UART_Transmit_IT(rosserial_handle->huart, rosserial_handle->txbuffer, 33);
	memset(rosserial_handle->txbuffer, 0 , sizeof(rosserial_handle->txbuffer));
	rosserial_handle->dataValid = 0;
}

void LRL_ROSSerial_SetPID(rosserial_cfgType *rosserial_handle, pid_cfgType *pid_cfg)
{
	pid_cfg->kp = rosserial_handle->data[2];
	pid_cfg->ki = rosserial_handle->data[3];
	pid_cfg->kd = rosserial_handle->data[4];

	HAL_UART_Transmit(rosserial_handle->huart, rosserial_handle->rxbuffer, rosserial_handle->pkt_len, 1);
	rosserial_handle->dataValid = 0;

}

void LRL_ROSSerial_GetPID(rosserial_cfgType *rosserial_handle, pid_cfgType *pid_cfg)
{
	// Start of packet markers.
	rosserial_handle->txbuffer[0] 	= 0xFF;
	rosserial_handle->txbuffer[1] 	= 0xFE;

	// Message data length.
	rosserial_handle->txbuffer[2] 	= 0x03; // low byte
	rosserial_handle->txbuffer[3] 	= 0x00; // high byte

	rosserial_handle->txbuffer[4] 	= 0xFF - 0x03;

	rosserial_handle->txbuffer[5] 	= 0x02;
	rosserial_handle->txbuffer[6] 	= 0x00;

	rosserial_handle->txbuffer[7] 	= pid_cfg->kp;
	rosserial_handle->txbuffer[8] 	= pid_cfg->ki;
	rosserial_handle->txbuffer[9] 	= pid_cfg->kd;

	uint8_t _checksum = 0;

	for(int i = 0; i < (0x03 + 0x02) ; i++)
	{
		_checksum += rosserial_handle->txbuffer[i+5];
	}

	rosserial_handle->txbuffer[10]	= _checksum;

	HAL_UART_Transmit_IT(rosserial_handle->huart, rosserial_handle->txbuffer, 10);
	memset(rosserial_handle->txbuffer, 0 , sizeof(rosserial_handle->txbuffer));


}
*/
