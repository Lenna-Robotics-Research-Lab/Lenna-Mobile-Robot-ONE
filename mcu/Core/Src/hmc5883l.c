/*
 * hmc5883l.c
 *
 *  Created on: May 13, 2024
 *      Author: ErfanRzt
 */

#include "HMC5883L.h"

// Hold pointer to HAL I2C device
static I2C_HandleTypeDef * _hi2c;

uint8_t _buffer[6];
uint8_t _reg;

float heading;

float HMC5883L_Set_Declination(int16_t declination_degs , int16_t declination_mins, char declination_dir)
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

void HMC5883L_init(I2C_HandleTypeDef *hi2c)
{
    _hi2c = hi2c;

    // write CONFIG_A register
    _reg = 0x3C;
	HAL_I2C_Master_Transmit(_hi2c, HMC5883L_ADDRESS, &_reg, 1, DELAY_TIMEOUT);
	_reg = 0x10;
	HAL_I2C_Mem_Write(_hi2c, HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_A, 1, &_reg, 1, DELAY_TIMEOUT);

	// write CONFIG_B register
	_reg = 0x3C;
	HAL_I2C_Master_Transmit(_hi2c, HMC5883L_ADDRESS, &_reg, 1, DELAY_TIMEOUT);
	_reg = 0xE0;
	HAL_I2C_Mem_Write(_hi2c, HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_B, 1, &_reg, 1, DELAY_TIMEOUT);

	// write MODE register
	_reg = 0x3C;
	HAL_I2C_Master_Transmit(_hi2c, HMC5883L_ADDRESS, &_reg, 1, DELAY_TIMEOUT);
	_reg = 0x01;
	HAL_I2C_Mem_Write(_hi2c, HMC5883L_ADDRESS, 0x02, 1, &_reg, 1, DELAY_TIMEOUT);

	HAL_Delay(6);
}

void HMC5883L_readHeading(int16_t *x, int16_t *y, int16_t *z, float *headingDegrees)
{

	// write MODE register
	_reg = 0x3C;
	HAL_I2C_Master_Transmit(_hi2c, HMC5883L_ADDRESS, &_reg, 1, DELAY_TIMEOUT);
	_reg = 0x01;
	HAL_I2C_Mem_Write(_hi2c, HMC5883L_ADDRESS, 0x02, 1, &_reg, 1, DELAY_TIMEOUT);
	HAL_Delay(6);

	_reg = 0x3D;
	HAL_I2C_Master_Transmit(_hi2c, HMC5883L_ADDRESS, &_reg, 1, DELAY_TIMEOUT);
	HAL_I2C_Mem_Read(_hi2c, HMC5883L_ADDRESS, 0x03, 1, (uint8_t *)&_buffer, 6, DELAY_TIMEOUT);

	*x = (int16_t)((_buffer[0] << 8) | _buffer[1]);
	*y = (int16_t)((_buffer[4] << 8) | _buffer[5]);
	*z = (int16_t)((_buffer[2] << 8) | _buffer[3]);

	heading = atan2(*x, *y);

	heading += HMC5883L_Set_Declination(5, 3, 'E');

	// Correct for when signs are reversed.
    if(heading < 0)
    	heading += 2*M_PI;

    // Check for wrap due to addition of declination.
    if(heading > 2*M_PI)
    	heading -= 2*M_PI;

    // Convert radians to degrees for readability.
    *headingDegrees = heading * 180/M_PI;
}
