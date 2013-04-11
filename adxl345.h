/*
 * adxl345.h
 *
 *  Created on: Jan 27, 2013
 *      Author: joao
 */

#ifndef ADXL345_H_
#define ADXL345_H_

#include "stdint.h"

#define ADXL345_ADDR			0x53
#define ADXL345_WRITE			0xA6
#define ADXL345_READ			0xA7
#define POWER_REGISTER			0x2D
#define DATA_FORMAT_REGISTER	0x31
#define BW_RATE_REGISTER		0x2C
#define DATA_X_LSB_REGISTER		0x32
#define DATA_X_MSB_REGISTER		0x33
#define DATA_Y_LSB_REGISTER		0x34
#define DATA_Y_MSB_REGISTER		0x35
#define DATA_Z_LSB_REGISTER		0x36
#define DATA_Z_MSB_REGISTER		0x37

struct Accelerometer
{
	int acc_x;
	int acc_y;
	int acc_z;

};


void init_adlx345(void);
char get_dev_id_adxl345(void);
int get_acc_x(void);
int get_acc_y(void);
int get_acc_z(void);
void get_acc(struct Accelerometer * acc);
void ADXL345_GetRange(void);
int TwoComplement2ModSig_16bit(uint16_t a);

#endif /* ADXL345_H_ */
