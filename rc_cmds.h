/*
 * uart_config.h
 *
 *  Created on: 15 de Dez de 2012
 *      Author: Jo�o
 */

#ifndef UART_CONFIG_H_
#define UART_CONFIG_H_


//#include "car.h"

#define MAX_BUFFER_SIZE			50
#define FIRST_CHAR				':'
#define LAST_CHAR				';'
#define THRESHOLD_BETWEEN_MSG	300L	//ms

// Remote settings
#define REMOTE_MIN_THROTLE		637
#define REMOTE_ZERO_THROTLE		748
#define REMOTE_MAX_THROTLE		853
#define REMOTE_MIN_STEER		587
#define REMOTE_ZERO_STEER		697
#define REMOTE_MAX_STEER		794




struct rc_cmds
{
	int Drive;							// -100 : 100
	unsigned int Steer;					// -100 : 100
	char new_msg, size_msg;
	char rx[MAX_BUFFER_SIZE];
	unsigned short ptr, first_char, size;
	unsigned long int last_millis;
};

struct rc_parameters
{
	unsigned int battery_voltage;	// mV
	char head_lights, tail_lights, left_neon, right_neon;
};

extern struct rc_cmds ferrari288gto;
extern struct rc_parameters ferrari288gto_param;

void default_uart(void);
void add_rx_char(char );
void add_rx_str(char *, unsigned int );
void parseMessage(void);

#endif /* UART_CONFIG_H_ */
