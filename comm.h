/*
 * comm.h
 *
 *  Created on: Nov 23, 2013
 *      Author: joao
 */

#ifndef COMM_H_
#define COMM_H_

#include "common_includes.h"
#include "Utilities/pid.h"
#include "Utilities/Encoder.h"
#include "Utilities/INA226.h"
#include "Utilities/servo.h"
#include <stdio.h>
#include "defined_macros.h"

typedef struct ROSCASDataFromRASPI_
{
	int8_t v_linear;
	int8_t v_angular;
	uint8_t cmd;
}ROSCASDataFromRASPI;

typedef struct ROSCASDataToRASPI_
{
	int32_t left_encoder_count;
	int32_t right_encoder_count;
	uint8_t battery_voltage;
	uint8_t battery_current;
	uint8_t cmd_back;
}ROSCASDataToRASPI;

typedef enum
{
	RECEIVING_STATE = 0, SENDING_AFTER_RECEIVING
}SSI_Interrupt_State;


extern void drive_pwm(int pwm, bool brake);
extern pid velocity_pid;


void initSerialComm(unsigned long ulBaud);
void initSPIComm(void);
void communication_update_function(void);

#if UART_SERIAL_PARSE_SSCANF
void serial_receive(void);
uint8_t serial_parse(char *buffer);
#endif

#ifdef __cplusplus
extern "C"
{
#endif
extern uint32_t millis();
void SSIIntHandler(void);
#ifdef __cplusplus
}
#endif

#endif /* COMM_H_ */
