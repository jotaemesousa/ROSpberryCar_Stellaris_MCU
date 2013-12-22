/*
 * defined_macros.h
 *
 *  Created on: Dec 22, 2013
 *      Author: joao
 */

#ifndef DEFINED_MACROS_H_
#define DEFINED_MACROS_H_

//Firmware
#define STELLARIS_VERSION	10

// SSI
#define BLINKY_BIT          0x80
#define ASK_DATA_BIT        0x40
#define HBRIDGEMODE_BIT     0x20
#define ASK_FIRMWARE_BIT    0x10
#define STARTSTOP_BIT       0x08

// use sensors
#define USE_I2C
#define USE_INA226
//#define USE_NRF24

#define SYSTICKS_PER_SECOND     1000

// HEARTBEAT
#define TICKS_PER_SECOND 		1000

// servo and drive
#define MAX_PWM_STEER			100
#define MAX_PWM_DRIVE			10000

// debug
#define DEBUG
#define DEBUG_CMD

#endif /* DEFINED_MACROS_H_ */
