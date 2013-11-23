/*
 * comm.h
 *
 *  Created on: Nov 23, 2013
 *      Author: joao
 */

#ifndef COMM_H_
#define COMM_H_

#include <stdint.h>
#include <utils/uartstdio.h>
#include <utils/ustdlib.h>
#include <string.h>
#include <stdio.h>

void serial_receive(void);
uint8_t serial_parse(char *buffer);

#endif /* COMM_H_ */
