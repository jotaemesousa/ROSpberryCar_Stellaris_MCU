/*
 * uart_config.cpp
 *
 *  Created on: 15 de Dez de 2012
 *      Author: Jo�o
 */

#include "rc_cmds.h"
#include "car.h"
#include "timer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct rc_cmds ferrari288gto;
struct rc_parameters ferrari288gto_param;

void default_rc(void)
{
	ferrari288gto.Drive = 0;
	ferrari288gto.Steer = 0;
	ferrari288gto.new_msg = 0;
	ferrari288gto.ptr = 0;
	ferrari288gto.first_char = 0;
	ferrari288gto.size = 0;
	ferrari288gto.size_msg = 0;
	ferrari288gto.last_millis = 0;
}

void add_rx_char(char c)
{
	if( ferrari288gto.ptr >= MAX_BUFFER_SIZE)
	{
		ferrari288gto.ptr = 0;
	}

	if(ferrari288gto.ptr == ferrari288gto.first_char && ferrari288gto.size)
	{
		ferrari288gto.first_char++;

		if(ferrari288gto.first_char >= MAX_BUFFER_SIZE)
		{
			ferrari288gto.first_char = 0;
		}
	}

	ferrari288gto.rx[ferrari288gto.ptr++] = c;

	if(c == LAST_CHAR)
	{
		ferrari288gto.new_msg = 1;
	}

	ferrari288gto.size++;
}

void add_rx_str(char *s, unsigned int n_char)
{
	if(n_char > 0)
	{
		int i = 0;
		for (i = 0; i < n_char; ++i)
		{
			add_rx_char(s[i]);
		}
	}
}

void parseMessage(void)
{
	int vals[5], c = 0, i_vals = 0;
	char temp_str[10];
	memset(temp_str,0,10);

	short state = 0, offset = 0, counter = 0;

	int i;
	for (i = ferrari288gto.first_char; i < (ferrari288gto.first_char + ferrari288gto.size); ++i)
	{
		counter++;
		if(i == MAX_BUFFER_SIZE)
		{
			offset -= MAX_BUFFER_SIZE;
		}
		switch (state)
		{
		case 0:
			if(ferrari288gto.rx[i + offset] == FIRST_CHAR)
			{
				c = 0;
				state = 1;

			}
			break;

		case 1:
			if(ferrari288gto.rx[i + offset] == LAST_CHAR)
			{
				if(c)
				{
					vals[i_vals++] = atoi(temp_str);
				}
				c = 0;
				ferrari288gto.first_char = i + offset + 1;

				if(ferrari288gto.first_char >= MAX_BUFFER_SIZE)
				{
					ferrari288gto.first_char = 0;
				}
				ferrari288gto.size -= counter;
				counter = 0;

				// do stuff
				// set PWM
				if(i_vals == 3 && vals[0] == 0 )
				{
					ferrari288gto.Drive = vals[1];
					ferrari288gto.Steer = vals[2];
					ferrari288gto.last_millis = millis();
				}
			}
			else if(ferrari288gto.rx[i + offset] == ' ')
			{
				if(c)
				{
					vals[i_vals++] = atoi(temp_str);
				}
				c = 0;
				memset(temp_str,0,10);
			}
			else
			{
				temp_str[c++] = ferrari288gto.rx[i + offset];
			}

			break;
		}
	}
}

