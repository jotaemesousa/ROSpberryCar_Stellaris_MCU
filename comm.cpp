/*
 * comm.cpp
 *
 *  Created on: Nov 23, 2013
 *      Author: joao
 */
#include "comm.h"

extern INA226 power_meter;
ROSCASDataFromRASPI struct_to_receive;
ROSCASDataToRASPI struct_to_send;

void initSerialComm(unsigned long ulBaud)
{
	//
	// Enable GPIO port A which is used for UART0 pins.
	// TODO: change this to whichever GPIO port you are using.
	//
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
	// Select the alternate (UART) function for these pins.
	// TODO: change this to select the port/pin you are using.
	//
	MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// Initialize the UART for console I/O.
	//
	UARTStdioInitExpClk(0,ulBaud);

	UARTEchoSet(false);
}

void initSPIComm(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	GPIOPinTypeSSI(GPIO_PORTA_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_2|GPIO_PIN_3); //SPI0 output
	SSIDisable(SSI0_BASE);
	SSIConfigSetExpClk(SSI0_BASE,SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,SSI_MODE_SLAVE, 5000,8);
	SSIEnable(SSI0_BASE);

	SSIIntRegister(SSI0_BASE, SSIIntHandler);
	SSIIntClear(SSI0_BASE, SSI_RXTO /*| SSI_RXFF*/);
	SSIIntEnable(SSI0_BASE, SSI_RXTO /*| SSI_RXFF*/);
	IntEnable(INT_SSI0);
}

void serial_receive(void)
{
	char inChar;                        // temporary input char
	static char inData_[100];
	static int index_ = 0;
	static uint8_t receiving_cmd = 0;

	//        UARTprintf("RX available %d \n", UARTRxBytesAvail());
	while(UARTRxBytesAvail() > 0)        //if bytes available at Serial port
	{
		//        	UARTprintf(" PARSE \n");
		inChar = UARTgetc();                // read from port

		if(index_ < 98)                // read up to 98 bytes
		{
			if(inChar == ':')
			{
				if(receiving_cmd == 0)
				{
					receiving_cmd = 1;

					inData_[index_] = inChar;        // store char
					++index_;                        // increment index
					inData_[index_] = 0;                // just to finish string
				}
			}
			else if(receiving_cmd == 1)
			{
				inData_[index_] = inChar;        // store char
				++index_;                        // increment index
				inData_[index_] = 0;                // just to finish string

			}
		}
		else                        // put end char ";"
		{
			index_ = 0;

		}

		if(receiving_cmd)
		{
			if(inChar == ';')
			{                        // if the last char is ";"

				if(!serial_parse(inData_))        //parse data
				{
					receiving_cmd = 0;
					index_ = 0;

					inData_[index_] = 0;
				}
				else
				{
					// no parse action
					receiving_cmd = 0;
					index_ = 0;

					inData_[index_] = 0;
				}
			}

		}
	}

}

uint8_t serial_parse(char *buffer)
{
	int d1 = 0, d2 = 0, d3 = 0;
	//        UARTprintf(" PARSE \n");
	if(!ustrncmp(buffer, ":V ",3))
	{
		sscanf(buffer, ":V %d %d;", &d1, &d2);
		UARTprintf(" echo d1 = %d, d2 = %d\n", d1, d2);

		velocity_pid.setNewReference((float)d1,d2);



	}
	if(!ustrncmp(buffer, ":P ",3))
	{
		sscanf(buffer, ":P %d %d %d;", &d1, &d2, &d3);
		UARTprintf(" echo p = %d, i = %d, d = %d\n", d1, d2, d3);
		velocity_pid.setGains((float)d1/10.0,(float)d2/10.0,(float)d3/10.0);
	}

	return 1;
}

#ifdef __cplusplus
extern "C"
{
#endif
void SSIIntHandler(void)
{
	SSIIntClear(SSI0_BASE, SSI_RXTO);

	static int8_t bytes_left_to_send = 0;
	static int8_t n_bytes_received = 0;
	static SSI_Interrupt_State state_interrupt = RECEIVING_STATE;
	static unsigned long received_byte = 0, buffer_index = 0;

	//MAP_GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_7,GPIO_PIN_7);

	uint8_t *pointer_received = (uint8_t *)&struct_to_receive;
	uint8_t *pointer_send = (uint8_t *)&struct_to_send;

	switch(state_interrupt)
	{
	case RECEIVING_STATE:

		// Receives up to 4 bytes (minimum bytes to trigger the interrupt
		if(SSIDataGetNonBlocking(SSI0_BASE, &received_byte))
		{

			*(pointer_received + n_bytes_received) = received_byte;
			buffer_index++;
			n_bytes_received++;
		}

		if(n_bytes_received >= 3)
		{
			// update vars


			if(struct_to_receive.cmd & ASK_DATA_BIT)
			{
				if(struct_to_receive.cmd & ASK_FIRMWARE_BIT)
				{
					struct_to_send.left_encoder_count = STELLARIS_VERSION;
					struct_to_send.right_encoder_count = -STELLARIS_VERSION;
					struct_to_send.battery_voltage  = power_meter.get_bus_voltage()/100;
					struct_to_send.battery_current = power_meter.get_bus_current()/10;
					struct_to_send.cmd_back = struct_to_receive.cmd;
				}
				else
				{
					int32_t left = 0, right = 0;
					encoder_read_reset(&left,&right,1);
					struct_to_send.left_encoder_count = left;
					struct_to_send.right_encoder_count = right;
					struct_to_send.battery_voltage  = power_meter.get_bus_voltage()/100;
					struct_to_send.battery_current = power_meter.get_bus_current()/10;
					struct_to_send.cmd_back = struct_to_receive.cmd;
				}
				state_interrupt = SENDING_AFTER_RECEIVING;
				bytes_left_to_send = sizeof( ROSCASDataToRASPI);
				SSIDataPutNonBlocking(SSI0_BASE, pointer_send[sizeof( ROSCASDataToRASPI) - bytes_left_to_send]);
				bytes_left_to_send--;
			}
			else
			{
				state_interrupt = RECEIVING_STATE;
				n_bytes_received = 0;
				buffer_index = 0;
				bytes_left_to_send = 0;
			}
		}



		break;

	case SENDING_AFTER_RECEIVING:

		SSIDataGetNonBlocking(SSI0_BASE, &received_byte);

		if(bytes_left_to_send > 0)
		{
			SSIDataPutNonBlocking(SSI0_BASE, pointer_send[sizeof( ROSCASDataToRASPI) - bytes_left_to_send]);
			bytes_left_to_send--;
		}
		else
		{
			state_interrupt = RECEIVING_STATE;
			n_bytes_received = 0;
			buffer_index = 0;
		}


		break;
	}



	//MAP_GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_7,0);

}
#ifdef __cplusplus
}
#endif
