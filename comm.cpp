/*
 * comm.cpp
 *
 *  Created on: Nov 23, 2013
 *      Author: joao
 */
#include "comm.h"

extern void drive_pwm(int pwm, bool brake);

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
        int d1 = 0, d2 = 0;
//        UARTprintf(" PARSE \n");
        if(!ustrncmp(buffer, ":V ",3))
        {
        	sscanf(buffer, ":V %d %d;", &d1, &d2);
        	UARTprintf(" echo d1 = %d, d2 = %d\n", d1, d2);

        	drive_pwm(d1,0);
        }
        return 1;
}
