extern "C" {
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include <stdint.h>

#define SYSTICKS_PER_SECOND     1000

static unsigned long milliSec = 0;

void SysTickHandler()
{
	milliSec++;
}

uint32_t millis(){
	return milliSec;
}





void InitConsole(void)
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
	UARTStdioInit(0);
}


}

#include "rf24/RF24.h"
static unsigned long ulClockMS=0;

int main(void)
{
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |SYSCTL_XTAL_12MHZ); //50MHZ

	//
	// Enable peripherals to operate when CPU is in sleep.
	//
	MAP_SysCtlPeripheralClockGating(true);

	//
	// Configure SysTick to occur 1000 times per second, to use as a time
	// reference.  Enable SysTick to generate interrupts.
	//
	MAP_SysTickPeriodSet(MAP_SysCtlClockGet() / SYSTICKS_PER_SECOND);
	MAP_SysTickIntEnable();
	MAP_SysTickEnable();

	//
	// Get the current processor clock frequency.
	//
	ulClockMS = MAP_SysCtlClockGet() / (3 * 1000);

	InitConsole();

	RF24 radio = RF24();

	// Radio pipe addresses for the 2 nodes to communicate.
	const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

	// The debug-friendly names of those roles
	const char* role_friendly_name[] = { "invalid", "Ping out", "Pong back"};

	// The various roles supported by this sketch
	typedef enum { role_ping_out = 1, role_pong_back } role_e;

	// The role of the current running sketch
	role_e role;
	role = role_ping_out;

	UARTprintf("\n\rRF24/examples/pingpair/\n\r");
	UARTprintf("ROLE: %s\n\r",role_friendly_name[role]);

	//
	// Setup and configure rf radio
	//
	radio.begin();

	// optionally, increase the delay between retries & # of retries
	radio.setRetries(15,15);

	// optionally, reduce the payload size.  seems to
	// improve reliability
	radio.setPayloadSize(8);

	radio.setDataRate(RF24_2MBPS);

	//
	// Open pipes to other nodes for communication
	//

	// This simple sketch opens two pipes for these two nodes to communicate
	// back and forth.
	// Open 'our' pipe for writing
	// Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)

	if ( role == role_ping_out )
	{
		radio.openWritingPipe(pipes[0]);
		radio.openReadingPipe(1,pipes[1]);
	}
	else
	{
		radio.openWritingPipe(pipes[1]);
		radio.openReadingPipe(1,pipes[0]);
	}

	//
	// Start listening
	//

	radio.startListening();

	//
	// Dump the configuration of the rf unit for debugging
	//

	radio.printDetails();



	while (1) {

		//
		// Ping out role.  Repeatedly send the current time
		//

		if (role == role_ping_out)
		{
			// First, stop listening so we can talk.
			radio.stopListening();

			// Take the time, and send it.  This will block until complete
			unsigned long time = millis();
			UARTprintf("Now sending %u...",time);
			bool ok = radio.write( &time, sizeof(unsigned long) );

			if (ok)
				UARTprintf("ok...");
			else
				UARTprintf("failed.\n\r");

			// Now, continue listening
			radio.startListening();

			// Wait here until we get a response, or timeout (250ms)
			unsigned long started_waiting_at = millis();
			bool timeout = false;
			while ( ! radio.available() && ! timeout )
				if (millis() - started_waiting_at > 200 )
					timeout = true;

			// Describe the results
			if ( timeout )
			{
				UARTprintf("Failed, response timed out.\n\r");
			}
			else
			{
				// Grab the response, compare, and send to debugging spew
				unsigned long got_time;
				radio.read( &got_time, sizeof(unsigned long) );

				// Spew it
				UARTprintf("Got response %u, round-trip delay: %u\n\r",got_time,millis()-got_time);
			}

			// Try again 1s later
			MAP_SysCtlDelay(ulClockMS*1000);
		}

		//
		// Pong back role.  Receive each packet, dump it out, and send it back
		//

		if ( role == role_pong_back )
		{
			// if there is data ready
			if ( radio.available() )
			{
				// Dump the payloads until we've gotten everything
				unsigned long got_time;
				bool done = false;
				while (!done)
				{
					// Fetch the payload, and see if this was the last one.
					done = radio.read( &got_time, sizeof(unsigned long) );

					// Spew it
					UARTprintf("Got payload %u...",got_time);

					// Delay just a little bit to let the other unit
					// make the transition to receiver
					MAP_SysCtlDelay(ulClockMS*20);
				}

				// First, stop listening so we can talk
				radio.stopListening();

				// Send the final one back.
				radio.write( &got_time, sizeof(unsigned long) );
				UARTprintf("Sent response.\n\r");

				// Now, resume listening so we catch the next packets.
				radio.startListening();
			}
		}

	}

}

