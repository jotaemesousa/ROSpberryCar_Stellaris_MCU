#include "common_includes.h"
#include "defined_macros.h"
#include "comm.h"

#include "Utilities/servo.h"
#include "remote_defines.h"
#include "Utilities/pid.h"
#include "Utilities/gpio_pwm_lights.h"
#include "rf24/RF24.h"

#define INA226_ALERT_PIN		GPIO_PIN_0
#define INA226_ALERT_PORT		GPIO_PORTE_AHB_BASE

extern "C"
{
void SysTickHandler();
uint32_t millis();

static unsigned long milliSec = 0;
}

static unsigned long ulClockMS=0;
double map_value(double x, double in_min, double in_max, double out_min, double out_max, bool trunc = false);

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

	// init Serial Comm
	initSerialComm(230400);


#ifdef DEBUG
	UARTprintf("Setting up PWM ... \n");
#endif
	configurePWM();
	configureGPIO();
#ifdef DEBUG
	UARTprintf("done\n");
#endif

#ifdef DEBUG
	UARTprintf("Setting up Servo ... \n");
#endif
	servo_init();
	servo_setPosition(90);
#ifdef DEBUG
	UARTprintf("done\n");
#endif

#ifdef DEBUG
	UARTprintf("Starting QEI...");
#endif
	encoder_init();
#ifdef DEBUG
	UARTprintf("done\n");
#endif

#ifdef USE_I2C
#ifdef DEBUG
	UARTprintf("Setting up I2C\n");
#endif
	//I2C
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	MAP_GPIOPinTypeI2C(GPIO_PORTB_AHB_BASE,GPIO_PIN_2 | GPIO_PIN_3);
	MAP_I2CMasterInitExpClk(I2C0_MASTER_BASE,SysCtlClockGet(),true);  //false = 100khz , true = 400khz
	I2CMasterTimeoutSet(I2C0_MASTER_BASE, 1000);
#ifdef DEBUG
	UARTprintf("done\n");
#endif
#endif

#ifdef USE_I2C
#ifdef USE_INA226
#ifdef DEBUG
	UARTprintf("Setting up INA226\n");
#endif
	initINA226();
#ifdef DEBUG
	UARTprintf("done\n");
#endif
#endif
#endif

#ifdef DEBUG
	UARTprintf("Configuring NRF24L01...");
#endif

	RC_remote ferrari;
	ferrari.buttons = 0;
	ferrari.linear = 0;
	ferrari.steer = 0;
	uint32_t last_millis = millis();

	RF24 radio = RF24();

	// Radio pipe addresses for the 2 nodes to communicate.
	const uint64_t pipes[3] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL, 0xF0F0F0F0C3LL};

	// Setup and configure rf radio
	radio.begin();

	// optionally, increase the delay between retries & # of retries
	radio.setRetries(15,15);

	// optionally, reduce the payload size.  seems to
	// improve reliability
	radio.setPayloadSize(sizeof(RC_remote));

	radio.setDataRate(RF24_250KBPS);

	// Open pipes to other nodes for communication
	radio.openWritingPipe(pipes[1]);
	radio.openReadingPipe(1,pipes[0]);

	// Start listening
	radio.startListening();

	// Dump the configuration of the rf unit for debugging
	radio.printDetails();

	radio.stopListening();
	radio.startListening();

#ifdef DEBUG
	UARTprintf("done\n");
#endif

	while (1)
	{
		// if there is data ready
		if ( radio.available() )
		{
			bool done = false;
			while (!done)
			{

				// Fetch the payload, and see if this was the last one.
				done = radio.read( &ferrari, sizeof(RC_remote));

				if(done)
				{
					if((ferrari.buttons & L1_BUTTON) == L1_BUTTON)
					{

					}
					else
					{

					}

					if((ferrari.buttons & R1_BUTTON) == R1_BUTTON)
					{
						servo_setPosition(map_value(ferrari.steer, -127, 127, SERVO_MIN, SERVO_MAX));
					}
					else
					{
						servo_setPosition(map_value(ferrari.steer, -127, 127, SERVO_MIN_PARTIAL, SERVO_MAX_PARTIAL));
					}


					last_millis = millis();

					if((ferrari.buttons & ASK_BIT) == ASK_BIT)
					{
						uint8_t temp;
						temp = GPIOPinRead(INA226_ALERT_PORT, INA226_ALERT_PIN);

#ifdef DEBUG
						UARTprintf("portE1 = %x\n", temp);
#endif
						temp = (temp & INA226_ALERT_PIN) == INA226_ALERT_PIN ? 0 : 1;

#ifdef DEBUG
						UARTprintf("sent = %d\n", ferrari.buttons);
#endif
						radio.stopListening();
						radio.write(&temp, sizeof(uint8_t));
						radio.startListening();
					}
					//					//SysCtlDelay(50*ulClockMS);
				}
			}
		}
	}
}


void SysTickHandler()
{
	milliSec++;

	communication_update_function();
}

uint32_t millis()
{
	return milliSec;
}

double map_value(double x, double in_min, double in_max, double out_min, double out_max, bool trunc)
{
	if(trunc)
	{
		double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
		if(temp > out_max) temp = out_max;
		if(temp < out_min) temp = out_min;
		return temp;
	}
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
