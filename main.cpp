#include "common_includes.h"
#include "defined_macros.h"
#include "comm.h"

#include "Utilities/servo.h"
#include "remote_defines.h"
#include "Utilities/pid.h"
#include "Utilities/gpio_pwm_lights.h"

extern "C"
{
void SysTickHandler();
uint32_t millis();

static unsigned long milliSec = 0;
}

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

	// init Serial Comm
	initSerialComm(230400);

	// init SSI0 in slave mode
	initSPIComm();

#ifdef DEBUG
	UARTprintf("Setting up PWM ... \n");
#endif
	configurePWM();
	configureGPIO();
#ifdef DEBUG
	UARTprintf("done\n");
#endif

#ifdef DEBUG
	UARTprintf("Setting up PID\n");
#endif
	initCarPID();
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

#ifdef DEBUG
	UARTprintf("SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0)\n");
#endif

#ifdef USE_I2C
#ifdef DEBUG
	UARTprintf("Setting up I2C\n");
#endif
	//I2C
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeI2C(GPIO_PORTB_BASE,GPIO_PIN_2 | GPIO_PIN_3);
	I2CMasterInitExpClk(I2C0_MASTER_BASE,SysCtlClockGet(),true);  //false = 100khz , true = 400khz
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


	while (1)
	{

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
