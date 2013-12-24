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
pid velocity_pid = pid();
INA226 power_meter;

unsigned long last_dongle_millis = 0, last_uart_millis= 0, last_dongle_millis_pid = 0;


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

	int16_t le_sum = 0, re_sum = 0 ;

#ifdef DEBUG
	UARTprintf("Setting up Servo ... \n");
#endif
	servo_init();
	servo_setPosition(90);
#ifdef DEBUG
	UARTprintf("done\n");
#endif

#ifdef DEBUG
	UARTprintf("Setting up PWM ... \n");
#endif
	configurePWM();
	configureGPIO();
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

#ifdef DEBUG
	UARTprintf("Setting up PID\n");
#endif
	velocity_pid.setGains(5.0,2.5,0.0);
	velocity_pid.setSampleTime(0.050);
	velocity_pid.setMaxAccumulatedError(40);
	velocity_pid.setFilter(0.20);
	velocity_pid.setMaxOutput(50);
	velocity_pid.setMinOutput(-50);
	velocity_pid.initSensor(0);
	velocity_pid.setNewReference(0,1);

#ifdef DEBUG
	UARTprintf("done\n");
#endif

#ifdef USE_I2C
#ifdef DEBUG
	UARTprintf("Setting up I2C\n");
#endif
	//I2C
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeI2C(GPIO_PORTB_BASE,GPIO_PIN_2 | GPIO_PIN_3);
	I2CMasterInitExpClk(I2C0_MASTER_BASE,SysCtlClockGet(),false);  //false = 100khz , true = 400khz
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
	power_meter = INA226(0x45);
	power_meter.set_sample_average(4);
	power_meter.set_calibration_value(445);
	power_meter.set_bus_voltage_limit(7.0);
	power_meter.set_mask_enable_register(BUS_UNDER_LIMIT);
#ifdef DEBUG
	UARTprintf("done\n");
#endif
#endif
#endif


	while (1)
	{

		if(millis() - last_dongle_millis_pid > 50)
		{
			last_dongle_millis_pid = millis();

			int32_t le = 0, re=0, out = 0;
			encoder_read_reset(&le, &re);
			out = velocity_pid.run((le + re)/2);
			drive_pwm(out,1);
			le_sum += le;
			re_sum += re;
		}
	}
}


void SysTickHandler()
{
	milliSec++;

	//	if(millis() - ferrari288gto.last_millis > THRESHOLD_BETWEEN_MSG)
	//	{
	//		ferrari288gto.Drive = 0;
	//		ferrari288gto.Steer = SERVO_CENTER_ANGLE;
	//
	//		//drive_pwm(0,0);
	//
	//		servo_setPosition(ferrari288gto.Steer);
	//	}
}

uint32_t millis()
{
	return milliSec;
}
