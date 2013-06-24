extern "C" {
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"
#include <driverlib/adc.h>
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include <stdint.h>

#include "rc_cmds.h"
#include "timer.h"
#include "servo.h"
#include "INA226.h"

#define SYSTICKS_PER_SECOND     1000
#define ASK_BIT			0x10
//HEARTBEAT
#define TICKS_PER_SECOND 		1000

// servo and drive
#define MAX_PWM_STEER		100
#define MAX_PWM_DRIVE		10000


void setupADC(void);
void configurePWM(void);
void configureGPIO(void);
void startConversion0(unsigned long int * values);
void updateADCValues(unsigned long int * values);
void drive_pwm(int pwm, bool brake);
uint32_t millis();


static unsigned long milliSec = 0;

void SysTickHandler()
{
	milliSec++;

	if(millis() - ferrari288gto.last_millis > THRESHOLD_BETWEEN_MSG)
	{
		ferrari288gto.Drive = 0;
		ferrari288gto.Steer = SERVO_CENTER_ANGLE;

		drive_pwm(0,0);

		servo_setPosition(ferrari288gto.Steer);
	}
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
// testes
int min1 = 1023, max1 = 0, min2 = 1023, max2 = 0;



#include "rf24/RF24.h"
#include "remote_defines.h"
static unsigned long ulClockMS=0;

bool convert_values(RC_remote &in, struct rc_cmds &out);


#define DEBUG

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

	RC_remote ferrari;
	ferrari.linear = 0;
	ferrari.steer = 0;
	ferrari.buttons = 0;

#ifdef DEBUG
	UARTprintf("Setting up Servo ... \n");
#endif
	servo_init();
	servo_setPosition(90);

#ifdef DEBUG
	UARTprintf("Setting up PWM ... \n");
#endif
	configurePWM();
	configureGPIO();

//#ifdef DEBUG
//	UARTprintf("Setting up ADC ... \n");
//#endif
//	setupADC();

#ifdef DEBUG
    UARTprintf("SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0)\n");
#endif

#ifdef USE_I2C
    //I2C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeI2C(GPIO_PORTB_BASE,GPIO_PIN_2 | GPIO_PIN_3);
    I2CMasterInitExpClk(I2C0_MASTER_BASE,SysCtlClockGet(),false);  //false = 100khz , true = 400khz
    I2CMasterTimeoutSet(I2C0_MASTER_BASE, 1000);
#ifdef DEBUG
    UARTprintf("I2C configured\n");
#endif
#endif

#ifdef USE_INA226
    INA226 power_meter = INA226(0x45);
    power_meter.set_sample_average(4);
    power_meter.set_calibration_value(445);
    power_meter.set_bus_voltage_limit(7.0);
    power_meter.set_mask_enable_register(BUS_UNDER_LIMIT);
#endif

#ifdef USE_NRF24
    RF24 radio = RF24();

	// Radio pipe addresses for the 2 nodes to communicate.
	const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

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

#ifdef DEBUG
	// Dump the configuration of the rf unit for debugging
	radio.printDetails();
#endif

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

					ferrari288gto.last_millis = millis();
#ifdef DEBUG
					UARTprintf("l = %d, a = %d\n",ferrari.linear,ferrari.steer);
#endif
					convert_values(ferrari, ferrari288gto);

#ifdef DEBUG
					UARTprintf("L = %d, A = %d\n",(int)ferrari288gto.Drive, (int)ferrari288gto.Steer);
#endif
					servo_setPosition(ferrari288gto.Steer);
					drive_pwm(ferrari288gto.Drive, 0);

					if((ferrari.buttons & ASK_BIT) == ASK_BIT)
					{
						uint8_t temp;
						temp = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1);

#ifdef DEBUG
						UARTprintf("portE1 = %x\n", temp);
#endif
						temp = (temp & GPIO_PIN_1) == GPIO_PIN_1 ? 0 : 1;

#ifdef DEBUG
						UARTprintf("sent = %d\n", temp);
#endif
						radio.stopListening();
						radio.write(&temp, sizeof(uint8_t));
						radio.startListening();
					}
					//SysCtlDelay(50*ulClockMS);
				}
			}
		}
	}
}


void setupADC(void)
{

//	//Enable ADC0
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
//	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);
//
//    //
//    // Enable the first sample sequencer to capture the value of channel 0 when
//    // the processor trigger occurs.
//    //
//	ADCSequenceDisable(ADC_BASE, 0);
//	ADCSequenceEnable(ADC_BASE, 0);
//	ADCSequenceConfigure(ADC_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
//	ADCSequenceStepConfigure(ADC_BASE, 0, BATTERY_ADC, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH1 );
//	ADCSoftwareOversampleConfigure(ADC_BASE, 0, 4);
//
//	ADCIntClear(ADC_BASE, 0);

}

void startConversion0(unsigned long int * values)
{
	//
	// Trigger the sample sequence.
	//
	ADCProcessorTrigger(ADC_BASE, 0);
	//
	// Wait until the sample sequence has completed.
	//
	while(!ADCIntStatus(ADC_BASE, 0, false))
	{
	}

	ADCIntClear(ADC_BASE, 0);


	//
	// Read the value from the ADC.
	//
	ADCSequenceDataGet(ADC_BASE, 0, values);

	updateADCValues(values);

}

void updateADCValues(unsigned long int * values)
{
	//ferrari_steer_pid.current_pos = values[STEER_ADC];
	// TODO: conversion factor
	// 662 * 3300 /1024
	//ferrari288gto_param.battery_voltage = values[BATTERY_ADC]*1;


	//max1 = values[0];
	//max2 = values[1];

}

void configurePWM(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_6);

	PWMGenConfigure(PWM_BASE,PWM_GEN_2,PWM_GEN_MODE_UP_DOWN|PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure(PWM_BASE,PWM_GEN_3,PWM_GEN_MODE_UP_DOWN|PWM_GEN_MODE_NO_SYNC);

	PWMGenPeriodSet(PWM_BASE, PWM_GEN_2, MAX_PWM_DRIVE);		// Drive PWM
	PWMGenPeriodSet(PWM_BASE, PWM_GEN_3, MAX_PWM_DRIVE);		// Drive PWM

	PWMOutputState(PWM_BASE, (PWM_OUT_4_BIT | PWM_OUT_5_BIT|PWM_OUT_7_BIT | PWM_OUT_6_BIT), true);
	PWMGenEnable(PWM_BASE, PWM_GEN_2);
	PWMGenEnable(PWM_BASE, PWM_GEN_3);

	PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, 0);
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, 0);
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_6, 0);
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, 0);

}

void configureGPIO(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	MAP_GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// PE0 = Alert ina226
	// PE1 = IRQ MPU6050

	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);

}

void drive_pwm(int pwm, bool brake)
{
	if(!brake)
	{
		//write pwm vales
		if(pwm < 0 && pwm > -128)
		{
			PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, 0);
			PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, (PWMGenPeriodGet(PWM_BASE, PWM_GEN_2)-5) * -pwm / 128);
		}
		else if(pwm >= 0 && pwm < 128)
		{
			PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, 0);
			PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, (PWMGenPeriodGet(PWM_BASE, PWM_GEN_2)-5) * pwm / 128);
		}
		else if(pwm >= 128)
		{
			PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, 0);
			PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAX_PWM_DRIVE - 5);
		}
		else if(pwm <= -128)
		{
			PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAX_PWM_DRIVE - 5);
			PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, 0);
		}
	}
	else
	{
		//TODO : slow decay
	}
}

bool convert_values(RC_remote &in, struct rc_cmds &out)
{
	int d,s;

	d = in.linear;
	s = in.steer;

	int max_vel_fwd = 0, max_vel_rev = 0;
	if((in.buttons & L1_BUTTON) == L1_BUTTON)
	{
		max_vel_fwd = DRV_FRONT_N2O;
		max_vel_rev = DRV_REAR_N2O;
	}
	else
	{
		max_vel_fwd = DRV_FRONT;
		max_vel_rev = DRV_REAR;
	}

	if(d == 0){
		out.Drive = DRV_ZERO;
	}
	else if(d <= -127)
	{
		out.Drive = max_vel_rev;
	}
	else if(d >= 127)
	{
		out.Drive = max_vel_fwd;
	}
	else if(d < 0 && d > -127)
	{
		out.Drive = (-d * (max_vel_rev - DRV_ZERO))/(127) + DRV_ZERO;
	}
	else if(d > 0 && d < 127)
	{
		out.Drive = ((d * (max_vel_fwd - DRV_ZERO))/127) + DRV_ZERO;
	}

	if(s == 0)
	{
		out.Steer = SERVO_CENTER_ANGLE;
	}
	else if(s <= -127)
	{
		out.Steer = SERVO_LEFT_ANGLE;
	}
	else if(s >= 127)
	{
		out.Steer = SERVO_RIGHT_ANGLE;
	}
	else if(s < 0 && s > -127)
	{
		out.Steer = (-s * (SERVO_LEFT_ANGLE - SERVO_CENTER_ANGLE))/(127) + SERVO_CENTER_ANGLE;
	}
	else if(s > 0 && s < 127)
	{
		out.Steer = ((s * (SERVO_RIGHT_ANGLE - SERVO_CENTER_ANGLE))/127) + SERVO_CENTER_ANGLE;
	}

	return true;
}
//void drive_pwm(void)
//{
//	//write pwm vales
//	if(ferrari288gto.Drive < 0 && ferrari288gto.Drive > -100)
//	{
//		PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, 0);
//		PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, (PWMGenPeriodGet(PWM_BASE, PWM_GEN_2)-5) * -ferrari288gto.Drive / 100);
//	}
//	else if(ferrari288gto.Drive >= 0 && ferrari288gto.Drive < 100)
//	{
//		PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, 0);
//		PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, (PWMGenPeriodGet(PWM_BASE, PWM_GEN_2)-5) * ferrari288gto.Drive / 100);
//	}
//	else if(ferrari288gto.Drive >= 100)
//	{
//		PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, 0);
//		PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAX_PWM_DRIVE - 5);
//	}
//	else if(ferrari288gto.Drive <= -100)
//	{
//		PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAX_PWM_DRIVE - 5);
//		PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, 0);
//	}
//}

