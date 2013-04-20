#include <inc/lm3s2776.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <driverlib/debug.h>
#include "driverlib/pwm.h"
#include <driverlib/sysctl.h>
#include <driverlib/systick.h>
#include <driverlib/gpio.h>
#include <driverlib/adc.h>
#include <stdlib.h>
#include "utils/ustdlib.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "servo.h"
#include <math.h>

#include "rc_cmds.h"
#include "adxl345.h"
#include "timer.h"


static unsigned long ulClockMS=0;

//HEARTBEAT
#define TICKS_PER_SECOND 		1000

// adc
#define BATTERY_ADC			1

// servo and drive
#define MAX_PWM_STEER		100
#define MAX_PWM_DRIVE		10000


void setupADC(void);
void configurePWM(void);
void startConversion0(unsigned int * values);
void updateADCValues(unsigned int * values);
void drive_pwm(void);

// testes
int min1 = 1023, max1 = 0, min2 = 1023, max2 = 0;

int main(void)
{
	//SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_12MHZ);
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |SYSCTL_XTAL_12MHZ);

	ulClockMS = SysCtlClockGet() / (3 * 1000);

#ifdef DEBUG
	UARTprintf("Setting up UART ... \n");
#endif
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Initialize the UART as a console for text I/O.
	UARTStdioInitExpClk(0,115200);
#ifdef DEBUG
	UARTprintf("Setting up Servo ... \n");
#endif
	servo_init();
	servo_setPosition(90);

#ifdef DEBUG
	UARTprintf("Setting up PWM ... \n");
#endif
	configurePWM();

#ifdef DEBUG
	UARTprintf("Setting up ADC ... \n");
#endif
	setupADC();

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);


	//setup HeartBeat (using SysTickTimer)
    SysTickPeriodSet(SysCtlClockGet()/TICKS_PER_SECOND);
    SysTickEnable();
    SysTickIntEnable();

#ifdef DEBUG
    UARTprintf("SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0)\n");
#endif

    //I2C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

#ifdef DEBUG
    UARTprintf("GPIOPinTypeI2C(GPIO_PORTB_BASE,GPIO_PIN_2 | GPIO_PIN_3);\n");
#endif

    GPIOPinTypeI2C(GPIO_PORTB_BASE,GPIO_PIN_2 | GPIO_PIN_3);

#ifdef DEBUG
    UARTprintf("I2CMasterInitExpClk(I2C0_MASTER_BASE,SysCtlClockGet(),false);\n");
#endif

    I2CMasterInitExpClk(I2C0_MASTER_BASE,SysCtlClockGet(),false);  //false = 100khz , true = 400khz
    I2CMasterTimeoutSet(I2C0_MASTER_BASE, 1000);

#ifdef DEBUG
    UARTprintf("Init ADLX345_acc\n");
#endif

#ifdef ADLX345_INSTALLED
    init_adlx345();
#endif

#ifdef DEBUG
    UARTprintf("Init ADLX345_acc done\n");

    UARTprintf("dev id\n");

    UARTprintf("%X\n",get_dev_id_adxl345());
#endif

#ifdef ADLX345_INSTALLED
    struct Accelerometer adxl345;
#endif

#ifdef DEBUG
    UARTprintf("enter loop\n");
#endif

#ifdef ADLX345_INSTALLED
    ADXL345_GetRange();
#endif

#ifdef DEBUG
	UARTprintf("starting\n");
#endif


	char input;
	unsigned int adc_pointer[2];
	int c = 0;

	startConversion0(adc_pointer);

	while(1)
	{
		while(UARTCharsAvail(UART0_BASE))		// read serial data from Raspberry Pi
		{
			input = UARTCharGet(UART0_BASE);	// get a char
			add_rx_char(input);					// store the got char
			UARTCharPut(UART0_BASE, input);		// echo char
		}

		startConversion0(adc_pointer);
		if(ferrari288gto.new_msg == 1)
		{
			parseMessage();

			drive_pwm();

			servo_setPosition(ferrari288gto.Steer);

			ferrari288gto.new_msg = 0;
		}

#ifdef DEBUG_PID
		c++;
		if(c > 200)
		{
			//UARTprintf("%d %d drive=%d\n", timer0.s, timer0.ms, ferrari288gto.Drive);

			//UARTprintf("ch1=m%d M%d ch2=m%d M%d\n", min1, max1, min2, max2);
			c=0;
		}
#endif

#ifdef ADLX345_INSTALLED
		get_acc(&adxl345);
#endif
		SysCtlDelay(1*ulClockMS);

#ifdef DEBUG
		UARTprintf("x=%d\ty=%d\tz=%d\n",adxl345.acc_x, adxl345.acc_y, adxl345.acc_z);
		SysCtlDelay(50*ulClockMS);
#endif

	}
}

void setupADC(void)
{

	//Enable ADC0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);

    //
    // Enable the first sample sequencer to capture the value of channel 0 when
    // the processor trigger occurs.
    //
	ADCSequenceDisable(ADC_BASE, 0);
	ADCSequenceEnable(ADC_BASE, 0);
	ADCSequenceConfigure(ADC_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC_BASE, 0, BATTERY_ADC, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH1 );
	ADCSoftwareOversampleConfigure(ADC_BASE, 0, 4);

	ADCIntClear(ADC_BASE, 0);

}

void startConversion0(unsigned int * values)
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
	ADCSequenceDataGet(ADC_BASE, 0, (unsigned int)values);

	updateADCValues(values);

}

void updateADCValues(unsigned int * values)
{
	//ferrari_steer_pid.current_pos = values[STEER_ADC];
	// TODO: conversion factor
	// 662 * 3300 /1024
	//ferrari288gto_param.battery_voltage = values[BATTERY_ADC]*1;


	max1 = values[0];
	max2 = values[1];

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

void drive_pwm(void)
{
	//write pwm vales
	if(ferrari288gto.Drive < 0 && ferrari288gto.Drive > -100)
	{
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, 0);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, (PWMGenPeriodGet(PWM_BASE, PWM_GEN_2)-5) * -ferrari288gto.Drive / 100);
	}
	else if(ferrari288gto.Drive >= 0 && ferrari288gto.Drive < 100)
	{
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, 0);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, (PWMGenPeriodGet(PWM_BASE, PWM_GEN_2)-5) * ferrari288gto.Drive / 100);
	}
	else if(ferrari288gto.Drive >= 100)
	{
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, 0);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAX_PWM_DRIVE - 5);
	}
	else if(ferrari288gto.Drive <= -100)
	{
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAX_PWM_DRIVE - 5);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, 0);
	}
}
