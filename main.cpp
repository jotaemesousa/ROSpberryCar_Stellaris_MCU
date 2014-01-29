extern "C" {
#include <math.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_i2c.h>
#include <inc/hw_ints.h>
#include <driverlib/adc.h>
#include <driverlib/interrupt.h>
#include <driverlib/i2c.h>
//#include <driverlib/pwm.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/systick.h>
#include <driverlib/gpio.h>
#include <driverlib/uart.h>
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include <stdint.h>
#include <driverlib/ssi.h>

#include "Utilities/rc_cmds.h"
#include "timer.h"
#include "Utilities/servo.h"
#include "Utilities/INA226.h"
#include "Utilities/Encoder.h"
#include "soft_pwm.h"

// use sensors
//#define USE_I2C
//#define USE_INA226
#define USE_NRF24

#define SYSTICKS_PER_SECOND     1000

// HEARTBEAT
#define TICKS_PER_SECOND 		1000

// servo and drive
#define MAX_PWM_STEER			100
#define MAX_PWM_DRIVE			10000

// debug
#define DEBUG
#define DEBUG_CMD

void configurePWM(void);
void configureGPIO(void);
void SysTickHandler();
uint32_t millis();

static unsigned long milliSec = 0;

void InitConsole(void)
{
	//
	// Enable GPIO port A which is used for UART0 pins.
	// TODO: change this to whichever GPIO port you are using.
	//
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	MAP_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOA);
	//
	// Select the alternate (UART) function for these pins.
	// TODO: change this to select the port/pin you are using.
	//
	MAP_GPIOPinTypeUART(GPIO_PORTA_AHB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// Initialize the UART for console I/O.
	//
	UARTStdioInitExpClk(0,230400);
}



}
// testes
int min1 = 1023, max1 = 0, min2 = 1023, max2 = 0;


#include "comm.h"
#include "rf24/RF24.h"
#include "remote_defines.h"
#include "Utilities/pid.h"

static unsigned long ulClockMS=0;
pid velocity_pid = pid();

unsigned long last_dongle_millis = 0, last_car_param_millis = 0, last_dongle_millis_pid = 0;

bool convert_values(RC_remote &in, RC_Param &car_param, struct rc_cmds &out);
void updateLights(RC_remote &in);
void drive_pwm(int pwm, bool brake);



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
	UARTEchoSet(false);

	RC_remote ferrari;
	ferrari.linear = 0;
	ferrari.steer = 0;
	ferrari.buttons = 0;

	int16_t le_sum = 0, re_sum = 0 ;

	RC_Param car_param;

#ifdef DEBUG
	UARTprintf("Setting up PWM ... \n");
#endif
	configurePWM();
#ifdef DEBUG
	UARTprintf("Done \n");
#endif



//#ifdef DEBUG
//	UARTprintf("Setting up GPIO ... \n");
//#endif
//	configureGPIO();
//#ifdef DEBUG
//	UARTprintf("Done \n");
//#endif
#ifdef DEBUG
	UARTprintf("Setting up Servo ...\n");
#endif
	servo_init();
	servo_setPosition(90);
#ifdef DEBUG
	UARTprintf("Done \n");
#endif

#ifdef DEBUG
	UARTprintf("Starting QEI...\n");
#endif
	encoder_init();
#ifdef DEBUG
	UARTprintf("Done \n");
#endif

#ifdef USE_I2C
#ifdef DEBUG
	UARTprintf("Starting i2c ...\n");
#endif
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

#ifdef USE_I2C
#ifdef USE_INA226
#ifdef DEBUG
	UARTprintf("setting up ina226 ...\n");
#endif
	INA226 power_meter = INA226(0x45);
	power_meter.set_sample_average(4);
	power_meter.set_calibration_value(445);
	power_meter.set_bus_voltage_limit(7.0);
	power_meter.set_mask_enable_register(BUS_UNDER_LIMIT);
#ifdef DEBUG
	UARTprintf("Done \n");
#endif
#endif
#endif




#ifdef USE_NRF24
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

#ifdef DEBUG
	// Dump the configuration of the rf unit for debugging
	radio.printDetails();
#endif

#endif

	while (1)
	{
#ifdef USE_NRF24
		//if there is data ready
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
#ifdef DEBUG_CMD
					UARTprintf("l = %d, a = %d\n",ferrari.linear,ferrari.steer);
#endif
					convert_values(ferrari, car_param, ferrari288gto);

#ifdef DEBUG_CMD
					UARTprintf("L = %d, A = %d\n",(int)ferrari288gto.Drive, (int)ferrari288gto.Steer);
#endif
					servo_setPosition(ferrari288gto.Steer);
//					ferrari288gto.last_millis = millis();
					//							drive_pwm(ferrari288gto.Drive, 0);

					velocity_pid.setNewReference((float)ferrari288gto.Drive/4.0,0);

					if((ferrari.buttons & ASK_BIT) == ASK_BIT)
					{
//						uint8_t temp;
//						temp = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1);
//
//#ifdef DEBUG
//						UARTprintf("portE1 = %x\n", temp);
//#endif
//						temp = (temp & GPIO_PIN_1) == GPIO_PIN_1 ? 0 : 1;
//
//#ifdef DEBUG
//						UARTprintf("sent = %d\n", temp);
//#endif
//						radio.stopListening();
//						radio.write(&temp, sizeof(uint8_t));
//						radio.startListening();
					}


					updateLights(ferrari);
					//SysCtlDelay(50*ulClockMS);
				}
			}
		}

//		if(millis() - last_car_param_millis > CAR_PARAM_MILLIS)
//		{
//			last_car_param_millis = millis();
//
//			int32_t l_vel, r_vel;
//			encoder_get_velocity(&l_vel, &r_vel, millis());
//			car_param.velocity = (l_vel + r_vel)/2;
//			car_param.batery_level = power_meter.get_bus_voltage();
//			car_param.x = 0;
//			car_param.y = 0;
//		}

		if(millis() - last_dongle_millis > DONGLE_MILLIS)
		{
			last_dongle_millis = millis();

			radio.stopListening();
			radio.openWritingPipe(pipes[2]);

			radio.write(&car_param, sizeof(RC_Param));
			radio.openWritingPipe(pipes[1]);
			radio.startListening();

			UARTprintf(":Enc %03d %03d;\n", le_sum, re_sum);
			le_sum = 0;
			re_sum = 0;
		}
#endif

		serial_receive();

		if(millis() - last_dongle_millis_pid > 50)
		{
			last_dongle_millis_pid = millis();

			int32_t le = 0, re=0, out = 0;
			encoder_read_reset(&le, &re);
			out = velocity_pid.run(le);
			drive_pwm(out,1);
			le_sum += le;
			re_sum += re;
		}
	}
}

void configurePWM(void)
{
	initSoftPWM(500,40);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_AHB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	setPWMGenFreq(1,50);

	setPWMGenFreq(3,500);
	SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_AHB_BASE, GPIO_PIN_6 | GPIO_PIN_7);


	enablePWM();
}

void configureGPIO(void)
{
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
//	MAP_GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);
//
//	// PE0 = Alert ina226
//	// PE1 = IRQ MPU6050
//
//	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);

}

void drive_pwm(int pwm, bool brake)
{
//	if(!brake)
//	{
//		//write pwm vales
//		if (pwm==0)
//		{
//			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, false);
//		}
//		else if(pwm > 0 && pwm < 127)
//		{
//			MAP_PWMOutputState(PWM_BASE, PWM_OUT_5_BIT, false);
//			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, true);
//			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2) * pwm / 127);
//
//		}
//		else if (pwm >=127)
//		{
//			MAP_PWMOutputState(PWM_BASE, PWM_OUT_5_BIT, false);
//			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, true);
//			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
//		}
//		else if ( pwm > -127)
//		{
//			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, false);
//			MAP_PWMOutputState(PWM_BASE, PWM_OUT_5_BIT, true);
//			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2) * -pwm / 127);
//		}
//		else
//		{
//			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, false);
//			MAP_PWMOutputState(PWM_BASE, PWM_OUT_5_BIT, true);
//			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
//		}
//	}
//	else
//	{
//		//write pwm vales
//		if (pwm==0)
//		{
//			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, true);
//			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
//			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
//		}
//		else if(pwm > 0 && pwm < 127)
//		{
//			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, true);
//			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2) * (127 - pwm) / 127);
//			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
//
//		}
//		else if (pwm >= 127)
//		{
//			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, true);
//			MAP_PWMOutputState(PWM_BASE, PWM_OUT_5_BIT, false);
//			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
//		}
//		else if ( pwm > -127)
//		{
//			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, true);
//			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2) * (127 + pwm) / 127);
//			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
//		}
//		else
//		{
//			MAP_PWMOutputState(PWM_BASE, PWM_OUT_5_BIT, true);
//			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, false);
//			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
//		}
//	}
}

void updateLights(RC_remote &in)
{
	static uint8_t last_buttons = 0, front_state = 0, tail_state = 0;

	if((in.buttons & R2_BUTTON) == R2_BUTTON)
	{
		if((last_buttons & R2_BUTTON) != R2_BUTTON)
		{
			front_state ++;
			tail_state ++;
			if(front_state > 3) front_state = 3;
			if(tail_state > 3) tail_state = 3;
#ifdef DEBUG
			UARTprintf("R2\n");
#endif
		}
	}
	else if((in.buttons & L2_BUTTON) == L2_BUTTON)
	{
		if((last_buttons & L2_BUTTON) != L2_BUTTON)
		{
			front_state --;
			tail_state--;
			if(front_state < 0) front_state = 0;
			if(tail_state < 0) tail_state = 0;
#ifdef DEBUG
			UARTprintf("L2\n");
#endif
		}
	}

//	switch (front_state)
//	{
//	case 0:
//		PWMPulseWidthSet(PWM_BASE, PWM_OUT_6, 0);
//		break;
//
//	case 1:
//		PWMPulseWidthSet(PWM_BASE, PWM_OUT_6, PWMGenPeriodGet(PWM_BASE, PWM_GEN_3) / 6);
//		break;
//
//	case 2:
//		PWMPulseWidthSet(PWM_BASE, PWM_OUT_6, PWMGenPeriodGet(PWM_BASE, PWM_GEN_3) / 2);
//		break;
//
//	case 3:
//		PWMPulseWidthSet(PWM_BASE, PWM_OUT_6, PWMGenPeriodGet(PWM_BASE, PWM_GEN_3)-5);
//		break;
//	}
//
//	switch(tail_state)
//	{
//	case 0:
//		if((in.buttons & R1_BUTTON) == R1_BUTTON)
//		{
//			PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, PWMGenPeriodGet(PWM_BASE, PWM_GEN_3) - 5);
//		}
//		else
//		{
//			PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, 0);
//		}
//		break;
//	case 1:
//	case 2:
//	case 3:
//
//		if((in.buttons & R1_BUTTON) == R1_BUTTON)
//		{
//			PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, PWMGenPeriodGet(PWM_BASE, PWM_GEN_3) - 5);
//		}
//		else
//		{
//			PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, PWMGenPeriodGet(PWM_BASE, PWM_GEN_3) / 3);
//		}
//		break;
//
//	}

	last_buttons = in.buttons;

}

bool convert_values(RC_remote &in, RC_Param &car_param, struct rc_cmds &out)
{
	float steer_factor = 1;
	static bool steer_mode = 0;

	if(car_param.velocity < VEL_STEER_MIN && steer_mode == 1)
	{
		steer_factor = 1;
		steer_mode = 0;
	}
	else if(car_param.velocity > VEL_STEER_MAX && steer_mode == 0)
	{
		if(in.steer > 0)
		{
			steer_factor = (pow(in.steer / 127.0, 2) + in.steer/127.0) / 2.0;
		}
		else
		{
			steer_factor = (pow(in.steer / 127.0, 2) - in.steer/127.0) / 2.0;
		}
		steer_mode = 0;
	}

	int d,s;

	d = in.linear;
	s = in.steer * steer_factor;

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
