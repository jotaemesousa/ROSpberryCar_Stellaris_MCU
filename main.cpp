#include "common_includes.h"
#include "defined_macros.h"



#include "Utilities/servo.h"

extern "C"
{

void configurePWM(void);
void configureGPIO(void);
void SysTickHandler();
uint32_t millis();

static unsigned long milliSec = 0;

}
// testes
int min1 = 1023, max1 = 0, min2 = 1023, max2 = 0;


#include "comm.h"
#include "remote_defines.h"
#include "Utilities/pid.h"

static unsigned long ulClockMS=0;
pid velocity_pid = pid();
INA226 power_meter;

unsigned long last_dongle_millis = 0, last_uart_millis= 0, last_dongle_millis_pid = 0;

bool convert_values(RC_remote &in, RC_Param &car_param, RC_Cmds &out);
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
	UARTprintf("Setting up PWM ... \n");
#endif
	configurePWM();
	configureGPIO();

	UARTprintf("Starting QEI...");
	encoder_init();
	UARTprintf("done\n");


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
	power_meter = INA226(0x45);
	power_meter.set_sample_average(4);
	power_meter.set_calibration_value(445);
	power_meter.set_bus_voltage_limit(7.0);
	power_meter.set_mask_enable_register(BUS_UNDER_LIMIT);
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

		if(millis() - last_uart_millis > 1000)
		{
			last_uart_millis = millis();

			UARTprintf(":Enc %03d %03d;\n", le_sum, re_sum);
			le_sum = 0;
			re_sum = 0;
		}
	}
}

void configurePWM(void)
{
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	MAP_GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);

	MAP_PWMGenConfigure(PWM_BASE,PWM_GEN_2,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
	MAP_PWMGenConfigure(PWM_BASE,PWM_GEN_3,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);

	MAP_PWMGenPeriodSet(PWM_BASE, PWM_GEN_2, MAX_PWM_DRIVE);		// Drive PWM
	MAP_PWMGenPeriodSet(PWM_BASE, PWM_GEN_3, MAX_PWM_DRIVE);		// Drive PWM

	MAP_PWMGenEnable(PWM_BASE, PWM_GEN_2);
	MAP_PWMGenEnable(PWM_BASE, PWM_GEN_3);

	MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, false);

	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);

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
		if (pwm==0)
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, false);
		}
		else if(pwm > 0 && pwm < 127)
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_5_BIT, false);
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, true);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2) * pwm / 127);

		}
		else if (pwm >=127)
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_5_BIT, false);
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, true);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
		}
		else if ( pwm > -127)
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, false);
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_5_BIT, true);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2) * -pwm / 127);
		}
		else
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, false);
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_5_BIT, true);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
		}
	}
	else
	{
		//write pwm vales
		if (pwm==0)
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, true);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
		}
		else if(pwm > 0 && pwm < 127)
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, true);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2) * (127 - pwm) / 127);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));

		}
		else if (pwm >= 127)
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, true);
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_5_BIT, false);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
		}
		else if ( pwm > -127)
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, true);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2) * (127 + pwm) / 127);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
		}
		else
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_5_BIT, true);
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, false);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
		}
	}
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

	switch (front_state)
	{
	case 0:
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_6, 0);
		break;

	case 1:
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_6, PWMGenPeriodGet(PWM_BASE, PWM_GEN_3) / 6);
		break;

	case 2:
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_6, PWMGenPeriodGet(PWM_BASE, PWM_GEN_3) / 2);
		break;

	case 3:
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_6, PWMGenPeriodGet(PWM_BASE, PWM_GEN_3)-5);
		break;
	}

	switch(tail_state)
	{
	case 0:
		if((in.buttons & R1_BUTTON) == R1_BUTTON)
		{
			PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, PWMGenPeriodGet(PWM_BASE, PWM_GEN_3) - 5);
		}
		else
		{
			PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, 0);
		}
		break;
	case 1:
	case 2:
	case 3:

		if((in.buttons & R1_BUTTON) == R1_BUTTON)
		{
			PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, PWMGenPeriodGet(PWM_BASE, PWM_GEN_3) - 5);
		}
		else
		{
			PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, PWMGenPeriodGet(PWM_BASE, PWM_GEN_3) / 3);
		}
		break;

	}

	last_buttons = in.buttons;

}

bool convert_values(RC_remote &in, RC_Param &car_param, RC_Cmds &out)
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
