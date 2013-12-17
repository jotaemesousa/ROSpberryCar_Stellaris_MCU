/*
 * INA226.h
 *
 *  Created on: May 13, 2013
 *      Author: joao
 */

#ifndef INA226_H_
#define INA226_H_

#include "stdint.h"

// Registers
#define REG_CONFGURATION					0x00
#define REG_SHUNT_VOLTAGE					0x01
#define REG_BUS_VOLTAGE						0x02
#define REG_POWER							0x03
#define REG_CURRENT							0x04
#define REG_CALIBRATION						0x05
#define REG_MASK_ENABLE						0x06
#define REG_ALERT_LIMIT						0x07

// Operating Mode Settings
#define MODE_POWER_DOWN						0b00000000
#define MODE_SHUNT_VOLTAGE_TRIGGERED		0b00000001
#define MODE_BUS_VOLTAGE_TRIGGERED			0b00000010
#define MODE_SHUNT_BUS_VOLTAGE_TRIGGERED	0b00000011
#define MODE_SHUNT_VOLTAGE_CONTINUOUS		0b00000101
#define MODE_BUS_VOLTAGE_CONTINUOUS			0b00000110
#define MODE_SHUNT_BUS_VOLTAGE_CONTINUOUS	0b00000111

// Mask/ Enable
#define SHUNT_OVER_LIMIT					0x8000
#define SHUNT_UNDER_LIMIT					0x4000
#define BUS_OVER_LIMIT						0x2000
#define BUS_UNDER_LIMIT						0x1000
#define OVER_POWER_LIMIT					0x0800
#define CONVERSION_READY					0x0400
#define ALERT_FUNCTION_FLAG					0x0010
#define CONVERSION_READY_FLAG				0x0008
#define MATH_OVERFLOW_FLAG					0x0004
#define ALERT_POLARITY_BIT					0x0002
#define ALERT_LATCH_ENABLE					0x0001


class INA226
{

	char addr_;
	uint16_t conf_reg_high_,conf_reg_low_;



public:
	INA226();
	INA226(char addr);
	~INA226();
//private
	uint16_t read_register(uint8_t reg);
	void write_register(uint8_t reg, uint16_t value);


	void set_i2c_addr(char addr){ addr_ = addr;}
	void set_sample_average(int avg);
	void set_vbus_conv_timer(uint16_t ct);
	void set_vshunt_conv_timer(uint16_t ct);
	void set_operating_mode(uint8_t mode);
	void set_alert_limit_register(uint16_t value);
	void set_calibration_value(uint16_t calib);
	void set_mask_enable_register(uint16_t bits);

	int get_bus_voltage(bool return_raw_data = true);
	int get_bus_current(void);
	int get_shunt_voltage(bool return_raw_data = true);
	uint16_t get_calibration_value(void);
	uint16_t get_alert_limit_register(void);
	uint16_t get_mask_enable_register(void);

	// mask / enable functions
	void set_shunt_over_limit_bit(void);
	void set_shunt_under_limit_bit(void);
	void set_bus_over_limit_bit(void);
	void set_bus_under_limit_bit(void);
	void set_over_power_limit_bit(void);
	void set_conversion_ready_alert_bit(void);

	// alert limit register
	void set_bus_voltage_limit(float volts);
	void set_shunt_voltage_limit(float microvolts);
	void set_power_limit(float watts);
	void set_alert_polarity_bit(bool mode);
	bool read_math_overflow_flag(void);

};

int TwoComplement2ModSig_16bit(uint16_t a);
uint16_t ModSig_16bit2TwoComplement(int a);

#endif /* INA226_H_ */
