/*
 * INA226.h
 *
 *  Created on: May 13, 2013
 *      Author: joao
 */

#ifndef INA226_H_
#define INA226_H_

enum registers
{
	configuration_register = 0,
	shunt_voltage_register,
	bus_voltage_register,
	power_register,
	current_register,
	calibration_register,
	mask_enable_register,
	alert_limit_register

};
typedef enum registers i2c_registers;

enum operating_mode_settings
{
	power_down = 0,
	shunt_voltage_triggered,
	bus_voltage_triggered,
	shunt_bus_voltage_triggered,
	shunt_voltage_continuous = 5,
	bus_voltage_continuous,
	shunt_bus_voltage_continuous

};
typedef enum operating_mode_settings INA226_settings;

class INA226
{

	char addr_;
	uint16_t conf_reg_;

	uint16_t read_register(registers reg);
	void write_register(registers reg, uint16_t value);

public:
	INA226(void);
	INA226(char addr);



	void set_i2c_addr(char addr){ addr_ = addr;}
	void set_sample_average(int avg);
	void set_vbus_conv_timer(uint16_t ct);
	void set_vshunt_conv_timer(uint16_t ct);
	void set_operating_mode(operating_mode_settings mode);

	virtual ~INA226();
};

#endif /* INA226_H_ */
