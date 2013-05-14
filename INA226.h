/*
 * INA226.h
 *
 *  Created on: May 13, 2013
 *      Author: joao
 */

#ifndef INA226_H_
#define INA226_H_

// Registers
#define CONFGURATION_REGISTER				0x00
#define SHUNT_VOLTAGE_REGISTER				0x01
#define BUS_VOLTAGE_REGISTER				0x02
#define POWER_REGISTER						0x03
#define CURRENT_REGISTER					0x04
#define CALIBRATION_REGISTER				0x05
#define MASK_ENABLE_REGISTER				0x06
#define ALERT_LIMIT_REGISTER				0x07

// Operating Mode Settings
#define POWER_DOWN_MODE						0b00000000
#define SHUNT_VOLTAGE_TRIGGERED_MODE		0b00000001
#define BUS_VOLTAGE_TRIGGERED_MODE			0b00000010
#define SHUNT_BUS_VOLTAGE_TRIGGERED_MODE	0b00000011
#define SHUNT_VOLTAGE_CONTINUOUS_MODE		0b00000101
#define BUS_VOLTAGE_CONTINUOUS_MODE			0b00000110
#define SHUNT_BUS_VOLTAGE_CONTINUOUS_MODE	0b00000111

class INA226
{

	char addr_;
	uint16_t conf_reg_;

	uint16_t read_register(uint8_t reg);
	void write_register(uint8_t reg, uint16_t value);

public:
	INA226(void);
	INA226(char addr);



	void set_i2c_addr(char addr){ addr_ = addr;}
	void set_sample_average(int avg);
	void set_vbus_conv_timer(uint16_t ct);
	void set_vshunt_conv_timer(uint16_t ct);
	void set_operating_mode(uint8_t mode);

	virtual ~INA226();
};

#endif /* INA226_H_ */
