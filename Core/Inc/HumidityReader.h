/*
 * HumidityReader.h
 *
 *  Created on: Feb 1, 2021
 *      Author: maurizio
 */

#ifndef INC_HUMIDITYREADER_H_
#define INC_HUMIDITYREADER_H_

#include <SensorReader.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"


class HumidityReader: public SensorReader {
public:
	HumidityReader();
	virtual ~HumidityReader();
	float read(I2C_HandleTypeDef *i2c);
	void init(I2C_HandleTypeDef *i2c);
private:
	float calcH(uint16_t rh);
	uint8_t crcChecksum(uint8_t data[], uint8_t no_of_bytes, uint8_t checksum);
	const uint16_t POLYNOMIAL = 0x131;  // P(x)=x^8+x^5+x^4+1 = 100110001
	static constexpr uint8_t HUM_ADD = 0x80;	// I2C device address
	static constexpr uint8_t TRIGGER_RH_MEASUREMENT_HM = 0XE5;  // command trig. hum. meas. hold master
	static constexpr uint8_t TRIGGER_RH_MEASUREMENT_NHM = 0XF5; // command trig. hum. meas. no hold master
	static constexpr uint8_t USER_REGISTER_W = 0XE6;		    // command writing user register
	static constexpr uint8_t USER_REGISTER_R = 0XE7;            // command reading user register
	static constexpr uint8_t SOFT_RESET = 0XFE;                 // command soft reset

};

#endif /* INC_HUMIDITYREADER_H_ */
