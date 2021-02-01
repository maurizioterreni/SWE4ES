/*
 * TemperatureReader.h
 *
 *  Created on: Feb 1, 2021
 *      Author: maurizio
 */

#ifndef SRC_TEMPERATUREREADER_H_
#define SRC_TEMPERATUREREADER_H_

#include <SensorReader.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#define I2C_ADD 0x40	// I2C device address

#define TRIGGER_T_MEASUREMENT_HM 0XE3   // command trig. temp meas. hold master
#define TRIGGER_T_MEASUREMENT_NHM 0XF3  // command trig. temp meas. no hold master
#define USER_REGISTER_W 0XE6		    // command writing user register
#define USER_REGISTER_R 0XE7            // command reading user register
#define SOFT_RESET 0XFE                 // command soft reset

const uint16_t POLYNOMIAL = 0x131;  // P(x)=x^8+x^5+x^4+1 = 100110001


class TemperatureReader: SensorReader {
public:
	TemperatureReader();
	virtual ~TemperatureReader();
	float read(I2C_HandleTypeDef *i2c);
	void init(I2C_HandleTypeDef *i2c);
private:
	float calcT(uint16_t t);
	uint8_t crcChecksum(uint8_t data[], uint8_t no_of_bytes, uint8_t checksum);

};

#endif /* SRC_TEMPERATUREREADER_H_ */
