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


class TemperatureReader: public SensorReader {
public:
	TemperatureReader();
	virtual ~TemperatureReader();
	float read(I2C_HandleTypeDef *i2c);
	void init(I2C_HandleTypeDef *i2c);
private:
	static constexpr uint8_t I2C_ADR        = 0x40;
	static constexpr uint8_t USR_REG_WRITE  = 0xE6;
	static constexpr uint8_t USR_REG_READ   = 0xE7;
	static constexpr uint8_t SOFT_RESET     = 0xFE;

	static constexpr uint8_t TEMP_HOLD      = 0xE3;
	static constexpr uint8_t TEMP_POLL      = 0xF3;
	float calcT(uint16_t t);
	uint8_t crcChecksum(uint16_t message_from_sensor, uint8_t check_value_from_sensor);

};

#endif /* SRC_TEMPERATUREREADER_H_ */
