/*
 * I2CReader.h
 *
 *  Created on: Feb 1, 2021
 *      Author: maurizio
 */

#ifndef INC_I2CREADER_H_
#define INC_I2CREADER_H_

#include <SensorReader.h>
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

class I2CReader {
public:

	static I2CReader* getInstance();
	void init(I2C_HandleTypeDef *i2c, SensorReader* sensorReader);
	float getData(I2C_HandleTypeDef *i2c, SensorReader* sensorReader);
private:
	I2CReader();
	virtual ~I2CReader();
	static I2CReader * _instance;
	osSemaphoreId i2cSemaphoreHandle;
};

#endif /* INC_I2CREADER_H_ */
