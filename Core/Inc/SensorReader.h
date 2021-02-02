/*
 * SensorReader.h
 *
 *  Created on: Feb 1, 2021
 *      Author: maurizio
 */

#ifndef INC_SENSORREADER_H_
#define INC_SENSORREADER_H_

#include "stm32f1xx_hal.h"

class SensorReader {
public:
	SensorReader();
	virtual ~SensorReader();
	virtual void init(I2C_HandleTypeDef *i2c) = 0;
	virtual float read(I2C_HandleTypeDef *i2c) = 0;
};

#endif /* INC_SENSORREADER_H_ */
