/*
 * PressureReader.h
 *
 *  Created on: Feb 1, 2021
 *      Author: maurizio
 */

#ifndef INC_PRESSUREREADER_H_
#define INC_PRESSUREREADER_H_

#include <SensorReader.h>
#include <stdint.h>
#include "math.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#define PRES_ADD 0xEE
#define atmPress 101325 //Pa

class PressureReader: public SensorReader {
public:
	PressureReader();
	virtual ~PressureReader();
	float read(I2C_HandleTypeDef *i2c);
	void init(I2C_HandleTypeDef *i2c);
private:
	void readCalliberationData(I2C_HandleTypeDef *i2c);
	uint32_t getUPress(I2C_HandleTypeDef *i2c, int oss);
	// Defines according to the datsheet
	short AC1 = 0;
	short AC2 = 0;
	short AC3 = 0;
	unsigned short AC4 = 0;
	unsigned short AC5 = 0;
	unsigned short AC6 = 0;
	short B1 = 0;
	short B2 = 0;
	short MB = 0;
	short MC = 0;
	short MD = 0;
	/********************/
	long UT = 0;
	short oss = 0;
	long UP = 0;
	long X1 = 0;
	long X2 = 0;
	long X3 = 0;
	long B3 = 0;
	long B5 = 0;
	unsigned long B4 = 0;
	long B6 = 0;
	unsigned long B7 = 0;
	/*******************/
	long press = 0;

};

#endif /* INC_PRESSUREREADER_H_ */
