/*
 * I2CReader.cpp
 *
 *  Created on: Feb 1, 2021
 *      Author: maurizio
 */

#include <I2CReader.h>

I2CReader* I2CReader::_instance = 0;

I2CReader::I2CReader() {
	osSemaphoreDef(i2cSemaphore);
	i2cSemaphoreHandle = osSemaphoreCreate(osSemaphore(i2cSemaphore), 1);
}

I2CReader::~I2CReader() {

}

I2CReader* I2CReader::getInstance() {
	if (_instance == 0){
		_instance = new I2CReader();
	}

	return _instance;
}

void I2CReader::init(I2C_HandleTypeDef *i2c) {
	hi2c1 = i2c;
}


float I2CReader::getData(SensorReader* sensorReader) {
	osSemaphoreWait(sensorReader, osWaitForever);
	float value = sensorReader->read(hi2c1);
	osSemaphoreRelease(sensorReader);
	return value;
}

