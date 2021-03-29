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

void I2CReader::init(I2C_HandleTypeDef *i2c, SensorReader* sensorReader) {
	osSemaphoreWait(i2cSemaphoreHandle, osWaitForever);
	sensorReader->init(i2c);
	osSemaphoreRelease(i2cSemaphoreHandle);
}


float I2CReader::getData(I2C_HandleTypeDef *i2c, SensorReader* sensorReader) {
	float value = sensorReader->read(i2c);
	return value;
}

void I2CReader::wait() {
	osSemaphoreWait(i2cSemaphoreHandle, osWaitForever);
}

void I2CReader::release() {
	osSemaphoreRelease(i2cSemaphoreHandle);
}

