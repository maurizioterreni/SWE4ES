/*
 * SensorReaderFactory.cpp
 *
 *  Created on: Feb 1, 2021
 *      Author: maurizio
 */

#include <SensorReaderFactory.h>

SensorReaderFactory::SensorReaderFactory() {

}

SensorReaderFactory::~SensorReaderFactory() {

}


TemperatureReader* SensorReaderFactory::createTemperatureReader() {
	return new TemperatureReader();
}


HumidityReader* SensorReaderFactory::createHumidityReader() {
	return new HumidityReader();
}


PressureReader* SensorReaderFactory::createPressureReader() {
	return  new PressureReader();
}
