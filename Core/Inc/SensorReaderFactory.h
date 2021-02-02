/*
 * SensorReaderFactory.h
 *
 *  Created on: Feb 1, 2021
 *      Author: maurizio
 */

#ifndef INC_SENSORREADERFACTORY_H_
#define INC_SENSORREADERFACTORY_H_

#include <TemperatureReader.h>
#include <HumidityReader.h>
#include <PressureReader.h>

class SensorReaderFactory {
public:
	SensorReaderFactory();
	virtual ~SensorReaderFactory();
	TemperatureReader* createTemperatureReader();
	HumidityReader* createHumidityReader();
	PressureReader* createPressureReader();
};

#endif /* INC_SENSORREADERFACTORY_H_ */
