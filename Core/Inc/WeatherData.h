/*
 * WeatherData.h
 *
 *  Created on: Feb 2, 2021
 *      Author: maurizio
 */

#ifndef INC_WEATHERDATA_H_
#define INC_WEATHERDATA_H_

#include "cmsis_os.h"
#include "math.h"
#include <stdint.h>
#include <stdio.h>

class WeatherData {
public:
	virtual ~WeatherData();
	static WeatherData* getInstance();
	void updateTemperature(float value);
	void updateHumidity(float value);
	void updatePressure(float value);
	void calculateData();
	int getDataString(char *buf, int size);
protected:
	WeatherData();
private:
	void semaphoreWait();
	void semaphoreRelease();
	float invertedRH(float es, float rh);
	float calcWetbulb(float temp, float press, float hum);
	float calcDewpoint(float temp, float hum);
	static WeatherData* _instance;
	float temperature;
	float humidity;
	float pressure;
	float dewpoint;
	float wetbulb;
	osSemaphoreId wthSemaphoreHandle;
};

#endif /* INC_WEATHERDATA_H_ */
