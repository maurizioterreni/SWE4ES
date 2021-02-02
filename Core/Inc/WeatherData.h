/*
 * WeatherData.h
 *
 *  Created on: Feb 2, 2021
 *      Author: maurizio
 */

#ifndef INC_WEATHERDATA_H_
#define INC_WEATHERDATA_H_

#include "cmsis_os.h"

class WeatherData {
public:
	virtual ~WeatherData();
	static WeatherData* getInstance();
	void updateTemperature(float value);
	void updateHumidity(float value);
	void updatePressure(float value);
	void calculateData();
protected:
	WeatherData();
private:
	void semaphoreWait();
	void semaphoreRelease();
	static WeatherData* _instance;
	float temperature;
	float humidity;
	float pressure;
	float dewpoint;
	float wetbulb;
	osSemaphoreId wthSemaphoreHandle;
};

#endif /* INC_WEATHERDATA_H_ */
