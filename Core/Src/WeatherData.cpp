/*
 * WeatherData.cpp
 *
 *  Created on: Feb 2, 2021
 *      Author: maurizio
 */

#include <WeatherData.h>

WeatherData* WeatherData::_instance = 0;

WeatherData* WeatherData::getInstance() {
	if (_instance == 0){
		_instance = new WeatherData();
	}

	return _instance;
}

WeatherData::WeatherData() {
	osSemaphoreDef(wthSemaphore);
	wthSemaphoreHandle = osSemaphoreCreate(osSemaphore(wthSemaphore), 1);

	temperature = -100.0f;
	humidity = -1.0f;
	pressure = -1.0f;
}

WeatherData::~WeatherData() {

}

void WeatherData::updateTemperature(float value) {
	semaphoreWait();
	temperature = temperature < -99.0 ? temperature : (temperature + value) / 2;
	semaphoreRelease();
}

void WeatherData::updateHumidity(float value) {
	semaphoreWait();
	humidity = humidity < -1.0 ? humidity : (humidity + value) / 2;
	semaphoreRelease();
}

void WeatherData::updatePressure(float value) {
	semaphoreWait();
	pressure = pressure < -1.0 ? pressure : (pressure + value) / 2;
	semaphoreRelease();
}

void WeatherData::calculateData() {
}



void WeatherData::semaphoreWait() {
	osSemaphoreWait(wthSemaphoreHandle, osWaitForever);
}

void WeatherData::semaphoreRelease() {
	osSemaphoreRelease(wthSemaphoreHandle);
}
