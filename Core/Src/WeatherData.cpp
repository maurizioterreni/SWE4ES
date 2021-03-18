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
	dewpoint = calcDewpoint(temperature, humidity);
	wetbulb = calcWetbulb(temperature, pressure, humidity);
}



void WeatherData::semaphoreWait() {
	osSemaphoreWait(wthSemaphoreHandle, osWaitForever);
}

void WeatherData::semaphoreRelease() {
	osSemaphoreRelease(wthSemaphoreHandle);
}


float WeatherData::invertedRH(float temp, float rh) {
	return temp * (rh/100);
}


float WeatherData::calcWetbulb(float temp, float press, float hum) {
	float res = 0.025100616f;
	return invertedRH(temp, hum) - res - press * temp * 0.00066f;
}


float WeatherData::calcDewpoint(float temp, float hum) {
	return (temp - (14.55 + 0.114 * temp) * (1 - (0.01 * hum)) - pow(((2.5 + 0.007 * temp) * (1 - (0.01 * hum))),3) - (15.9 + 0.117 * temp) * pow((1 - (0.01 * hum)), 14));
}


int WeatherData::getDataString(char *buf, int size) {
	semaphoreWait();
	int len = snprintf(buf, size, "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n",
			temperature,
			humidity,
			pressure,
			wetbulb,
			dewpoint);
	semaphoreRelease();
	return len;
}
