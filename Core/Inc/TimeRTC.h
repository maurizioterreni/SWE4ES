/*
 * TimeRTC.h
 *
 *  Created on: Jan 23, 2021
 *      Author: maurizio
 */

#ifndef INC_TIMERTC_H_
#define INC_TIMERTC_H_

#include "stm32f1xx_hal.h"
#include <time.h>
#include <stdio.h>

class TimeRTC {
public:
	virtual ~TimeRTC();
	void setTime(uint8_t hour, uint8_t minute, uint8_t seconds, uint8_t weekDay, uint8_t month, uint8_t date, uint8_t year);
	void setTime(time_t now);
	void getTime(char *buffer, int length);
	void getDateTime(char *buffer, int length);
	void getDate(char *buffer, int length);
	time_t getTimestamp();
	static TimeRTC* getInstance();
	void init(RTC_HandleTypeDef *h);
private:
	static TimeRTC* instance;
	TimeRTC();
	RTC_HandleTypeDef *hrtc;
};

#endif /* INC_TIMERTC_H_ */
