/*
 * TimeRTC.cpp
 *
 *  Created on: Jan 23, 2021
 *      Author: maurizio
 */

#include <TimeRTC.h>

TimeRTC* TimeRTC::instance = 0;

TimeRTC::TimeRTC() {

}

TimeRTC::~TimeRTC() {
}

void TimeRTC::init(RTC_HandleTypeDef *h) {
	hrtc = h;
}

TimeRTC* TimeRTC::getInstance() {
	if (instance == 0){
		instance = new TimeRTC();
	}

	return instance;
}

void TimeRTC::setTime(uint8_t hour, uint8_t minute, uint8_t seconds, uint8_t weekDay, uint8_t month, uint8_t date, uint8_t year) {
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	sTime.Hours = hour; // set hours
	sTime.Minutes = 0x20; // set minutes
	sTime.Seconds = 0x30; // set seconds
//	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//	sTime.StoreOperation = RTC_STOREOPERATION_RESET;

	if (HAL_RTC_SetTime(hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {

	}
	sDate.WeekDay = RTC_WEEKDAY_THURSDAY; //day
	sDate.Month = RTC_MONTH_AUGUST; //month
	sDate.Date = 0x9; // date
	sDate.Year = 0x18; // year
	if (HAL_RTC_SetDate(hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK) {
//		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_RTCEx_BKUPWrite(hrtc, RTC_BKP_DR1, 0x32F2); // backup register
}

void TimeRTC::setTime(time_t now) {

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	struct tm time_tm;
	time_tm = *(localtime(&now));

	sTime.Hours = (uint8_t)time_tm.tm_hour;
	sTime.Minutes = (uint8_t)time_tm.tm_min;
	sTime.Seconds = (uint8_t)time_tm.tm_sec;

	sDate.WeekDay = (uint8_t)time_tm.tm_wday;
	sDate.Month = (uint8_t)time_tm.tm_mon+1; //momth 1- This is why date math is frustrating.
	sDate.Date = (uint8_t)time_tm.tm_mday;
	sDate.Year = (uint16_t)(time_tm.tm_year+1900-2000); // time.h is years since 1900, chip is years since 2000

	setTime(sTime.Hours, sTime.Minutes, sTime.Seconds, sDate.WeekDay, sDate.Month, sDate.Date, sDate.Year);
}


void TimeRTC::getTime(char *buffer, int length) {
	RTC_TimeTypeDef gTime;
	HAL_RTC_GetTime(hrtc, &gTime, RTC_FORMAT_BIN);
	sprintf((char*)buffer,"%02d:%02d:%02d", gTime.Hours, gTime.Minutes, gTime.Seconds);
}


void TimeRTC::getDateTime(char *buffer, int length) {
	RTC_DateTypeDef gDate;
	RTC_TimeTypeDef gTime;
	HAL_RTC_GetTime(hrtc, &gTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, &gDate, RTC_FORMAT_BIN);
	sprintf((char*)buffer,"%02d-%02d-%2d %02d:%02d:%02d", gDate.Date, gDate.Month, 2000 + gDate.Year, gTime.Hours, gTime.Minutes, gTime.Seconds);
}


void TimeRTC::getDate(char *buffer, int length) {
	RTC_DateTypeDef gDate;
	HAL_RTC_GetDate(hrtc, &gDate, RTC_FORMAT_BIN);
	sprintf((char*)buffer,"%02d-%02d-%2d", gDate.Date, gDate.Month, 2000 + gDate.Year);
}


time_t TimeRTC::getTimestamp() {
	RTC_TimeTypeDef currentTime;
	RTC_DateTypeDef currentDate;
	struct tm currTime;


	/* Code to get timestamp
	 *
	 *  You must call HAL_RTC_GetDate() after HAL_RTC_GetTime() to unlock the values
	 *  in the higher-order calendar shadow registers to ensure consistency between the time and date values.
	 *  Reading RTC current time locks the values in calendar shadow registers until Current date is read
	 *  to ensure consistency between the time and date values.
	 */

	HAL_RTC_GetTime(hrtc, &currentTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, &currentDate, RTC_FORMAT_BIN);

	currTime.tm_year = currentDate.Year + 100;  // In fact: 2000 + 18 - 1900
	currTime.tm_mday = currentDate.Date;
	currTime.tm_mon  = currentDate.Month - 1;

	currTime.tm_hour = currentTime.Hours;
	currTime.tm_min  = currentTime.Minutes;
	currTime.tm_sec  = currentTime.Seconds;

	return mktime(&currTime);
}


