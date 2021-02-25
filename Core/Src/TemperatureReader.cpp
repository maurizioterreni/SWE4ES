/*
 * TemperatureReader.cpp
 *
 *  Created on: Feb 1, 2021
 *      Author: maurizio
 */

#include "TemperatureReader.h"

TemperatureReader::TemperatureReader() {

}

TemperatureReader::~TemperatureReader() {
}

void TemperatureReader::init(I2C_HandleTypeDef *i2c) {

}

float TemperatureReader::read(I2C_HandleTypeDef *i2c) {
	//	uint8_t checksum;
	uint8_t res[3];
	uint16_t result;
	uint32_t d = 85;

	//	if(command == TRIGGER_RH_MEASUREMENT_HM || command == TRIGGER_RH_MEASUREMENT_NHM) d = 30;
	//	if(command == TRIGGER_T_MEASUREMENT_HM || command == TRIGGER_T_MEASUREMENT_NHM) d = 85;
	uint8_t data = TEMP_POLL;
	HAL_I2C_Mem_Write(i2c, I2C_ADR, USR_REG_WRITE, 1, &data, 1, 1);

	osDelay(d);

	HAL_I2C_Mem_Read(i2c, I2C_ADR, USR_REG_READ, 1, res, 3, 1);

	result = (res[0] << 8);
	result += res[1];

//	if (crcChecksum(rawValue, checksum) != 0) {
//		return -100.0f;
//	}
	//	checksum = res[2];

	//	if(CRC_Checksum (data,2,checksum)) {
	//		reset();
	//		return 1;
	//	}

	//	else return result;

	result = result & 0xFFFC;

	float temp = calcT(result);

	return temp;
}



float TemperatureReader::calcT(uint16_t t) {

	float tempTemperature = t * (175.72 / 65536.0); //2^16 = 65536

	return tempTemperature - 46.85;
}

uint8_t TemperatureReader::crcChecksum(uint16_t message_from_sensor, uint8_t check_value_from_sensor) {
	//Test cases from datasheet:
	//message = 0xDC, checkvalue is 0x79
	//message = 0x683A, checkvalue is 0x7C
	//message = 0x4E85, checkvalue is 0x6B

	uint32_t remainder = (uint32_t)message_from_sensor << 8; //Pad with 8 bits because we have to add in the check value
	remainder |= check_value_from_sensor; //Add on the check value

	uint32_t divsor = (uint32_t) 0x988000;

	for (int i = 0 ; i < 16 ; i++) //Operate on only 16 positions of max 24. The remaining 8 are our remainder and should be zero when we're done.
	{
		//Serial.print("remainder: ");
		//Serial.println(remainder, BIN);
		//Serial.print("divsor:    ");
		//Serial.println(divsor, BIN);
		//Serial.println();

		if ( remainder & (uint32_t)1 << (23 - i) ) //Check if there is a one in the left position
			remainder ^= divsor;

		divsor >>= 1; //Rotate the divsor max 16 times so that we have 8 bits left of a remainder
	}

	return (uint8_t)remainder;
}
