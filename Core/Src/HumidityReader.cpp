/*
 * HumidityReader.cpp
 *
 *  Created on: Feb 1, 2021
 *      Author: maurizio
 */

#include <HumidityReader.h>

HumidityReader::HumidityReader() {
	// TODO Auto-generated constructor stub

}

HumidityReader::~HumidityReader() {
	// TODO Auto-generated destructor stub
}

void HumidityReader::init(I2C_HandleTypeDef *i2c) {

}

float HumidityReader::read(I2C_HandleTypeDef *i2c) {

	//	uint8_t checksum;
	uint8_t res[3];
	uint16_t result;
	uint32_t d = 30;

	//	if(command == TRIGGER_RH_MEASUREMENT_HM || command == TRIGGER_RH_MEASUREMENT_NHM) d = 30;
	//	if(command == TRIGGER_T_MEASUREMENT_HM || command == TRIGGER_T_MEASUREMENT_NHM) d = 85;
	uint8_t data = TRIGGER_RH_MEASUREMENT_NHM;
	HAL_I2C_Mem_Write(i2c, HUM_ADD, USER_REGISTER_W, 1, &data, 1, HAL_MAX_DELAY);

	osDelay(d);

	HAL_I2C_Mem_Read(i2c, HUM_ADD, USER_REGISTER_R, 1, res, 3, HAL_MAX_DELAY);

	result = (res[0] << 8);
	result += res[1];

	//	checksum = res[2];

	//	if(CRC_Checksum (data,2,checksum)) {
	//		reset();
	//		return 1;
	//	}

	//	else return result;

	result = result & 0xFFFC;

	return calcH(result);

}




float HumidityReader::calcH(uint16_t rh) {

	float tempRH = rh * (125.0 / 65536.0); //2^16 = 65536
	return tempRH - 6.0; //From page 14
}

uint8_t HumidityReader::crcChecksum(uint8_t data[], uint8_t no_of_bytes, uint8_t checksum) {
	uint8_t crc = 0;
	uint8_t byteCtr;

	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < no_of_bytes; ++byteCtr)
	{ crc ^= (data[byteCtr]);
	for (uint8_t bit = 8; bit > 0; --bit)
	{ if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
	else crc = (crc << 1);
	}
	}
	if (crc != checksum) return 1;
	else return 0;
}

