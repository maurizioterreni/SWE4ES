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
	HAL_I2C_Mem_Write(i2c, I2C_ADD, USER_REGISTER_W, 1, &data, 1, 1000);

	osDelay(d);

	HAL_I2C_Mem_Read(i2c, I2C_ADD, USER_REGISTER_R, 1, res, 3, 1000);

	result = (res[0] << 8);
	result += res[1];

	//	checksum = res[2];

	//	if(CRC_Checksum (data,2,checksum)) {
	//		reset();
	//		return 1;
	//	}

	//	else return result;

	return calcH(result);

}




float HumidityReader::calcH(uint16_t rh) {

	rh &= ~0x0003;	// clean last two bits

	return (-6.0 + 125.0/65536 * (float)rh); // return relative humidity
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

