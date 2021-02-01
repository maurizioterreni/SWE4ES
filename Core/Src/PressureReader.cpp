/*
 * PressureReader.cpp
 *
 *  Created on: Feb 1, 2021
 *      Author: maurizio
 */

#include <PressureReader.h>

PressureReader::PressureReader() {
}

PressureReader::~PressureReader() {
}

void PressureReader::init(I2C_HandleTypeDef *i2c) {
	readCalliberationData(i2c);
}

float PressureReader::read(I2C_HandleTypeDef* i2c) {
	int oss = 0;
	float press = .0f;
	UP = getUPress(i2c, oss);
	X1 = ((UT-AC6) * (AC5/(pow(2,15))));
	X2 = ((MC*(pow(2,11))) / (X1+MD));
	B5 = X1+X2;
	B6 = B5-4000;
	X1 = (B2 * (B6*B6/(pow(2,12))))/(pow(2,11));
	X2 = AC2*B6/(pow(2,11));
	X3 = X1+X2;
	B3 = (((AC1*4+X3)<<oss)+2)/4;
	X1 = AC3*B6/pow(2,13);
	X2 = (B1 * (B6*B6/(pow(2,12))))/(pow(2,16));
	X3 = ((X1+X2)+2)/pow(2,2);
	B4 = AC4*(unsigned long)(X3+32768)/(pow(2,15));
	B7 = ((unsigned long)UP-B3)*(50000>>oss);
	if (B7<0x80000000) {
		press = (B7*2)/B4;
	}
	else {
		press = (B7/B4)*2;
	}
	X1 = (press/(pow(2,8)))*(press/(pow(2,8)));
	X1 = (X1*3038)/(pow(2,16));
	X2 = (-7357*press)/(pow(2,16));
	return press + (X1+X2+3791)/(pow(2,4));
}

void PressureReader::readCalliberationData(I2C_HandleTypeDef *i2c) {
	uint8_t Callib_Data[22] = {0};
	uint16_t Callib_Start = 0xAA;
	HAL_I2C_Mem_Read(i2c, I2C_ADD, Callib_Start, 1, Callib_Data,22, HAL_MAX_DELAY);

	AC1 = ((Callib_Data[0] << 8) | Callib_Data[1]);
	AC2 = ((Callib_Data[2] << 8) | Callib_Data[3]);
	AC3 = ((Callib_Data[4] << 8) | Callib_Data[5]);
	AC4 = ((Callib_Data[6] << 8) | Callib_Data[7]);
	AC5 = ((Callib_Data[8] << 8) | Callib_Data[9]);
	AC6 = ((Callib_Data[10] << 8) | Callib_Data[11]);
	B1 = ((Callib_Data[12] << 8) | Callib_Data[13]);
	B2 = ((Callib_Data[14] << 8) | Callib_Data[15]);
	MB = ((Callib_Data[16] << 8) | Callib_Data[17]);
	MC = ((Callib_Data[18] << 8) | Callib_Data[19]);
	MD = ((Callib_Data[20] << 8) | Callib_Data[21]);
}

uint32_t PressureReader::getUPress(I2C_HandleTypeDef *i2c, int oss) {  // oversampling setting 0,1,2,3
	uint8_t datatowrite = 0x34+(oss<<6);
	uint8_t Press_RAW[3] = {0};
	HAL_I2C_Mem_Write(i2c, I2C_ADD, 0xF4, 1, &datatowrite, 1, 1000);
	uint32_t d = 0;
	if(oss == 0) {
		d = 5;
	} else if(oss == 1) {
		d = 8;
	} else if(oss == 2) {
		d = 14;
	} else if(oss == 3) {
		d = 26;
	}
	HAL_Delay(d);
	HAL_I2C_Mem_Read(i2c, I2C_ADD, 0xF6, 1, Press_RAW, 3, 1000);
	return (((Press_RAW[0]<<16)+(Press_RAW[1]<<8)+Press_RAW[2]) >> (8-oss));
}
