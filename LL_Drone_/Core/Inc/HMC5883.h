/*
 * HMC5883.h
 *
 *  Created on: Apr 14, 2020
 *      Author: perSec
 */



#include "I2C.h"

#include "main.h"


#ifndef HMC5883_H_
#define HMC5883_H_

#define HMC5883_ADDRESS  0b00011110<<1
#define mag_para_size 6


#define offset_x mag_para_arr[0]
#define offset_y mag_para_arr[1]
#define offset_z mag_para_arr[2]
#define scale_x mag_para_arr[3]
#define scale_y mag_para_arr[4]
#define scale_z mag_para_arr[5]


typedef struct{
	I2C_struct i2c;
	uint8_t magneto_address;
	float mx, my, mz;
	float mag_para_arr[mag_para_size];
}HMC5883;


HMC5883 hmc5883;


void Mag_Writebyte(HMC5883 * I2C,uint8_t register_address,uint8_t data);

uint8_t Mag_Readbyte(HMC5883* I2C, uint8_t register_address);

uint8_t HMC_Status(HMC5883* I2C);

uint8_t HMC_ID(HMC5883* I2C, uint8_t* data);

void init_HMC(HMC5883* hmc5883,I2C_TypeDef* i2c);

void HMC_Mag_Read(HMC5883* hmc5883);

void HMC_Default_Reg(HMC5883* I2C);


#endif /* HMC5883_H_ */

