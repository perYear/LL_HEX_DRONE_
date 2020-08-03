/*
 * MPU6050.h
 *
 *  Created on: 2020. 4. 14.
 *      Author: perSec
 */
#include "MPU6050REG.h"


#include"main.h"

#include<math.h>

#include "I2C.h"

#include "init.h"

#ifndef MPU6050_H_
#define MPU6050_H_

#define MPU6050_ADDRESS 0b1101000<<1 // AD0pin = 0
#define alpha 0.9996



typedef struct __MPU6050{
	I2C_struct I2C;

	uint8_t gyro_address;

}MPU6050;

typedef struct{
	int32_t getmpuaccx,getmpuaccy,getmpuaccz;

	float pitch, roll;
	float f_gyx, f_gyy,f_gyz;

	float pitch_offset, roll_offset;		//pitch, roll
	int16_t gyro_offset[3];				//x,y,z

} ANGLE;

MPU6050 mpu6050;
ANGLE angle;

void Gyro_Writebyte(MPU6050 * mpu6050,uint8_t register_address,uint8_t data);

uint8_t Gyro_Readbyte(MPU6050 * mpu6050,uint8_t register_address);


void MPU_Reset(MPU6050* mpu6050);


void init_MPU6050(MPU6050* mpu6050, I2C_TypeDef* i2c);

void MPU_Readaccgyro(MPU6050* mpu6050,ANGLE* angle);


void MPU_Gyrocali(MPU6050* mpu6050,ANGLE* angle);

void MPU_Angcali(MPU6050* mpu6050,ANGLE* angle);

void MPU_Complementary_Filter(ANGLE* angle,uint8_t* raw_data);

#endif /* MPU6050_H_ */

