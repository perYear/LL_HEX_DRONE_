/*
 * HMC5883.c
 *
 *  Created on: 2020. 4. 13.
 *      Author: perSec
 */


#ifndef HMC5883_H_
#include <HMC5883.h>
#endif

extern uint8_t busy_flag;

void Mag_Writebyte(HMC5883 * I2C,uint8_t register_address,uint8_t data){
	uint8_t Trans[2]={register_address, data};
	I2C_Transmit(&I2C->i2c,I2C->magneto_address,Trans,2);
	//HAL_I2C_Master_Transmit(I2C->i2c, I2C->magneto_address,Trans,2,10);
}

uint8_t Mag_Readbyte(HMC5883* I2C, uint8_t register_address){
	uint8_t Trans[1]={register_address};
	uint8_t Receive[1];

	I2C_Transmit(&I2C->i2c,I2C->magneto_address,Trans,1);
	I2C_Receive(&I2C->i2c,I2C->magneto_address,Receive,1);
	return Receive[0];
}

uint8_t HMC_Status(HMC5883* I2C){
	uint8_t status;
	status=Mag_Readbyte(I2C,0x09);
	return status;
}

uint8_t HMC_ID(HMC5883* I2C, uint8_t* data){
	data[0]=Mag_Readbyte(I2C,0x0a);
	data[1]=Mag_Readbyte(I2C,0x0b);
	data[2]=Mag_Readbyte(I2C,0x0c);
}

void init_HMC(HMC5883* hmc5883,I2C_TypeDef* i2c){
	hmc5883->i2c.I2C=i2c;
	hmc5883->i2c.i2c_IsRx=0;
	hmc5883->i2c.i2c_busy=0;
	hmc5883->magneto_address=HMC5883_ADDRESS;

	Mag_Writebyte(hmc5883,0x00,0b00011000);
	Mag_Writebyte(hmc5883,0x01,0b00100000);
	Mag_Writebyte(hmc5883,0x02,0x00);
}

void HMC_Mag_Read(HMC5883* hmc5883){
	uint8_t Receive[6];
	uint8_t Trans[1];
	int16_t output[3];

	Trans[0]=0x03;
	I2C_Transmit(&hmc5883->i2c,hmc5883->magneto_address,Trans,1);
	I2C_Receive(&hmc5883->i2c,hmc5883->magneto_address,Receive,6);

	output[0]=(int16_t)((Receive[0]<<8) | Receive[1]);
	output[1]=(int16_t)((Receive[2]<<8) | Receive[3]);
	output[2]=(int16_t)((Receive[4]<<8) | Receive[5]);
	hmc5883->mx=(float)output[0];
	hmc5883->my=(float)output[2];
	hmc5883->mz=(float)output[1];
}

void HMC_Default_Reg(HMC5883* I2C){
	uint8_t reg;
	for(reg=0;reg<13;reg++){
		printf("%d: %.2x\r\n",reg,Mag_Readbyte(I2C,reg));
	}

}

