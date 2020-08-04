/*
 * MPU6050.c
 *
 *  Created on: 2020. 4. 14.
 *      Author: perSec
 */


#include "MPU6050.h"
#include <math.h>

void Gyro_Writebyte(MPU6050 * mpu6050,uint8_t register_address,uint8_t data){
	uint8_t Trans[2]={register_address, data};
	I2C_Transmit(&mpu6050->I2C, mpu6050->gyro_address,Trans,2);
}
uint8_t Gyro_Readbyte(MPU6050 * mpu6050,uint8_t register_address){
	uint8_t Trans[1]={register_address};
	uint8_t Receive[1];
	I2C_Transmit(&mpu6050->I2C, mpu6050->gyro_address,Trans,1);
	I2C_Receive(&mpu6050->I2C,mpu6050->gyro_address,Receive,1);

	return Receive[0];
}

void MPU_Reset(MPU6050* mpu6050){
	Gyro_Writebyte(mpu6050,MPU6050_RA_PWR_MGMT_1,0x80);
}


void init_MPU6050(MPU6050* mpu6050, I2C_TypeDef* i2c){
	mpu6050->I2C.I2C=i2c;
	mpu6050->I2C.i2c_busy=0;
	mpu6050->I2C.i2c_IsRx=0;
	mpu6050->gyro_address=MPU6050_ADDRESS;

	LL_mDelay(1000);
	Gyro_Writebyte(mpu6050,MPU6050_RA_PWR_MGMT_1,0x00);

	LL_mDelay(100);
	Gyro_Writebyte(mpu6050,MPU6050_RA_PWR_MGMT_1,0x01);

	Gyro_Writebyte(mpu6050,MPU6050_RA_SMPLRT_DIV,0x07);

	Gyro_Writebyte(mpu6050,MPU6050_RA_GYRO_CONFIG,0x08);

	Gyro_Writebyte(mpu6050,MPU6050_RA_ACCEL_CONFIG,0x08);
}

void MPU_Readaccgyro(MPU6050* mpu6050,ANGLE* angle){
	int16_t temp[3];
	int16_t tempmpuaccx,tempmpuaccy,tempmpuaccz;
	uint8_t databuf[14];


	databuf[0]=MPU6050_RA_GYRO_XOUT_H;
	I2C_Transmit(&mpu6050->I2C,mpu6050->gyro_address,databuf,1);
	I2C_Receive(&mpu6050->I2C,mpu6050->gyro_address,databuf,14);

	tempmpuaccx=(int16_t)(databuf[00]<<8 | databuf[1]);
	tempmpuaccy=(int16_t)(databuf[2]<<8 | databuf[3]);
	tempmpuaccz=(int16_t)(databuf[4]<<8 | databuf[5]);
	temp[0]=(int16_t)(databuf[8]<<8 | databuf[9]);
	temp[1]=(int16_t)(databuf[10]<<8 | databuf[11]);
	temp[2]=(int16_t)(databuf[12]<<8 | databuf[13]);
	angle->getmpuaccx=tempmpuaccx;
	angle->getmpuaccy=tempmpuaccy;
	angle->getmpuaccz=tempmpuaccz;
	angle->f_gyx=((float)(temp[0]-angle->gyro_offset[0]))/65.5;
	angle->f_gyy=((float)(temp[1]-angle->gyro_offset[1]))/65.5;
	angle->f_gyz=((float)(temp[2]-angle->gyro_offset[2]))/65.5;
	/*printf("%d %d %d\r\n",mpu6050->getmpuaccx,mpu6050->getmpuaccy,mpu6050->getmpuaccz);
	printf("%.1f %.1f %.1f\r\n\n\r",mpu6050->f_gyx,mpu6050->f_gyy,mpu6050->f_gyz);*/

}


void MPU_Gyrocali(MPU6050* mpu6050,ANGLE* angle){
	int32_t temp32[3]={0,0,0};
	int16_t temp16[3];
	uint8_t databuf[6];
	LL_mDelay(100);
	for(uint32_t i=0;i<2000;i++){

		if(i%100==0)
			printf("%d\r\n",i);
		databuf[0]=MPU6050_RA_GYRO_XOUT_H;
		I2C_Transmit(&mpu6050->I2C,mpu6050->gyro_address,databuf,1);
		I2C_Receive(&mpu6050->I2C,mpu6050->gyro_address,databuf,6);

		temp16[0]=(int16_t)(databuf[0]<<8 | databuf[1]);
		temp16[1]=(int16_t)(databuf[2]<<8 | databuf[3]);
		temp16[2]=(int16_t)(databuf[4]<<8 | databuf[5]);

		temp32[0]+=temp16[0];
		temp32[1]+=temp16[1];
		temp32[2]+=temp16[2];
		/*printf("%d %d %d\r\n",databuf[8],databuf[9],databuf[10]);
		printf("%d %d %d\r\n",databuf[11],databuf[12],databuf[13]);
		printf("%d %d %d\r\n\n\n\r",temp32[0],temp32[1],temp32[2]);*/
	}
	angle->gyro_offset[0]=temp32[0]/2000;
	angle->gyro_offset[1]=temp32[1]/2000;
	angle->gyro_offset[2]=temp32[2]/2000;
	printf("gy %d %d %d\r\n",angle->gyro_offset[0],angle->gyro_offset[1],angle->gyro_offset[2]);
}

void MPU_Angcali(MPU6050* mpu6050,ANGLE* angle){
	uint8_t databuf[14];
	int16_t temp[6];
	int32_t gettemp[6];
	float temp_offset[2];
	float acctotvec,accdegx,accdegy;

	for(uint32_t i=0;i<1000;i++){
		if(i%100==0)
			printf("%d\r\n",i);

		databuf[0]=MPU6050_RA_ACCEL_XOUT_H;
		I2C_Transmit(&mpu6050->I2C,mpu6050->gyro_address,databuf,1);
		I2C_Receive(&mpu6050->I2C,mpu6050->gyro_address,databuf,14);

		temp[0]=((databuf[0]<<8)|databuf[1]);
		temp[1]=((databuf[2]<<8)|databuf[3]);
		temp[2]=((databuf[4]<<8)|databuf[5]);
		temp[3]=((databuf[8]<<8)|databuf[9]);
		temp[4]=((databuf[10]<<8)|databuf[11]);
		temp[5]=((databuf[12]<<8)|databuf[13]);
		gettemp[0]=temp[0];
		gettemp[1]=temp[1];
		gettemp[2]=temp[2];
		gettemp[3]=(temp[3]-angle->gyro_offset[0])/65;
		gettemp[4]=(temp[4]-angle->gyro_offset[0])/65;
		gettemp[5]=(temp[5]-angle->gyro_offset[0])/65;

		acctotvec=sqrt((float)(gettemp[0]*gettemp[0]/100+gettemp[1]*gettemp[1]/100+gettemp[2]*gettemp[2]/100))*10;
		accdegx=asin((float)gettemp[0]/acctotvec)*(57.29577951);
		accdegy=asin((float)gettemp[1]/acctotvec)*(57.29577951);
		temp_offset[0]+=accdegx;
		temp_offset[1]+=accdegy;
	}
	angle->pitch_offset=temp_offset[0]/1000;
	angle->roll_offset=temp_offset[1]/1000;
	angle->pitch=angle->pitch_offset;
	angle->roll=angle->roll_offset;
	printf("p:%.1f r:%.1f\r\n",angle->pitch_offset,angle->roll_offset);
}

void MPU_Complementary_Filter(ANGLE* angle,uint8_t* raw_data){
	float accdegx,accdegy,acctotvec;
	float pitch, roll;
	//float gyx, gyy;
	float sin_pitch,cos_pitch,tan_roll;

	int16_t temp[3];
	int16_t tempmpuaccx,tempmpuaccy,tempmpuaccz;
	tempmpuaccx=(int16_t)(raw_data[0]<<8 | raw_data[1]);
	tempmpuaccy=(int16_t)(raw_data[2]<<8 | raw_data[3]);
	tempmpuaccz=(int16_t)(raw_data[4]<<8 | raw_data[5]);
	temp[0]=(int16_t)(raw_data[8]<<8 | raw_data[9]);
  	temp[1]=(int16_t)(raw_data[10]<<8 | raw_data[11]);
  	temp[2]=(int16_t)(raw_data[12]<<8 | raw_data[13]);

  	angle->getmpuaccx=tempmpuaccx;
  	angle->getmpuaccy=tempmpuaccy;
  	angle->getmpuaccz=tempmpuaccz;
  	angle->f_gyx=((float)(temp[0]-angle->gyro_offset[0]))/65.5;
  	angle->f_gyy=((float)(temp[1]-angle->gyro_offset[1]))/65.5;
  	angle->f_gyz=((float)(temp[2]-angle->gyro_offset[2]))/65.5;

  	pitch = angle->pitch - angle->pitch_offset;
  	roll = angle->roll - angle->roll_offset;

  	sin_pitch=sinf(pitch/57.29577951);
  	cos_pitch=cosf(pitch/57.29577951);
  	tan_roll=tanf(roll/57.29577951);

  	angle->f_gyx = angle->f_gyx + (sin_pitch * angle->f_gyy + cos_pitch * angle->f_gyz) * tan_roll;
  	angle->f_gyy = cos_pitch * angle->f_gyy - sin_pitch * angle->f_gyz;


  	acctotvec=sqrtf((float)(angle->getmpuaccx*angle->getmpuaccx/100
  			+angle->getmpuaccy*angle->getmpuaccy/100
			+angle->getmpuaccz*angle->getmpuaccz/100))*10;
  	accdegx=asinf((float)angle->getmpuaccx/acctotvec)*(57.29577951);
  	accdegy=asinf((float)angle->getmpuaccy/acctotvec)*(57.29577951);

  	angle->pitch=(alpha)*(angle->pitch-(angle->f_gyy)*dt)+(1-alpha)*(accdegx);
  	angle->roll=(alpha)*(angle->roll+(angle->f_gyx)*dt)+(1-alpha)*(accdegy);


}


