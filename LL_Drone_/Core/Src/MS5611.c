/*
 * MS5611.c
 *
 *  Created on: Jun 13, 2020
 *      Author: perSec
 */
#include "MS5611.h"


void MS_Writebyte(MS5611 * ms5611,uint8_t register_address,uint8_t data){
	uint8_t Trans[2]={register_address, data};
	I2C_Transmit(&ms5611->i2c,ms5611->i2c_address,Trans,2);
}



uint8_t MS_Readbyte(MS5611 * ms5611,uint8_t register_address){
	uint8_t Trans[1]={register_address};
	uint8_t Receive[1];
	I2C_Transmit(&ms5611->i2c,ms5611->i2c_address,Trans,1);
	I2C_Receive(&ms5611->i2c,ms5611->i2c_address,Receive,1);

	return Receive[0];
}

void MS_Command(MS5611 * ms5611, uint8_t command){
	I2C_Transmit(&ms5611->i2c,ms5611->i2c_address,&command,1);
}

void MS_ReadCoefficient(MS5611* ms5611){
	uint8_t command;
	uint8_t temp_coefficient[2];
	for(uint8_t i=0;i<6;i++){
		command=COEFFICIENT_ADD+((i+1)<<1);
		MS_Command(ms5611,command);
		I2C_Receive(&ms5611->i2c,ms5611->i2c_address,temp_coefficient,2);
		ms5611->coefficient[i]=(uint16_t)(temp_coefficient[0]<<8 | temp_coefficient[1]);
	}
}

void MS_init(MS5611*ms5611,I2C_TypeDef* i2c){
	LL_mDelay(100);

	ms5611->i2c.I2C=i2c;
	ms5611->i2c_address=MS5611_ADDRESS;
	ms5611->ms_condition=0;
	ms5611->pressure_ready_flag=0;
	MS_ReadCoefficient(ms5611);
}

void Calculate(MS5611*ms5611){
	int32_t dT,T2,TEMP;
	int64_t OFF,OFF2,SENS,SENS2;

	dT=(ms5611->D2) - (ms5611->coefficient[4] * (0x01<<8));
	TEMP=2000+(dT*ms5611->coefficient[5])/(0x01<<23);

	if(TEMP<2000){
		T2=(dT*dT)/(0x01<<31);
		OFF2=(5*(TEMP-2000)*(TEMP-2000))/(0x01<<1);
		SENS2=(5*(TEMP-2000)*(TEMP-2000))/(0x01<<2);
		if(TEMP<-1500){
			OFF2=OFF2+(7*(TEMP+1500)*(TEMP+1500));
			SENS2=SENS2+((11*(TEMP+1500)*(TEMP+1500))/(0x01<<1));
		}
	}
	else{
		T2=0;
		OFF2=0;
		SENS2=0;
	}

	OFF=((int64_t)ms5611->coefficient[1] *(0x01<<16)) +(((int64_t)ms5611->coefficient[3]*(int64_t)dT)/(0x01<<7));
	SENS=((int64_t)ms5611->coefficient[0]*(0x01<<15))+(((int64_t)ms5611->coefficient[2]*dT)/(0x01<<8));

	OFF=OFF-OFF2;
	SENS=SENS-SENS2;

	ms5611->temperature=TEMP-T2;
	ms5611->pressure=(((ms5611->D1*SENS)/(0X01<<21))-OFF)/(0x01<<15);
}

void Calculate_Temp_Pressure(MS5611 * ms5611){
	uint8_t temp[3];

	static int32_t p_mean[20]={0,};
	static int32_t p_sum=0;
	static uint8_t p_arr_count_20=0;

	if(BIT_VAL(ms5611->ms_condition,0)){
		CBI(ms5611->ms_condition,0);
		if(!BIT_VAL(ms5611->ms_condition,1)){
			MS_Command(ms5611,CONVERTD1_1024);
			SBI(ms5611->ms_condition,1);
		}
		else if(!BIT_VAL(ms5611->ms_condition,2)){
			MS_Command(ms5611,ADC_READ);
			I2C_Receive(&ms5611->i2c,ms5611->i2c_address,temp,3);
			ms5611->D1=(uint32_t)(temp[0]<<16 | temp[1]<<8 | temp[2] );
			MS_Command(ms5611,CONVERTD2_1024);
			SBI(ms5611->ms_condition,2);
		}
		else if(!BIT_VAL(ms5611->ms_condition,3)){
			MS_Command(ms5611,ADC_READ);
			I2C_Receive(&ms5611->i2c,ms5611->i2c_address,temp,3);
			ms5611->D2=(uint32_t)(temp[0]<<16 | temp[1]<<8 | temp[2] );
			Calculate(ms5611);

			p_sum+=ms5611->pressure;
			p_sum-=p_mean[p_arr_count_20];
			p_mean[p_arr_count_20]=ms5611->pressure;
			p_arr_count_20++;
			if(p_arr_count_20==20)
				p_arr_count_20=0;

			ms5611->pressure=p_sum/20;

			ms5611->pressure_ready_flag=1;
			ms5611->ms_condition=0;
		}

	}
}
