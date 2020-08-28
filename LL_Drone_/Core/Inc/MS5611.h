/*
 * MS5611.h
 *
 *  Created on: Jun 13, 2020
 *      Author: perSec
 */
#include "stm32f4xx_ll_utils.h"
#include "I2C.h"


#define SBI(address, ab) ( address |= (0x01<<ab) )
#define CBI(address, ab) ( address &= ~(0x01<<ab) )
#define BIT_VAL(reg, n) ((reg>>n)&0x01)

#ifndef INC_MS5611_H_
#define INC_MS5611_H_

#define MS5611_ADDRESS 0x77<<1 //CSB 0

#define CONVERTD1_1024 0X44
#define CONVERTD2_1024 0X54
#define COEFFICIENT_ADD 0XA0
#define ADC_READ 0x00

#define baro_para_size 3
#define baro_p baro_para[0]
#define baro_i baro_para[1]
#define baro_d baro_para[2]

typedef struct __MS5611{
	I2C_struct i2c;
	uint8_t i2c_address;
	uint16_t coefficient[6];
	uint32_t D1;
	uint32_t D2;
	int32_t temperature;
	int32_t pressure;
	uint8_t ms_condition;
	uint8_t pressure_ready_flag;
}MS5611;

MS5611 ms5611;
float baro_para[3];

float pressure_slow,pressure_fast;
float actual_pressure,pre_actual_pressure;

void MS_Writebyte(MS5611 * ms5611,uint8_t register_address,uint8_t data);

uint8_t MS_Readbyte(MS5611 * ms5611,uint8_t register_address);

void MS_init(MS5611*ms5611,I2C_TypeDef* i2c);

void Calculate_Temp_Pressure(MS5611 * ms5611);


#endif /* INC_MS5611_H_ */
