/*
 * util.h
 *
 *  Created on: 2020. 7. 23.
 *      Author: perSec
 */
#include "main.h"
#include "spi_flash.h"
#include "PID.h"
#include "MPU6050.h"
#include "HMC5883.h"
#include "GPS.h"
#include "NRF24L01.h"
#include "MS5611.h"
#include "mode.h"

#ifndef INC_UTIL_H_
#define INC_UTIL_H_


#define pid_flash_address 0x0000
#define pid_flash_size pid_para_size*4

#define mag_flash_address 0x1000
#define mag_flash_size mag_para_size*4

#define gps_flash_address 0x2000
#define gps_flash_size gps_para_size*4

#define baro_flash_address 0x3000
#define baro_flash_size baro_para_size*4

typedef struct{
	float pitch;
	float roll;
	float yaw;
	float dif_yaw;

	float pre_pitch;
	float pre_roll;
	float pre_dif_yaw;

	float set_pitch;
	float set_roll;
	float set_yaw;

	int32_t throttle;
	int32_t pressure_throttle;

}STATUS_DATA;

typedef struct{
	USART_TypeDef* UART;
	uint8_t command_buf[30];
	uint16_t command_length;
	uint8_t command_receive;
}UART_COMMAND;


UART_COMMAND command;

STATUS_DATA status_data;



int32_t motor[6];
uint32_t timer[6];


void init_COMMAND(UART_COMMAND* command,USART_TypeDef* uart);

void COMMAND_Decode(UART_COMMAND* command);


void save_pid_para();
void load_pid_para();

void save_mag_para();
void load_mag_para();

void save_GPS_para();
void load_GPS_para();

void save_baro_para();
void load_baro_para();

#endif /* INC_UTIL_H_ */
