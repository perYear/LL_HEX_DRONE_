/*
 * GPS.h
 *
 *  Created on: Aug 3, 2020
 *      Author: perSec
 */

#include "stm32f407xx.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_usart.h"

#ifndef INC_GPS_H_
#define INC_GPS_H_

#define GPS_MESSAGE_LENGTH 36

#define LATITUDE_OFFSET 33
#define LONGITUDE_OFFSET 124

#define gps_pitch_p_bound 10.0
#define gps_roll_p_bound 10.0

#define gps_pitch_speed_bound 4.0
#define gps_roll_speed_bound 4.0

#define gps_pitch_bound 7
#define gps_roll_bound 7

#define gps_para_size 3

#define gps_p gps_para[0]
#define gps_i gps_para[1]
#define gps_d gps_para[2]


typedef struct{
	USART_TypeDef* UART;
	DMA_TypeDef* DMA;
	uint8_t gps_raw_buf[GPS_MESSAGE_LENGTH+1];
}GPS_RAW_MESSAGE;

typedef struct{
	int32_t lon;
	int32_t lat;

	int32_t longitude_deg,longitude_min;
	float longitude_sec;

	int32_t latitude_deg,latitude_min;
	float latitude_sec;

	float sec_lon;
	float sec_lat;
}GPS_DATA;




GPS_RAW_MESSAGE gps_raw_message;

GPS_DATA gps_data;

float gps_para[gps_para_size];


void init_GPS(GPS_RAW_MESSAGE* gps_raw_message, USART_TypeDef* UART,DMA_TypeDef* DMA,uint32_t DMA_STREAM);

void GPS_Parsing(GPS_RAW_MESSAGE* message, GPS_DATA* gps_data);


#endif /* INC_GPS_H_ */
