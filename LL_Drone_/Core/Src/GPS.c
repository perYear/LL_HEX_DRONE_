/*
 * GPS.c
 *
 *  Created on: Aug 3, 2020
 *      Author: perSec
 */

#include "GPS.h"

#include "main.h"

void UART_Transmit(USART_TypeDef *USARTx, uint8_t * data, uint16_t length){
    uint16_t i=0;
    for(i=0;i<length;i++){
        LL_USART_TransmitData8(USARTx,data[i]);
        while(!LL_USART_IsActiveFlag_TXE(USARTx));
    }
}


void init_GPS(GPS_RAW_MESSAGE* gps_raw_message, USART_TypeDef* UART,DMA_TypeDef* DMA,uint32_t DMA_STREAM){

	//{0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x80,0x25,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x9A,0x79};//nmea to ubx protocol change(bitrate:9600)
	//{0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12};	//message rate 10hz
	//{0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A};	//message rate 5hz
	//{0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE};//nav-posllh message
	//{0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x00,0x01,0x00,0x00,0x00,0x00,0x23,0x2E};//nav-velned message
	//{0xB5,0x62,0x06,0x09,0x0D,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0x31,0xBF};	//save current configuration

	uint8_t NMEA_to_UBX[]={0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,
							0x00,0x00,0xD0,0x08,0x00,0x00,0x80,0x25,
							0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,
							0x00,0x00,0x9A,0x79};//nmea to ubx (bitrate:9600)
	uint8_t Message_Rate[]={0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A};	//message rate 5hz
	uint8_t GPS_Message_POSLLH[]={0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE};//nav_posllh message

	gps_raw_message->UART=UART;
	gps_raw_message->DMA=DMA;
	LL_mDelay(500);
	UART_Transmit(UART,NMEA_to_UBX,28);
	LL_mDelay(100);
	UART_Transmit(UART,Message_Rate,14);
	LL_mDelay(100);
	UART_Transmit(UART,GPS_Message_POSLLH,16);
	LL_mDelay(100);

	//DMA, INTERRUPT SETTINGS
	LL_DMA_SetMemoryAddress(DMA,DMA_STREAM,(uint32_t)(gps_raw_message->gps_raw_buf));
	LL_DMA_SetPeriphAddress(DMA,DMA_STREAM,(uint32_t)&(UART->DR));
	LL_DMA_SetDataLength(DMA,DMA_STREAM,GPS_MESSAGE_LENGTH+1);

	//DMA->LIFCR=0x003D0000;	// STREAM2 CLEAR ALL FLAG
	DMA->HIFCR=0x00000F00;	// STREAM5 CLEAR ALL FLAG

	LL_DMA_EnableStream(DMA,DMA_STREAM);
	LL_USART_EnableDMAReq_RX(UART);
	LL_USART_EnableIT_IDLE(UART);
}

void GPS_Parsing(GPS_RAW_MESSAGE* message, GPS_DATA* gps_data){
	uint8_t* ptr,*gps_ptr=message->gps_raw_buf;
	int32_t temp;
	if(gps_ptr[0]==0xB5 && gps_ptr[1]==0x62){
		ptr=gps_ptr+6+4;
		gps_data->lon = (ptr[3] << 24) + (ptr[2] << 16) + (ptr[1] << 8) + (ptr[0]);
		gps_data->longitude_deg=gps_data->lon/10000000;
		temp=gps_data->lon%10000000;
		gps_data->longitude_min=(temp*60)/10000000;
		temp=(temp*60)%10000000;
		gps_data->longitude_sec=((float)(temp*60))/10000000;

		if(gps_data->longitude_deg<124 || gps_data->longitude_deg>132){
			gps_data->sec_lon=0;
		}
		else{
			gps_data->sec_lon = (float)(gps_data->longitude_deg - LONGITUDE_OFFSET) * 3600 + (float)gps_data->longitude_min * 60 + gps_data->longitude_sec;
		}

		ptr += 4;
		gps_data->lat = (ptr[3] << 24) + (ptr[2] << 16) + (ptr[1] << 8) + (ptr[0]);
		gps_data->latitude_deg=gps_data->lat/10000000;
		temp=gps_data->lat%10000000;
		gps_data->latitude_min=(temp*60)/10000000;
		temp=(temp*60)%10000000;
		gps_data->latitude_sec=((float)(temp*60))/10000000;

		if(gps_data->latitude_deg<33 || gps_data->latitude_deg>43){
			gps_data->sec_lat=0;
		}
		else{
			gps_data->sec_lat = (float)(gps_data->latitude_deg - LATITUDE_OFFSET) * 3600 + (float)gps_data->latitude_min * 60 + gps_data->latitude_sec;
		}
		gps_ptr+=36;
	}
}
