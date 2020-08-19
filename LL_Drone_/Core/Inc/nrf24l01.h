/*
 * NRF240L1.h
 *
 *  Created on: Jun 28, 2020
 *      Author: perSec
 */


#include "main.h"

#include "SPI.h"


#ifndef INC_NRF240L1_H_
#define INC_NRF240L1_H_


#define TXDATA_LENGTH 32

typedef enum _type{
	RX=0,
	TX=1
}type;

typedef struct{
	type mode;
	uint16_t length;
}NRF24L01_para;

typedef struct{
	SPI_TypeDef* SPI;
	NRF24L01_para para;
	GPIO_TypeDef* chip_select_port;
	uint16_t chip_select_pin;
	GPIO_TypeDef* chip_enable_port;
	uint16_t chip_enable_pin;
}NRF24L01;

typedef struct __attribute__((packed)){
	uint8_t nrf_address;
	//uint32_t nrf_tot_counter;
	float pitch;
	float roll;
	//float yaw;
	int32_t motor[6];
}NRF24L01_TX_DATA1;

NRF24L01 nrf_tx;
NRF24L01_TX_DATA1 txdata;



void nrf_Write(NRF24L01* nrf,uint8_t address, uint8_t *data);

void nrf_Read(NRF24L01* nrf,uint8_t address, uint8_t *databuf);
void nrf_enable_pulse(NRF24L01* nrf, TIM_TypeDef* tim,uint16_t usec);

void nrf_init(NRF24L01* nrf,SPI_TypeDef* spi,type mode,GPIO_TypeDef* chip_select_port,uint16_t chip_select_pin ,GPIO_TypeDef* chip_enable_port,uint16_t chip_enable_pin,uint16_t length);

void flush_TX(NRF24L01* nrf);

void flush_RX(NRF24L01* nrf);

void nrf_TX_address(NRF24L01* nrf,uint8_t* address);

void nrf_RX_address(NRF24L01* nrf,uint8_t pipe, uint8_t* address);

void nrf_TX_payload(NRF24L01* nrf, uint8_t* data);

void nrf_RX_payload(NRF24L01* nrf, uint8_t* output);

void nrf_clear_interrupt(NRF24L01* nrf);

void nrf_power_up(NRF24L01* nrf);

void nrf_set_TX(NRF24L01* nrf);
void nrf_set_RX(NRF24L01* nrf);

void nrf_crc_enable(NRF24L01* nrf);
void nrf_crc_disable(NRF24L01* nrf);
void nrf_crc_encoding(NRF24L01* nrf,uint8_t byte);

void nrf_enable_pipe(NRF24L01* nrf, uint8_t pipe);

void nrf_disable_pipe(NRF24L01* nrf, uint8_t pipe);
void nrf_data_length(NRF24L01*nrf,uint8_t pipe, uint16_t length);

void nrf_fifo_status(NRF24L01* nrf,uint8_t* output);
void nrf_status(NRF24L01* nrf, uint8_t * output);
void nrf_observe_TX(NRF24L01* nrf, uint8_t * output);

void nrf_enable_dpl_pipe(NRF24L01* nrf,uint8_t pipe);
void nrf_enable_dpl(NRF24L01* nrf);

void dump_reg(NRF24L01* nrf);
#endif /* INC_NRF240L1_H_ */
