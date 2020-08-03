/*
 * spi_flash.c
 *
 *  Created on: 2020. 1. 23.
 *      Author: perSec
 */

#include "spi_flash.h"

uint8_t flash_busy(SPI_TypeDef *SPI){
	uint8_t status_reg;
	flash_read_status(SPI,1,&status_reg);
	return status_reg & 0x01;
}

void flash_write_enable(SPI_TypeDef *SPI){
	uint8_t buf;
	buf=0x06;
	LL_GPIO_ResetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);
	SPI_Transmit(SPI,&buf,1);
	LL_GPIO_SetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);
}

void flash_write_disable(SPI_TypeDef *SPI){
	uint8_t buf;
	buf=0x04;
	LL_GPIO_ResetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);
	SPI_Transmit(SPI,&buf,1);
	LL_GPIO_SetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);
}

void flash_read_status(SPI_TypeDef *SPI,uint8_t num,uint8_t* out_buf){
	uint8_t mosi_buf[2];
	uint8_t miso_buf[2];
	if(num==1)
		mosi_buf[0]=0x05;
	else if(num==2)
		mosi_buf[0]=0x35;
	else
		return;

	LL_GPIO_ResetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);
	SPI_TransmitReceive(SPI,mosi_buf,miso_buf,2);
	LL_GPIO_SetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);

	out_buf[0]=miso_buf[1];
	//outbuf[1]=miso_buf[2];
}

void flash_read_jedec_id(SPI_TypeDef *SPI,uint8_t* out_buf){
	uint8_t mosi_buf[4];
	uint8_t miso_buf[4];

	mosi_buf[0]=0x9f;

	LL_GPIO_ResetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);
	SPI_TransmitReceive(SPI,mosi_buf,miso_buf,4);
	LL_GPIO_SetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);

	for(int i=0;i<3;i++)
		out_buf[i]=miso_buf[i+1];
}

void flash_read_unique_id(SPI_TypeDef *SPI,int8_t* out_buf){
	uint8_t mosi_buf[5];
	uint8_t miso_buf[8];

	mosi_buf[0]=0x4b;

	LL_GPIO_ResetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);
	SPI_Transmit(SPI,mosi_buf,5);
	SPI_Receive(SPI,miso_buf,8);
	LL_GPIO_SetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);

	for(int i=0;i<8;i++)
		out_buf[i]=miso_buf[i];
}

void flash_write(SPI_TypeDef *SPI,uint32_t address ,uint8_t* data_buf, uint16_t size){
	uint8_t mosi_buf[4];
	mosi_buf[0]=0x02;
	while(flash_busy(SPI)==0x01);

	for(int i=0;i<3;i++){
		mosi_buf[3-i]=address&0x000000ff;
		address=address>>8;
	}

	flash_write_enable(SPI);
	LL_GPIO_ResetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);
	SPI_Transmit(SPI,mosi_buf,4);
	SPI_Transmit(SPI,data_buf,size);
	LL_GPIO_SetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);
	//flash_write_disable();
}
void flash_read(SPI_TypeDef *SPI,uint32_t address, uint8_t* out_buf, uint16_t size){
	uint8_t mosi_buf[4];
	mosi_buf[0]=0x03;

	while(flash_busy(SPI)==0x01);

	for(int i=0;i<3;i++){
		mosi_buf[3-i]=address&0x000000ff;
		address=address>>8;
	}

	LL_GPIO_ResetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);
	SPI_Transmit(SPI,mosi_buf,4);
	SPI_Receive(SPI,out_buf,size);
	LL_GPIO_SetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);
}
void flash_sector_erase(SPI_TypeDef *SPI,uint32_t address){
	uint8_t mosi_buf[4];
	mosi_buf[0]=0x20;
	for(int i=0;i<3;i++){
		mosi_buf[3-i]=address&0x000000ff;
		address=address>>8;
	}
	flash_write_enable(SPI);
	LL_GPIO_ResetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);
	SPI_Transmit(SPI,mosi_buf,4);
	LL_GPIO_SetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);
}
void flash_erase_chip(SPI_TypeDef *SPI)
{
    uint8_t Write_Enable = 0x06;
    uint8_t Erase_Chip = 0xC7;

    LL_GPIO_ResetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);     // CS to low
    SPI_Transmit(SPI,&Write_Enable,1); // Write Enable Command
    LL_GPIO_SetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);       // CS to high

    LL_GPIO_ResetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);     // CS to low
    SPI_Transmit(SPI,&Erase_Chip,1);   // Erase Chip Command
    LL_GPIO_SetOutputPin(FLASH_CS_GPIO_Port,FLASH_CS_Pin);       // CS to high
}


