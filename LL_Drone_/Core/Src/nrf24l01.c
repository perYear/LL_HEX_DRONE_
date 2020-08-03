/*

 * nrf24l01.c
 *
 *  Created on: Jul 18, 2020
 *      Author: perSec


#include "nrf24l01.h"

void nrf_Read(SPI_TypeDef *SPI,uint8_t address, uint8_t *databuf){
	uint8_t spiTXbuf[6];
	uint8_t spiRXbuf[6];

	spiTXbuf[0]=address;


	if(address==0x0a || address==0x0b||address==0x10){
		LL_GPIO_ResetOutputPin(NRF_CS_GPIO_Port,NRF_CS_Pin);
		//FULL
		//SPI_TransmitReceive(SPI,spiTXbuf,spiRXbuf,6);

		//TXRX
		SPI_Transmit(SPI,spiTXbuf,1);
		SPI_Receive(SPI,spiRXbuf,5);

		LL_GPIO_SetOutputPin(NRF_CS_GPIO_Port,NRF_CS_Pin);
		//FULL
		for(int i=1;i<6;i++){
			databuf[i-1]=spiRXbuf[i];
		}

		//TXRX
		for(int i=0;i<5;i++){
			databuf[i]=spiRXbuf[i];
		}
	}
	else{
		LL_GPIO_ResetOutputPin(NRF_CS_GPIO_Port,NRF_CS_Pin);
		//FULL
		//SPI_TransmitReceive(SPI,spiTXbuf,spiRXbuf,2);

		//TXRX
		SPI_Transmit(SPI,spiTXbuf,1);
		SPI_Receive(SPI,spiRXbuf,1);

		LL_GPIO_SetOutputPin(NRF_CS_GPIO_Port,NRF_CS_Pin);
		//FULL
		//databuf[0]=spiRXbuf[1];

		//TXRX
		databuf[0]=spiRXbuf[0];
	}
}

void nrf_Write(SPI_TypeDef *SPI,uint8_t address, uint8_t *data){
	uint8_t spiTXbuf[6];

	spiTXbuf[0]=address|0x20;
	if(address==0x0a || address==0x0b||address==0x10){
		for(int i=1;i<6;i++){
			spiTXbuf[i]=data[i-1];
		}
		LL_GPIO_ResetOutputPin(NRF_CS_GPIO_Port,NRF_CS_Pin);
		SPI_Transmit(SPI,spiTXbuf,6);
		LL_GPIO_SetOutputPin(NRF_CS_GPIO_Port,NRF_CS_Pin);
	}
	else if(data==0x00){
		LL_GPIO_ResetOutputPin(NRF_CS_GPIO_Port,NRF_CS_Pin);
		SPI_Transmit(SPI,spiTXbuf,1);
		LL_GPIO_SetOutputPin(NRF_CS_GPIO_Port,NRF_CS_Pin);
	}
	else{
		spiTXbuf[1]=data[0];
		LL_GPIO_ResetOutputPin(NRF_CS_GPIO_Port,NRF_CS_Pin);
		SPI_Transmit(SPI,spiTXbuf,2);
		LL_GPIO_SetOutputPin(NRF_CS_GPIO_Port,NRF_CS_Pin);
	}
}


void dump_reg(SPI_TypeDef *SPI){
	uint8_t spiTXbuf[6];
	uint8_t spiRXbuf[6];

	spiTXbuf[0]=0x00;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x01;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x02;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x03;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x04;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x05;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x06;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x07;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x08;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x09;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x0a;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X%02X%02X%02X%02X\r\n",spiTXbuf[0],spiRXbuf[0],spiRXbuf[1],spiRXbuf[2],spiRXbuf[3],spiRXbuf[4]);

	spiTXbuf[0]=0x0b;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X%02X%02X%02X%02X\r\n",spiTXbuf[0],spiRXbuf[0],spiRXbuf[1],spiRXbuf[2],spiRXbuf[3],spiRXbuf[4]);

	spiTXbuf[0]=0x0c;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x0d;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x0e;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x0f;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x10;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X%02X%02X%02X%02X \r\n",spiTXbuf[0],spiRXbuf[0],spiRXbuf[1],spiRXbuf[2],spiRXbuf[3],spiRXbuf[4]);

	spiTXbuf[0]=0x11;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x12;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x13;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x14;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x15;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x16;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x17;
	nrf_Read(SPI,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

}
*/
