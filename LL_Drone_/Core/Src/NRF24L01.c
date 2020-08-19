/*
 * NRF240L1.c
 *
 *  Created on: Jun 28, 2020
 *      Author: perSec
 */

#include "NRF24L01.h"

void nrf_Write(NRF24L01* nrf,uint8_t address, uint8_t *data){
	uint8_t spiTXbuf[6];

	spiTXbuf[0]=address|0x20;
	if(address==0x0a || address==0x0b||address==0x10){
		for(int i=1;i<6;i++){
			spiTXbuf[i]=data[i-1];
		}
		LL_GPIO_ResetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);
		SPI_Transmit(nrf->SPI,spiTXbuf,6);
		LL_GPIO_SetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);
	}
	else if(data==0x00){
		LL_GPIO_ResetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);
		SPI_Transmit(nrf->SPI,spiTXbuf,1);
		LL_GPIO_SetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);
	}
	else{
		spiTXbuf[1]=data[0];
		LL_GPIO_ResetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);
		SPI_Transmit(nrf->SPI,spiTXbuf,2);
		LL_GPIO_SetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);
	}
}

void nrf_Read(NRF24L01* nrf,uint8_t address, uint8_t *databuf){
	uint8_t spiTXbuf[6];
	uint8_t spiRXbuf[6];

	spiTXbuf[0]=address;


	if(address==0x0a || address==0x0b||address==0x10){
		LL_GPIO_ResetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);
		SPI_TransmitReceive(nrf->SPI,spiTXbuf,spiRXbuf,6);
		LL_GPIO_SetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);
		for(int i=1;i<6;i++){
			databuf[i-1]=spiRXbuf[i];
		}
	}
	else{
		LL_GPIO_ResetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);
		SPI_TransmitReceive(nrf->SPI,spiTXbuf,spiRXbuf,2);
		LL_GPIO_SetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);
		databuf[0]=spiRXbuf[1];
	}
}

void nrf_enable_pulse(NRF24L01* nrf, TIM_TypeDef* tim,uint16_t usec){
	tim->ARR=(uint32_t)usec;
	LL_TIM_EnableCounter(tim);
	LL_GPIO_SetOutputPin(nrf->chip_enable_port,nrf->chip_enable_pin);
	while(tim->CNT!=0){};
	LL_GPIO_ResetOutputPin(nrf->chip_enable_port,nrf->chip_enable_pin);
	LL_TIM_DisableCounter(tim);
}

void nrf_power_down(NRF24L01* nrf){
	uint8_t data=0;
	nrf_Read(nrf,0x00,&data);
	data &= ~(0x01<<1);
	nrf_Write(nrf,0x00,&data);
}

void nrf_power_up(NRF24L01* nrf){
	uint8_t data=0;
	nrf_Read(nrf,0x00,&data);
	data|= (0x01<<1);
	nrf_Write(nrf,0x00,&data);
}


void nrf_init(NRF24L01* nrf,SPI_TypeDef* spi,type mode,GPIO_TypeDef* chip_select_port,uint16_t chip_select_pin ,GPIO_TypeDef* chip_enable_port,uint16_t chip_enable_pin,uint16_t length){
	nrf->SPI=spi;
	nrf->para.mode=mode;
	nrf->chip_select_port=chip_select_port;
	nrf->chip_select_pin=chip_select_pin;
	nrf->chip_enable_port=chip_enable_port;
	nrf->chip_enable_pin=chip_enable_pin;
	//HAL_GPIO_WritePin(nrf->chip_select_port,nrf->chip_select_pin,1);  //??
	LL_mDelay(100);
	LL_GPIO_SetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);

	nrf_power_down(nrf);
	if(mode==TX){
		LL_GPIO_ResetOutputPin(nrf->chip_enable_port,nrf->chip_enable_pin);
		nrf_set_TX(nrf);
	}
	else if(mode==RX){
		LL_GPIO_SetOutputPin(nrf->chip_enable_port,nrf->chip_enable_pin);
		nrf_set_RX(nrf);
	}

	nrf_crc_enable(nrf);
	nrf_crc_encoding(nrf,1);
	nrf_enable_pipe(nrf,0);
	nrf_disable_pipe(nrf,1);
	nrf_data_length(nrf,0,32);
	nrf_enable_dpl(nrf);
	nrf_enable_dpl_pipe(nrf,0);

	nrf_data_length(nrf,0,length);		//data pipe 0
	nrf_clear_interrupt(nrf);
	nrf_power_up(nrf);

	flush_TX(nrf);
	flush_RX(nrf);

	LL_mDelay(10);


}

void flush_TX(NRF24L01* nrf){
	nrf_Write(nrf,0xE1,0x00);
}

void flush_RX(NRF24L01* nrf){
	nrf_Write(nrf,0xE2,0x00);
}

void nrf_TX_address(NRF24L01* nrf,uint8_t* address){
	nrf_Write(nrf,0x10,address);
}

void nrf_RX_address(NRF24L01* nrf,uint8_t pipe, uint8_t* address){
	nrf_Write(nrf,0x0A+pipe,address);
}

void nrf_TX_payload(NRF24L01* nrf, uint8_t* data){
	uint8_t address;
	uint8_t status;

	address=0xA0;


	/*nrf_status(nrf,&status);
	if(status&(0x10)){
		status|= (0x01<<4);
	}
	if(status&(0x20)){
		status|=(0x01<<5);
	}
	nrf_Write(nrf,0x07,&status);*/

	flush_TX(nrf);
	LL_GPIO_ResetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);
	SPI_Transmit(nrf->SPI,&address,1);
	SPI_Transmit(nrf->SPI,data,nrf->para.length);
	LL_GPIO_SetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);

	nrf_status(nrf,&status);
	if(status&(0x10)){
		status|= (0x01<<4);
	}
	if(status&(0x20)){
		status|=(0x01<<5);
	}
	nrf_Write(nrf,0x07,&status);

}

void nrf_TX_payload_noack(NRF24L01* nrf, uint8_t* data){
	uint8_t spiTXbuf[17];

	spiTXbuf[0]=0b10110000;
	for(uint8_t i=0;i<16;i++){
		spiTXbuf[i+1]=data[i];
	}
	LL_GPIO_ResetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);
	SPI_Transmit(nrf->SPI,spiTXbuf,33);
	LL_GPIO_SetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);

}

void nrf_RX_payload(NRF24L01* nrf, uint8_t* output){
	uint8_t address;
	uint8_t status;
	address=0x61;

	LL_GPIO_ResetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);
	SPI_Transmit(nrf->SPI,&address,1);
	SPI_Receive(nrf->SPI,output,nrf->para.length);
	LL_GPIO_SetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);
	flush_RX(nrf);

	nrf_status(nrf,&status);
	if(status&(0x40)){
		status|= (0x01<<6);
	}
	nrf_Write(nrf,0x07,&status);

}

void nrf_RX_DR_enable(NRF24L01* nrf){
	uint8_t reg;
	nrf_Read(nrf,0x00,&reg);
	reg&=~(0x01<<6);
	nrf_Write(nrf,0x00,&reg);
}
void nrf_RX_DR_disable(NRF24L01* nrf){
	uint8_t reg;
	nrf_Read(nrf,0x00,&reg);
	reg|= (0x01<<6);
	nrf_Write(nrf,0x00,&reg);
}

void nrf_TX_DS_enable(NRF24L01* nrf){
	uint8_t reg;
	nrf_Read(nrf,0x00,&reg);
	reg&=~(0x01<<5);
	nrf_Write(nrf,0x00,&reg);
}
void nrf_TX_DS_disable(NRF24L01* nrf){
	uint8_t reg;
	nrf_Read(nrf,0x00,&reg);
	reg|= (0x01<<5);
	nrf_Write(nrf,0x00,&reg);
}

void nrf_MAX_RT_enable(NRF24L01* nrf){
	uint8_t reg;
	nrf_Read(nrf,0x00,&reg);
	reg&=~(0x01<<4);
	nrf_Write(nrf,0x00,&reg);
}
void nrf_MAX_RT_disable(NRF24L01* nrf){
	uint8_t reg;
	nrf_Read(nrf,0x00,&reg);
	reg|= (0x01<<4);
	nrf_Write(nrf,0x00,&reg);
}

void nrf_clear_interrupt(NRF24L01* nrf){
	uint8_t status;
	nrf_status(nrf,&status);
	status|=(0x07<<4);
	nrf_Write(nrf,0x07,&status);
}

void nrf_set_TX(NRF24L01* nrf){
	uint8_t reg=0;
	nrf_RX_DR_disable(nrf);
	nrf_MAX_RT_disable(nrf);
	nrf_TX_DS_enable(nrf);
	nrf_Read(nrf,0x00,&reg);
	reg &= (~0x01);
	nrf_Write(nrf,0x00,&reg);
}

void nrf_set_RX(NRF24L01* nrf){
	uint8_t reg=0;
	nrf_RX_DR_enable(nrf);
	nrf_MAX_RT_enable(nrf);
	nrf_TX_DS_disable(nrf);
	nrf_Read(nrf,0x00,&reg);
	reg|= 0x01;
	nrf_Write(nrf,0x00,&reg);
}

void nrf_crc_enable(NRF24L01* nrf){
	uint8_t reg=0;
	nrf_Read(nrf,0x00,&reg);
	reg|= 0x01<<3;
	nrf_Write(nrf,0x00,&reg);
}

void nrf_crc_disable(NRF24L01* nrf){
	uint8_t reg=0;
	nrf_Read(nrf,0x00,&reg);
	reg&=~(0x01<<3);
	nrf_Write(nrf,0x00,&reg);
}

void nrf_crc_encoding(NRF24L01* nrf,uint8_t byte){
	uint8_t reg=0;
	nrf_Read(nrf,0x00,&reg);
	if(byte==1)
		reg&=~(0x01<<2);
	else if(byte==2)
		reg|=(0x01<<2);
	nrf_Write(nrf,0x00,&reg);
}
void nrf_enable_pipe(NRF24L01* nrf, uint8_t pipe){
	uint8_t reg=0;
	nrf_Read(nrf,0x02,&reg);
	reg|= 0x01<<pipe;
	nrf_Write(nrf,0x02,&reg);
}

void nrf_disable_pipe(NRF24L01* nrf, uint8_t pipe){
	uint8_t reg=0;
	nrf_Read(nrf,0x02,&reg);
	reg &= ~(0x01<<pipe);
	nrf_Write(nrf,0x02,&reg);
}

void nrf_data_length(NRF24L01*nrf,uint8_t pipe, uint16_t length){
	uint8_t reg[2];
	reg[0]=0x11+pipe;
	reg[1]=length;
	nrf->para.length=length;
	nrf_Write(nrf,reg[0],&reg[1]);
}

void nrf_rfch(NRF24L01*nrf, uint8_t rf){
	uint8_t rfch=rf;
	nrf_Write(nrf,0x05,&rfch);
}



void nrf_fifo_status(NRF24L01* nrf,uint8_t* output){
	nrf_Read(nrf,0x17,output);
}

void nrf_status(NRF24L01* nrf, uint8_t * output){
	uint8_t address=0x07;
	LL_GPIO_ResetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);
	SPI_TransmitReceive(nrf->SPI,&address,output,1);
	LL_GPIO_SetOutputPin(nrf->chip_select_port,nrf->chip_select_pin);
}

void nrf_observe_TX(NRF24L01* nrf, uint8_t * output){
	nrf_Read(nrf,0x08,output);
}

void nrf_enable_dpl(NRF24L01* nrf){
	uint8_t reg;
	nrf_Read(nrf,0x1D,&reg);
	reg|= 0x01<<2;
	nrf_Write(nrf,0x1D,&reg);

}

void nrf_enable_dpl_pipe(NRF24L01* nrf,uint8_t pipe){
	uint8_t reg;
	nrf_Read(nrf,0x1C,&reg);
	reg|=0x01<<pipe;
	nrf_Write(nrf,0x1C,&reg);
}

void dump_reg(NRF24L01* nrf){
	uint8_t spiTXbuf[6];
	uint8_t spiRXbuf[6];

	spiTXbuf[0]=0x00;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);


	spiTXbuf[0]=0x01;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);


	spiTXbuf[0]=0x02;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x03;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);


	spiTXbuf[0]=0x04;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);


	spiTXbuf[0]=0x05;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);


	spiTXbuf[0]=0x06;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x07;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x08;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x09;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x0a;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X%02X%02X%02X%02X\r\n",spiTXbuf[0],spiRXbuf[0],spiRXbuf[1],spiRXbuf[2],spiRXbuf[3],spiRXbuf[4]);


	spiTXbuf[0]=0x0b;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X%02X%02X%02X%02X\r\n",spiTXbuf[0],spiRXbuf[0],spiRXbuf[1],spiRXbuf[2],spiRXbuf[3],spiRXbuf[4]);


	spiTXbuf[0]=0x0c;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

	spiTXbuf[0]=0x0d;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);


	spiTXbuf[0]=0x0e;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);


	spiTXbuf[0]=0x0f;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);


	spiTXbuf[0]=0x10;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X%02X%02X%02X%02X \r\n",spiTXbuf[0],spiRXbuf[0],spiRXbuf[1],spiRXbuf[2],spiRXbuf[3],spiRXbuf[4]);


	spiTXbuf[0]=0x11;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);


	spiTXbuf[0]=0x12;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);


	spiTXbuf[0]=0x13;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);


	spiTXbuf[0]=0x14;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);


	spiTXbuf[0]=0x15;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);


	spiTXbuf[0]=0x16;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);


	spiTXbuf[0]=0x17;
	nrf_Read(nrf,spiTXbuf[0],spiRXbuf);
	printf("%02X: %02X\r\n",spiTXbuf[0],spiRXbuf[0]);

}
