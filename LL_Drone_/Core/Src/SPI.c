/*

 * SPI.c
 *
 *  Created on: Jul 17, 2020
 *      Author: perSec
*/

#include "SPI.h"

void SPI_TransmitReceive(SPI_TypeDef *SPI,uint8_t* output,uint8_t* input, uint16_t size){
	LL_SPI_SetTransferDirection(SPI,LL_SPI_FULL_DUPLEX);

	if(!LL_SPI_IsEnabled(SPI)){
		LL_SPI_Enable(SPI);
	}

	for(uint16_t i=0;i<size;i++){
		while(!LL_SPI_IsActiveFlag_TXE(SPI));
		LL_SPI_TransmitData8(SPI,output[i]);

		while(!LL_SPI_IsActiveFlag_RXNE(SPI));
		input[i]=LL_SPI_ReceiveData8(SPI);
	}

	while(LL_SPI_IsActiveFlag_BSY(SPI));
}


void SPI_Transmit(SPI_TypeDef *SPI,uint8_t* output,uint16_t size){
	LL_SPI_SetTransferDirection(SPI,LL_SPI_FULL_DUPLEX);

	if(!LL_SPI_IsEnabled(SPI)){
		LL_SPI_Enable(SPI);
	}

	for(uint16_t i=0;i<size;i++){
		while(!LL_SPI_IsActiveFlag_TXE(SPI));
		LL_SPI_TransmitData8(SPI,output[i]);
	}
	while(LL_SPI_IsActiveFlag_BSY(SPI));

	if(LL_SPI_IsActiveFlag_RXNE(SPI)){
		LL_SPI_ReceiveData8(SPI);
	}
}



/*
void SPI_Receive(SPI_TypeDef *SPI, uint8_t* input, uint16_t size){

	if(!LL_SPI_IsEnabled(SPI)){
			LL_SPI_Enable(SPI);
	}
	LL_SPI_SetTransferDirection(SPI,LL_SPI_SIMPLEX_RX);
	for(uint16_t i=0;i<size;i++){
		while(!LL_SPI_IsActiveFlag_RXNE(SPI));
		input[i]=LL_SPI_ReceiveData8(SPI);
	}
	LL_SPI_SetTransferDirection(SPI,LL_SPI_FULL_DUPLEX);
}
*/



void SPI_Receive(SPI_TypeDef *SPI,uint8_t* input,uint16_t size){
	//LL_SPI_ReceiveData8(SPI);
	SPI_TransmitReceive(SPI,input,input,size);
}

