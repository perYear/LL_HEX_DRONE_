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


void SPI_DMA_TransmitReceive(SPI_TypeDef *SPI,DMA_TypeDef * RX_DMA, DMA_TypeDef * TX_DMA,uint32_t TX_STREAM,uint32_t RX_STREAM,uint8_t* tx_data, uint8_t* rx_data, uint16_t size){
	LL_SPI_SetTransferDirection(SPI,LL_SPI_FULL_DUPLEX);

	LL_SPI_DisableDMAReq_TX(SPI);
	LL_SPI_DisableDMAReq_RX(SPI);
	LL_DMA_DisableStream(RX_DMA,RX_STREAM);
	LL_DMA_DisableStream(TX_DMA,TX_STREAM);

	//clear all flag
	DMA2->LIFCR=0x0F400000;
	//clear all flag
	DMA2->LIFCR=0x0000003D;

	LL_DMA_SetDataLength(RX_DMA,RX_STREAM,(uint32_t)size);
	LL_DMA_SetMemoryAddress(RX_DMA,RX_STREAM,(uint32_t)rx_data);
	LL_DMA_SetPeriphAddress(RX_DMA,RX_STREAM,LL_SPI_DMA_GetRegAddr(SPI));
	LL_DMA_EnableIT_TC(RX_DMA,RX_STREAM);

	LL_DMA_SetDataLength(TX_DMA,TX_STREAM,(uint32_t)size);
	LL_DMA_SetMemoryAddress(TX_DMA,TX_STREAM,(uint32_t)tx_data);
	LL_DMA_SetPeriphAddress(TX_DMA,TX_STREAM,LL_SPI_DMA_GetRegAddr(SPI));
	//LL_DMA_EnableIT_TC(TX_DMA,TX_STREAM);

	LL_DMA_EnableStream(RX_DMA,RX_STREAM);
	LL_DMA_EnableStream(TX_DMA,TX_STREAM);

	LL_SPI_EnableDMAReq_RX(SPI);
	LL_SPI_EnableDMAReq_TX(SPI);

}
