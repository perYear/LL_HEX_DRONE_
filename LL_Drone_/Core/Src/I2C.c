/*
 * I2C.c
 *
 *  Created on: Jul 18, 2020
 *      Author: perSec
 */

#include "I2C.h"

void I2C_init(I2C_struct* I2C,I2C_TypeDef* I2C_type){
	I2C->I2C=I2C_type;
	I2C->i2c_busy=0;
	I2C->i2c_IsRx=0;
}


void I2C_Transmit(I2C_struct* I2C, uint8_t address, uint8_t* data,uint16_t size){
	if(I2C->i2c_busy==1){
		return;
	}
	I2C->i2c_busy=1;
	I2C->i2c_IsRx=0;
	LL_I2C_DisableDMAReq_RX(I2C->I2C);
	LL_I2C_DisableIT_TX(I2C->I2C);


	if(!LL_I2C_IsEnabled(I2C->I2C)){
		LL_I2C_Enable(I2C->I2C);
	}
	LL_I2C_DisableBitPOS(I2C->I2C);

	while(LL_I2C_IsActiveFlag_BUSY(I2C->I2C));

	LL_I2C_GenerateStartCondition(I2C->I2C);


	while( !LL_I2C_IsActiveFlag_SB(I2C->I2C) );
	LL_I2C_TransmitData8(I2C->I2C,address);



	while( !LL_I2C_IsActiveFlag_ADDR(I2C->I2C));
	LL_I2C_ClearFlag_ADDR(I2C->I2C);

	for(uint16_t i=0;i<size;i++){
		LL_I2C_TransmitData8(I2C->I2C,data[i]);
		while(!LL_I2C_IsActiveFlag_TXE(I2C->I2C));
	}
	while(!LL_I2C_IsActiveFlag_BTF(I2C->I2C));

	LL_I2C_GenerateStopCondition(I2C->I2C);
	I2C->i2c_busy=0;


}


void I2C_Receive(I2C_struct* I2C, uint8_t address, uint8_t* outputdata,uint16_t size){
	if(I2C->i2c_busy==1){
		return;
	}
	I2C->i2c_busy=1;
	I2C->i2c_IsRx=1;

	address |=(0x01);
	LL_I2C_DisableDMAReq_RX(I2C->I2C);
	LL_I2C_DisableIT_RX(I2C->I2C);

	if(!LL_I2C_IsEnabled(I2C->I2C)){
		LL_I2C_Enable(I2C->I2C);
	}
	LL_I2C_DisableBitPOS(I2C->I2C);


	while(LL_I2C_IsActiveFlag_BUSY(I2C->I2C));

	if(size==1)
		LL_I2C_AcknowledgeNextData(I2C->I2C,LL_I2C_NACK);
	else
		LL_I2C_AcknowledgeNextData(I2C->I2C,LL_I2C_ACK);



	LL_I2C_GenerateStartCondition(I2C->I2C);


	while( !LL_I2C_IsActiveFlag_SB(I2C->I2C) );
	LL_I2C_TransmitData8(I2C->I2C,address);

	while( !LL_I2C_IsActiveFlag_ADDR(I2C->I2C));
	LL_I2C_ClearFlag_ADDR(I2C->I2C);


	for(uint16_t i=0;i<size;i++){
		if(i==(size-1))
			LL_I2C_AcknowledgeNextData(I2C->I2C,LL_I2C_NACK);
		while(!LL_I2C_IsActiveFlag_RXNE(I2C->I2C));
		outputdata[i]=LL_I2C_ReceiveData8(I2C->I2C);
	}

	LL_I2C_GenerateStopCondition(I2C->I2C);

	I2C->i2c_busy=0;

}


void I2C_Receive_DMA(I2C_struct * I2C,I2C_DMA_struct* DMA, uint8_t address,uint32_t mem_address, uint32_t periph_address, uint32_t size){
	if(I2C->i2c_busy==1){
		return;
	}
	I2C->i2c_busy=1;
	I2C->i2c_IsRx=1;
	DMA->i2c_receive_dma=0;
	DMA->length=size;

	address |=(0x01);
	DMA->dev_address=address;

	LL_I2C_DisableIT_RX(I2C->I2C);
	LL_DMA_DisableStream(DMA->DMA,DMA->Stream);

	LL_DMA_SetMemoryAddress(DMA->DMA,DMA->Stream,mem_address);
	LL_DMA_SetPeriphAddress(DMA->DMA,DMA->Stream,periph_address);
	LL_DMA_EnableIT_TC(DMA->DMA,DMA->Stream);
	LL_DMA_SetDataLength(DMA->DMA,DMA->Stream, size);
	DMA->DMA->LIFCR=0x003D0000; //STREAM2 CLEAR ALL FLAG

	LL_DMA_EnableStream(DMA->DMA,DMA->Stream);
	LL_I2C_EnableIT_EVT(I2C->I2C);
	LL_I2C_EnableDMAReq_RX(I2C->I2C);

	if(!LL_I2C_IsEnabled(I2C->I2C)){
		LL_I2C_Enable(I2C->I2C);
	}
	LL_I2C_DisableBitPOS(I2C->I2C);

	while(LL_I2C_IsActiveFlag_BUSY(I2C->I2C));

	LL_I2C_AcknowledgeNextData(I2C->I2C,LL_I2C_ACK);

	LL_I2C_GenerateStartCondition(I2C->I2C);
}

void I2C_Receive_DMA_init(I2C_struct * I2C, I2C_DMA_struct * DMA, I2C_TypeDef * I2C_type, DMA_TypeDef * DMA_type, uint32_t Stream){
	I2C_init(I2C,I2C_type);

	//DMA
	DMA->DMA=DMA_type;
	DMA->Stream=Stream;
	DMA->i2c_receive_dma=0;
}






void I2C_Receive_Circular_DMA(I2C_struct * I2C,I2C_DMA_struct* DMA, uint8_t address){
	if(I2C->i2c_busy==1){
		return;
	}
	I2C->i2c_busy=1;
	I2C->i2c_IsRx=1;
	DMA->i2c_receive_dma=0;

	address |=(0x01);
	DMA->dev_address=address;

	LL_I2C_DisableIT_RX(I2C->I2C);
	LL_I2C_EnableIT_EVT(I2C->I2C);
	LL_I2C_EnableDMAReq_RX(I2C->I2C);

	if(!LL_I2C_IsEnabled(I2C->I2C)){
		LL_I2C_Enable(I2C->I2C);
	}
	LL_I2C_DisableBitPOS(I2C->I2C);

	while(LL_I2C_IsActiveFlag_BUSY(I2C->I2C));

	LL_I2C_AcknowledgeNextData(I2C->I2C,LL_I2C_ACK);

	LL_I2C_GenerateStartCondition(I2C->I2C);

}


void I2C_Receive_Circular_DMA_init(I2C_struct * I2C,
							I2C_DMA_struct * DMA,
							I2C_TypeDef * I2C_type,
							DMA_TypeDef * DMA_type,
							uint32_t Stream,
							uint32_t mem_address,
							uint32_t periph_address,
							uint32_t size){

	//I2C
	I2C_init(I2C,I2C_type);

	//DMA
	DMA->DMA=DMA_type;
	DMA->Stream=Stream;
	DMA->length=size;
	DMA->i2c_receive_dma=0;

	LL_DMA_DisableStream(DMA_type,Stream);
	LL_DMA_SetMemoryAddress(DMA_type,Stream,mem_address);
	LL_DMA_SetPeriphAddress(DMA_type,Stream,periph_address);
	LL_DMA_EnableIT_TC(DMA_type,Stream);
	LL_DMA_SetDataLength(DMA_type,Stream, size);

	DMA->DMA->LIFCR=0x0F400000;	//STREAM3

	LL_DMA_EnableStream(DMA_type,Stream);
}

void I2C_DMA_irq(I2C_struct *I2C, I2C_DMA_struct * DMA){
	if(LL_I2C_IsActiveFlag_SB(I2C->I2C) && I2C->i2c_IsRx==1){
		LL_I2C_TransmitData8(I2C->I2C,DMA->dev_address);
	}
	else if(LL_I2C_IsActiveFlag_ADDR(I2C->I2C) && I2C->i2c_IsRx==1){
		if(DMA->length==1){
			LL_I2C_AcknowledgeNextData(I2C->I2C,LL_I2C_NACK);
		}
		else{
			LL_I2C_AcknowledgeNextData(I2C->I2C,LL_I2C_ACK);
		}
		LL_I2C_EnableLastDMA(I2C->I2C);
		LL_I2C_ClearFlag_ADDR(I2C->I2C);
	}
}



void DMA_Stream_irq(I2C_struct* I2C, I2C_DMA_struct * DMA){
	//i2c1
	if(I2C->I2C==I2C1){
		if(LL_DMA_IsActiveFlag_TC0(DMA->DMA)){
			LL_DMA_ClearFlag_TC0(DMA->DMA);
			LL_I2C_GenerateStopCondition(I2C->I2C);
			LL_I2C_DisableLastDMA(I2C->I2C);

			DMA->i2c_receive_dma=1;
			I2C->i2c_IsRx=0;
			I2C->i2c_busy=0;
		}
	}

	//i2c2
	if(I2C->I2C==I2C2){
		if(LL_DMA_IsActiveFlag_TC3(DMA->DMA)){
			LL_DMA_ClearFlag_TC3(DMA->DMA);
			LL_I2C_GenerateStopCondition(I2C->I2C);
			LL_I2C_DisableLastDMA(I2C->I2C);

			DMA->i2c_receive_dma=1;
			I2C->i2c_IsRx=0;
			I2C->i2c_busy=0;
		}
	}

	//i2c3
	if(I2C->I2C==I2C3){
		if(LL_DMA_IsActiveFlag_TC2(DMA->DMA)){
			LL_DMA_ClearFlag_TC2(DMA->DMA);
			LL_I2C_GenerateStopCondition(I2C->I2C);
			LL_I2C_DisableLastDMA(I2C->I2C);

			DMA->i2c_receive_dma=1;
			I2C->i2c_IsRx=0;
			I2C->i2c_busy=0;
		}
	}
}

