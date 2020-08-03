/*
 * I2C.h
 *
 *  Created on: Jul 18, 2020
 *      Author: perSec
 */

#ifndef __STM32F4xx_LL_I2C_H
#include "stm32f4xx_ll_i2c.h"
#endif

#ifndef __STM32F4xx_LL_DMA_H
#include "stm32f4xx_ll_dma.h"
#endif


#ifndef INC_I2C_H_
#define INC_I2C_H_

typedef struct{
	I2C_TypeDef * I2C;
	uint8_t i2c_busy;
	uint8_t i2c_IsRx;
}I2C_struct;


typedef struct{
	DMA_TypeDef * DMA;
	uint8_t dev_address;
	uint32_t Stream;
	uint16_t length;
	uint8_t i2c_receive_dma;
}I2C_DMA_struct;

void I2C_Transmit(I2C_struct* I2C, uint8_t address, uint8_t* data,uint16_t size);

void I2C_Receive(I2C_struct* I2C, uint8_t address, uint8_t* outputdata,uint16_t size);


void I2C_Receive_DMA(I2C_struct * I2C,I2C_DMA_struct* DMA, uint8_t address);


void I2C_init(I2C_struct* I2C,I2C_TypeDef* I2C_type);

void I2C_Receive_DMA_init(I2C_struct * I2C,
							I2C_DMA_struct * DMA,
							I2C_TypeDef * I2C_type,
							DMA_TypeDef * DMA_type,
							uint32_t Stream,
							uint32_t mem_address,
							uint32_t periph_address,
							uint32_t size);


void I2C_DMA_irq(I2C_struct * I2C, I2C_DMA_struct * DMA);

void DMA_Stream_irq(I2C_struct * I2C, I2C_DMA_struct * DMA);

#endif /* INC_I2C_H_ */
