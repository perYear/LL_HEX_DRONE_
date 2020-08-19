/*

 * SPI.h
 *
 *  Created on: Jul 17, 2020
 *      Author: perSec
*/

#include "stm32f4xx_ll_spi.h"

#include "stm32f4xx_ll_dma.h"

#ifndef INC_SPI_H_
#define INC_SPI_H_

void SPI_TransmitReceive(SPI_TypeDef *SPI,uint8_t* output,uint8_t* input, uint16_t size);

void SPI_Transmit(SPI_TypeDef *SPI,uint8_t* output,uint16_t size);

void SPI_Receive(SPI_TypeDef *SPI,uint8_t* input,uint16_t size);

void SPI_DMA_Transmit(SPI_TypeDef *SPI, DMA_TypeDef * DMA,uint32_t STREAM, uint8_t* data, uint16_t size);

void SPI_DMA_Receive(SPI_TypeDef *SPI, DMA_TypeDef * DMA,uint32_t STREAM, uint8_t* data, uint16_t size);

void SPI_DMA_TransmitReceive(SPI_TypeDef *SPI,DMA_TypeDef * RX_DMA, DMA_TypeDef * TX_DMA,uint32_t TX_STREAM,uint32_t RX_STREAM,uint8_t* tx_data, uint8_t* rx_data, uint16_t size);

#endif /* INC_SPI_H_*/

