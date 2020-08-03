/*

 * SPI.h
 *
 *  Created on: Jul 17, 2020
 *      Author: perSec
*/
#ifndef STM32F4xx_LL_SPI_H
#include "stm32f4xx_ll_spi.h"
#endif

#ifndef INC_SPI_H_
#define INC_SPI_H_

void SPI_TransmitReceive(SPI_TypeDef *SPI,uint8_t* output,uint8_t* input, uint16_t size);

void SPI_Transmit(SPI_TypeDef *SPI,uint8_t* output,uint16_t size);

void SPI_Receive(SPI_TypeDef *SPI,uint8_t* input,uint16_t size);

#endif /* INC_SPI_H_*/

