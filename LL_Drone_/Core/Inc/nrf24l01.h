/*
 * nrf24l01.h
 *
 *  Created on: Jul 18, 2020
 *      Author: perSec
 */
#ifndef INC_SPI_H_
#include "SPI.h"
#endif

#ifndef __MAIN_H
#include "main.h"
#endif

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_


void nrf_Read(SPI_TypeDef *SPI,uint8_t address, uint8_t *databuf);

void nrf_Write(SPI_TypeDef *SPI,uint8_t address, uint8_t *data);


void dump_reg(SPI_TypeDef *SPI);

#endif /* INC_NRF24L01_H_ */
