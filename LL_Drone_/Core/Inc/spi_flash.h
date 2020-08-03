/*
 * spi_flash.h
 *
 *  Created on: 2020. 1. 23.
 *      Author: perSec
 */
#ifndef __MAIN_H
#include"main.h"
#endif

#ifndef SPI_FLASH_H_
#define SPI_FLASH_H_

#include "spi.h"

#define FLASH_CS_Pin LL_GPIO_PIN_15
#define FLASH_CS_GPIO_Port GPIOA

uint8_t flash_busy(SPI_TypeDef *SPI);

void flash_write_enable(SPI_TypeDef *SPI);

void flash_write_disable(SPI_TypeDef *SPI);

void flash_read_status(SPI_TypeDef *SPI,uint8_t num,uint8_t* out_buf);

void flash_read_jedec_id(SPI_TypeDef *SPI,uint8_t* out_buf);

void flash_read_unique_id(SPI_TypeDef *SPI,int8_t* out_buf);

void flash_write(SPI_TypeDef *SPI,uint32_t address ,uint8_t* data_buf, uint16_t size);
void flash_read(SPI_TypeDef *SPI,uint32_t address, uint8_t* out_buf, uint16_t size);
void flash_sector_erase(SPI_TypeDef *SPI,uint32_t address);
void flash_erase_chip(SPI_TypeDef *SPI);
#endif /* SPI_FLASH_H_ */
