/**
 ******************************************************************************
 * @file	rf_spi1.h
 * @author	Hampus Sandberg
 * @version	0.1
 * @date	2013-07-15
 * @brief	Manage SPI1
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RF_SPI1_H_
#define RF_SPI1_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Defines -------------------------------------------------------------------*/
/* Typedefs ------------------------------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/
void RF_SPI1_Init();
uint8_t RF_SPI1_WriteRead(uint8_t Data);
void RF_SPI1_Write(uint8_t Data);

#endif /* RF_SPI1_H_ */
