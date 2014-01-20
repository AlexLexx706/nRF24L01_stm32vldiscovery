/**
 ******************************************************************************
 * @file	rf_spi1.c
 * @author	Hampus Sandberg
 * @version	0.1
 * @date	2013-07-15
 * @brief
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "rf_spi1.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"

/* Private defines -----------------------------------------------------------*/
#define SPIx		SPI1

/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Functions -----------------------------------------------------------------*/
/**
 * @brief	Initializes the SPI
 * @param	None
 * @retval	None
 */
void RF_SPI1_Init()
{
	/* Enable GPIOx clock */
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOB | RCC_APB2ENR_AFIOEN, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);


	//SPI1_SCK/PB3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	//SPI1_MISO/PB4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	//SPI1_MOSI/PB5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//«аполн€ем структуру с параметрами SPI1 модул€
	SPI_StructInit(&SPI_InitStructure);

	/* Initialize SPIx */
	SPI_InitStructure.SPI_Direction 		= SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode 				= SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize 			= SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL 				= SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA 				= SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS 				= SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit 			= SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial 	= 7;
	SPI_Init(SPIx, &SPI_InitStructure);

	/* Enable SPIx */
	SPI_Cmd(SPIx, ENABLE);
}

/**
 * @brief	Writes and receives data from the SPI
 * @param	Data: data to be written to the SPI
 * @retval	The received data
 */
uint8_t RF_SPI1_WriteRead(uint8_t Data)
{
	/* Loop while DR register is not empty */
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);

	/* Send byte through the SPIx peripheral */
	SPI_I2S_SendData(SPIx, (uint16_t)Data);

	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);

	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPIx);
}

/**
 * @brief	Writes data to the SPI
 * @param	Data: data to be written to the SPI
 * @retval	None
 */
void RF_SPI1_Write(uint8_t Data)
{
	/* Loop while DR register is not empty */
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);

	/* Send byte through the SPIx peripheral */
	SPI_I2S_SendData(SPIx, (uint16_t)Data);
}


/* Interrupt Handlers --------------------------------------------------------*/
