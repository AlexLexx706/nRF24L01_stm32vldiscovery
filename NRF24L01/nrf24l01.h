/**
 ******************************************************************************
 * @file	nrf24l01.h
 * @author	Hampus Sandberg
 * @version	0.1
 * @date	2013-04-27
 * @brief	
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef NRF24L01_H_
#define NRF24L01_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "common.h"
#include "circularBuffer.h"

/* Defines -------------------------------------------------------------------*/
#define MSB_BYTES(BYTES)	((BYTES >> 8) & 0xFFFFFFFF)
#define LSB_BYTE(BYTES)		(BYTES & 0xFF)

#define NRF24L01_SetTxAddress(Device, Address)	(NRF24L01_SetTxAddressSeparated(Device, MSB_BYTES(Address), LSB_BYTE(Address)))
#define NRF24L01_SetRxPipeAddress(Device, Pipe, Address) (NRF24L01_SetRxPipeAddressSeparated(Device, Pipe, MSB_BYTES(Address), LSB_BYTE(Address)))

#define PAYLOAD_SIZE		32
#define DATA_COUNT_INDEX	0
#define MAX_DATA_COUNT		PAYLOAD_SIZE-1	// 1 byte datacount + 1 byte checksum
#define PAYLOAD_FILLER_DATA	0x00

#define NRF24L01_MAX_AVAILABLE_DATA	CIRCULARBUFFER_SIZE

/* Typedefs ------------------------------------------------------------------*/
typedef struct
{
	char* NRF24L01_DeviceName;

	uint16_t CSN_Pin;			/* Chip select pin */
	GPIO_TypeDef* CSN_GPIO;
	uint16_t CE_Pin;			/* Chip enable pin */
	GPIO_TypeDef* CE_GPIO;

	uint16_t IRQ_Pin;			/* Interrupt pin */
	GPIO_TypeDef* IRQ_GPIO;
	uint8_t IRQ_GPIO_PortSource;
	uint8_t IRQ_GPIO_PinSource;
	uint32_t IRQ_EXTI_Line;
	uint8_t IRQ_NVIC_IRQChannel;

	SPI_TypeDef* SPIx;			/* SPI peripheral to use */
	void (*SPIx_Init)();		/* SPI Initialization function to use */
	uint8_t (*SPIx_WriteRead)(uint8_t);	/* SPI WriteRead function to use */
	void (*SPIx_Write)(uint8_t);		/* SPI Write function to use */

	CircularBuffer_TypeDef RxPipeBuffer[6];	/* Buffer for the six RX Pipes */
	uint32_t ChecksumErrors;	/* Variable to hold the amount of checksum errors */
	Boolean InTxMode;			/* True if nRF24l01 Device is in TX mode, False otherwise */
	Boolean Initialized;		/* True if initialized, False otherwise */
	Boolean byte_mode;		/* True if initialized, False otherwise */
	Boolean dynamic_payloads_enabled;

	uint8_t packet_size;
	uint8_t received_packets_count;
	uint8_t packets_count;
	uint8_t no_size;

} NRF24L01_Device;


/* Function prototypes -------------------------------------------------------*/
void NRF24L01_Init(NRF24L01_Device* Device);
void NRF24L01_WritePayload(NRF24L01_Device* Device, uint8_t* Data, uint8_t ByteCount);
void NRF24L01_Write(NRF24L01_Device* Device, uint8_t* Data, uint8_t DataCount);

uint8_t NRF24L01_SetRxPipeAddressSeparated(NRF24L01_Device* Device, uint8_t Pipe, uint32_t AddressMSBytes, uint8_t AddressLSByte);
uint8_t NRF24L01_SetTxAddressSeparated(NRF24L01_Device* Device, uint32_t AddressMSBytes, uint8_t AddressLSByte);

uint8_t NRF24L01_SetRFChannel(NRF24L01_Device* Device, uint8_t Channel);
uint8_t NRF24L01_GetStatus(NRF24L01_Device* Device);

uint8_t NRF24L01_GetFIFOStatus(NRF24L01_Device* Device);
uint8_t NRF24L01_TxFIFOEmpty(NRF24L01_Device* Device);
uint8_t NRF24L01_RxFIFOEmpty(NRF24L01_Device* Device);

void NRF24L01_EnablePipes(NRF24L01_Device* Device, uint8_t Pipes);
void NRF24L01_DisablePipes(NRF24L01_Device* Device, uint8_t Pipes);
uint8_t NRF24L01_GetPipeNumber(NRF24L01_Device* Device);
uint8_t NRF24L01_GetAvailableDataForPipe(NRF24L01_Device* Device, uint8_t Pipe);
void NRF24L01_GetDataFromPipe(NRF24L01_Device* Device, uint8_t Pipe, uint8_t* Storage, uint8_t DataCount);

uint8_t NRF24L01_GetChecksum(NRF24L01_Device* Device, uint8_t* Data, uint8_t DataCount);
uint16_t NRF24L01_GetChecksumErrors(NRF24L01_Device* Device);

void NRF24L01_WriteDebugToUart(NRF24L01_Device* Device);

void NRF24L01_Interrupt(NRF24L01_Device* Device);

void NRF24L01_EnableDynamicPayloads(NRF24L01_Device* Device);

#endif /* NRF24L01_H_ */
