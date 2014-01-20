/**
 ******************************************************************************
 * @file	nrf24l01.c
 * @author	Hampus Sandberg
 * @version	0.1
 * @date	2013-04-27
 * @brief	
 ******************************************************************************
 */

/*
TODO:
- NRF24L01_Write(), so the user don't have to think about splitting the data into payloads
- Implement how different addresses are handled
*/

/* Includes ------------------------------------------------------------------*/
#include "millis.h"

#include "nrf24l01_register_map.h"
#include "nrf24l01.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"



/* Private defines -----------------------------------------------------------*/
//#if !defined(IRQ_PIN) && !defined(IRQ_DDR) && !defined(IRQ_PORT) && !defined(INTERRUPT_VECTOR) && !defined(IRQ_INTERRUPT)
//#error "Please define IRQ_PIN, IRQ_DDR, IRQ_PORT, INTERRUPT_VECTOR, IRQ_INTERRUPT in project properties"
//#endif
// #define IRQ_PIN				PORTD2	// INT0
// #define IRQ_DDR				DDRD
// #define IRQ_PORT			PORTD
// #define INTERRUPT_VECTOR	INT1_vect
// #define IRQ_INTERRUPT		1


#define TxPowerup(DEVICE)				writeRegisterOneByte(DEVICE, CONFIG, CONFIG_VALUE | ((1 << PWR_UP) | (0 << PRIM_RX)))
#define RxPowerup(DEVICE)				writeRegisterOneByte(DEVICE, CONFIG, CONFIG_VALUE | ((1 << PWR_UP) | (1 << PRIM_RX)))
#define ResetStatus(DEVICE)				writeRegisterOneByte(DEVICE, STATUS, (1 << TX_DS) | (1 << MAX_RT))
#define ResetStatusRxDr(DEVICE)  		writeRegisterOneByte(DEVICE, STATUS, (1 << RX_DR))
#define ResetStatusAll(DEVICE)			writeRegisterOneByte(DEVICE, STATUS, (1 << TX_DS) | (1 << MAX_RT) | (1 << RX_DR))

#define IsValidPipe(PIPE)				((PIPE) < 6)
#define GetPipeNumber(DEVICE) 			(((NRF24L01_GetStatus(DEVICE)) & 0xF) >> 1)
#define GetPipeFromStatus(THE_STATUS) 	(((THE_STATUS) & 0xF) >> 1)

#define DEFAULT_CHANNEL		66

//#define CONFIG_VALUE		((1 << MASK_RX_DR) | (1 << EN_CRC) | (0 << CRCO))
#define CONFIG_VALUE		((1 << EN_CRC) | (0 << CRCO))		 // Test with RX data ready interrupt

#define TIMEOUT_WRITE		500
#define TX_MODE_TIMEOUT		500

#define MAX_PIPES			6


/* Private variables ---------------------------------------------------------*/
/* Private Function Prototypes -----------------------------------------------*/
static void enableRf(NRF24L01_Device* Device) { GPIO_SetBits(Device->CE_GPIO, Device->CE_Pin); }
static void disableRf(NRF24L01_Device* Device) { GPIO_ResetBits(Device->CE_GPIO, Device->CE_Pin); }

static void selectNrf24l01(NRF24L01_Device* Device) { GPIO_ResetBits(Device->CSN_GPIO, Device->CSN_Pin); }
static void deselectNrf24l01(NRF24L01_Device* Device) { GPIO_SetBits(Device->CSN_GPIO, Device->CSN_Pin); }

static void writeRegisterOneByte(NRF24L01_Device* Device, uint8_t Register, uint8_t Data);
static void readRegister(NRF24L01_Device* Device, uint8_t Register, uint8_t* Storage, uint8_t ByteCount);
static void writeRegister(NRF24L01_Device* Device, uint8_t Register, uint8_t * Data, uint8_t ByteCount);

static void flushTX(NRF24L01_Device* Device);
static void flushRX(NRF24L01_Device* Device);
static void resetToRx(NRF24L01_Device* Device);

static uint8_t getData(NRF24L01_Device* Device, uint8_t* Storage);
static uint8_t dataReady(NRF24L01_Device* Device);

static void powerUpRx(NRF24L01_Device* Device);
static void powerUpTx(NRF24L01_Device* Device);


/* Functions -----------------------------------------------------------------*/
/**
 * @brief	Initializes the nRF24L01
 * @param	Device: The device to use
 * @retval	None
 */
void NRF24L01_Init(NRF24L01_Device* Device)
{
	Device->ChecksumErrors = 0;
	
	uint8_t i;
	for (i = 0; i < MAX_PIPES; i++) { circularBuffer_Init(&Device->RxPipeBuffer[i]); }
	
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Initialize CE (Chip Enable) pin */
	if (Device->CE_GPIO == GPIOA)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	else if (Device->CE_GPIO == GPIOB)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	else if (Device->CE_GPIO == GPIOC)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	else if (Device->CE_GPIO == GPIOD)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin 		= Device->CE_Pin;
	GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_50MHz;
	GPIO_Init(Device->CE_GPIO, &GPIO_InitStructure);

	/* Initialize CSN (Chip select) pin */
	if (Device->CSN_GPIO == GPIOA)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	else if (Device->CSN_GPIO == GPIOB)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	else if (Device->CSN_GPIO == GPIOC)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	else if (Device->CSN_GPIO == GPIOD)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);


	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin 		= Device->CSN_Pin;
	GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_50MHz;
	GPIO_Init(Device->CSN_GPIO, &GPIO_InitStructure);

	disableRf(Device);
	deselectNrf24l01(Device);

	/* Initialize IRQ (Interrupt) pin */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	if (Device->IRQ_GPIO == GPIOA)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	else if (Device->IRQ_GPIO == GPIOB)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	else if (Device->IRQ_GPIO == GPIOC)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	else if (Device->IRQ_GPIO == GPIOD)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);


	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin 		= Device->IRQ_Pin;
	GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_50MHz;
	GPIO_Init(Device->IRQ_GPIO, &GPIO_InitStructure);

	/* Initialize interrupt */
	/* Connect EXTIx Line to IRQ pin */
	GPIO_EXTILineConfig(Device->IRQ_GPIO_PortSource, Device->IRQ_GPIO_PinSource);

	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line 		= Device->IRQ_EXTI_Line;
	EXTI_InitStructure.EXTI_LineCmd 	= ENABLE;
	EXTI_InitStructure.EXTI_Mode 		= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger 	= EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTIx Interrupt to the lowest priority */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel 						= Device->IRQ_NVIC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    /* Initialize spi module */
	Device->SPIx_Init();

	/* Set channel */
	NRF24L01_SetRFChannel(Device, DEFAULT_CHANNEL);

	/* Set length of incoming payload */
	writeRegisterOneByte(Device, RX_PW_P0, PAYLOAD_SIZE);
	writeRegisterOneByte(Device, RX_PW_P1, PAYLOAD_SIZE);
	writeRegisterOneByte(Device, RX_PW_P2, PAYLOAD_SIZE);
	writeRegisterOneByte(Device, RX_PW_P3, PAYLOAD_SIZE);
	writeRegisterOneByte(Device, RX_PW_P4, PAYLOAD_SIZE);
	writeRegisterOneByte(Device, RX_PW_P5, PAYLOAD_SIZE);
	
	// Enable all RX pipes
	//NRF24L01_EnablePipes(Device, ALL_PIPES);
	NRF24L01_EnablePipes(Device, (PIPE_0 | PIPE_1));

	// Flush buffers
	flushTX(Device);
	flushRX(Device);
	ResetStatusAll(Device);

	//Замаскируем прерывания.
	uint8_t config = 0;
	readRegister(Device, CONFIG, &config, 1);
	//config |= ((1 << MASK_TX_DS) | (1 << MASK_MAX_RT));
	//writeRegisterOneByte(Device, CONFIG, config);

	// Start receiver
	Device->InTxMode = False;	/* Start in receiving mode */
	RxPowerup(Device);			/* Power up in receiving mode */
	enableRf(Device);			/* Listening for pakets */

	Device->Initialized = True;
}

/**
 * @brief	Write data and send it to the address specified in TX_ADDR
 * @param	Device: The device to use
 * @param	Data: Pointer to where the data is stored
 * @param	ByteCount: The number of bytes in Data
 * @retval	None
 * @note	W_TX_PAYLOAD commands wants LSByte first but Data is MSByte so the for loop is backwards
 *			Send as much data as defined in the payload
 */
void NRF24L01_WritePayload(NRF24L01_Device* Device, uint8_t* Data, uint8_t DataCount)
{
	/* You can only send the amount of data specified in MAX_DATA_COUNT */
	if (DataCount > MAX_DATA_COUNT) return;

	/* Wait until last payload is sent */
    while (Device->InTxMode)
    {
    	/* TODO: FIX THIS!!! */
    	uint8_t status = NRF24L01_GetStatus(Device);
    	// Data Sent TX FIFO interrupt, asserted when packet transmitted on TX.
    	if (status & (1 << TX_DS))
    	{
    		powerUpRx(Device);
    	}
    }
	
	uint8_t checksum = NRF24L01_GetChecksum(Device, Data, DataCount);
	
    disableRf(Device);
	powerUpTx(Device);
	flushTX(Device);
    
    selectNrf24l01(Device);
    Device->SPIx_Write(W_TX_PAYLOAD);
	Device->SPIx_Write(DataCount);	// Write data count
	uint8_t i;
	for (i = 0; i < DataCount; i++) Device->SPIx_Write(Data[i]);		// Write data
	Device->SPIx_Write(checksum);	// Write checksum
	for (i++; i <= MAX_DATA_COUNT; i++) Device->SPIx_Write(PAYLOAD_FILLER_DATA);	// Fill the rest of the payload
    deselectNrf24l01(Device);
    
    enableRf(Device);

    uint8_t status;
    for(;;)
    {
    	status = NRF24L01_GetStatus(Device);

    	if ((status & (1 << TX_DS)) || (status & (1 << MAX_RT)))
    	{
    		ResetStatus(Device);
    		powerUpRx(Device);
    		break;
    	}
    }
}


/**
 * @brief	Write data of arbitrary length [0-255]
 * @param	Device: The device to use
 * @param	Data: Pointer to where the data is stored
 * @param	ByteCount: The number of bytes in Data
 * @retval	None
 */
void NRF24L01_Write(NRF24L01_Device* Device, uint8_t* Data, uint8_t DataCount)
{
	// Only copy to good-sized parts if necessary
	if (DataCount <= MAX_DATA_COUNT)
	{
		NRF24L01_WritePayload(Device, Data, DataCount);
	}
	else
	{
		uint8_t tempData[MAX_DATA_COUNT];
		uint8_t tempDataCount = 0;
		uint8_t payloadCount = 0;

		uint16_t i;
		for (i = 0; i < DataCount; i++)
		{
			tempData[i - payloadCount * (MAX_DATA_COUNT)] = Data[i];
			tempDataCount++;
				
			if (tempDataCount == MAX_DATA_COUNT)
			{
				NRF24L01_WritePayload(Device, tempData, tempDataCount);
				tempDataCount = 0;
				payloadCount += 1;
				
				// ERROR: Doesn't seem to work if DataCount > 60
			}
		}
			
		// If it wasn't a multiple of MAX_DATA_COUNT write the rest
		if (tempDataCount)
		{
			NRF24L01_WritePayload(Device, Data, tempDataCount);
		}
	}
}


/**
 * @brief	Set the address for the different RX pipes on the nRF24L01
 * @param	Device: The device to use
 * @param	Pipe: The pipe to set
 * @param	AddressMSBytes: The 4 highest bytes in the address
 * @param	AddressLSByte: The lowest byte in the address
 * @retval	The status register
 */
uint8_t NRF24L01_SetRxPipeAddressSeparated(NRF24L01_Device* Device, uint8_t Pipe, uint32_t AddressMSBytes, uint8_t AddressLSByte)
{
	if (Pipe <= 1)
	{
		selectNrf24l01(Device);
		uint8_t status = Device->SPIx_WriteRead(W_REGISTER | (RX_ADDR_P0 + Pipe));
		Device->SPIx_Write(AddressLSByte);
		uint8_t i;
		for (i = 0; i < 4; i++)
		{
			Device->SPIx_Write((AddressMSBytes >> 8*i) & 0xFF);
		}
		deselectNrf24l01(Device);
		
		return status;
	}
	else if (IsValidPipe(Pipe))
	{
		selectNrf24l01(Device);
		uint8_t status = Device->SPIx_WriteRead(W_REGISTER | (RX_ADDR_P0 + Pipe));
		Device->SPIx_Write(AddressLSByte);	// MSByte of address in pipe 2 to 5 is equal to RX_ADDR_P1[39:8]
		deselectNrf24l01(Device);
		
		return status;
	}
	return 0;
}

/**
 * @brief	Set the address for the TX on the nRF24L01
 * @param	Device: The device to use
 * @param	AddressMSBytes: The 4 highest bytes in the address
 * @param	AddressLSByte: The lowest byte in the address
 * @retval	The status register
 */
uint8_t NRF24L01_SetTxAddressSeparated(NRF24L01_Device* Device, uint32_t AddressMSBytes, uint8_t AddressLSByte)
{
	selectNrf24l01(Device);
	uint8_t status = Device->SPIx_WriteRead(W_REGISTER | TX_ADDR);
	Device->SPIx_WriteRead(AddressLSByte);
	uint8_t i;
	for (i = 0; i < 4; i++)
	{
		Device->SPIx_WriteRead((AddressMSBytes >> 8*i) & 0xFF);
	}
	deselectNrf24l01(Device);
	
	return status;
}

/**
 * @brief	Set the RF channel to transmit on
 * @param	Device: The device to use
 * @param	Channel: The channel
 * @retval	The status register or 0 if invalid channel
 * @note	Freq = 2400 + RF_CH [MHz], -> 2400 MHz - 2525 MHz operation frequencies
 */
uint8_t NRF24L01_SetRFChannel(NRF24L01_Device* Device, uint8_t Channel)
{
	if (Channel <= 125)
	{
		selectNrf24l01(Device);
		Device->SPIx_WriteRead(W_REGISTER | RF_CH);
		Device->SPIx_WriteRead(Channel);
		deselectNrf24l01(Device);

		return 1;
	}
	return 0;
}

/**
 * @brief	Reads the statusregister in the nRF24L01
 * @param	Device: The device to use
 * @param	None
 * @retval	The status register
 */
uint8_t NRF24L01_GetStatus(NRF24L01_Device* Device)
{
	selectNrf24l01(Device);
	uint8_t status = Device->SPIx_WriteRead(0);
	deselectNrf24l01(Device);
	
	return status;
}

/**
 * @brief	Get the TX FIFO status
 * @param	Device: The device to use
 * @param	None
 * @retval	The TX FIFO status
 */
uint8_t NRF24L01_GetFIFOStatus(NRF24L01_Device* Device)
{
	uint8_t FIFOStatus = 0;
	readRegister(Device, FIFO_STATUS, &FIFOStatus, 1);
	return FIFOStatus;
}

/**
 * @brief	Check if the TX FIFO is empty
 * @param	Device: The device to use
 * @param	None
 * @retval	1: If empty
 * @retval	0: If not empty
 */
uint8_t NRF24L01_TxFIFOEmpty(NRF24L01_Device* Device)
{
	uint8_t FIFOStatus = NRF24L01_GetFIFOStatus(Device);
	return ((FIFOStatus & (1 << TX_EMPTY)) >> TX_EMPTY);
}

/**
 * @brief	Enable the specified pipes on nRF24L01
 * @param	Device: The device to use
 * @param	Pipes: The pipes that should be enabled
 * @retval	The status register
 */
void NRF24L01_EnablePipes(NRF24L01_Device* Device, uint8_t Pipes)
{
	if (Pipes <= 0x3F)
	{
		uint8_t pipeValue;
		readRegister(Device, EN_RXADDR, &pipeValue, 1);
		
		pipeValue |= (Pipes);
		writeRegisterOneByte(Device, EN_RXADDR, pipeValue);
	}
}

/**
 * @brief	Disable the specified pipes on nRF24L01
 * @param	Device: The device to use
 * @param	Pipes: The pipes that should be disabled
 * @retval	The status register
 */
void NRF24L01_DisablePipes(NRF24L01_Device* Device, uint8_t Pipes)
{
	if (Pipes <= 0x3F)
	{
		uint8_t pipeValue;
		readRegister(Device, EN_RXADDR, &pipeValue, 1);
		pipeValue &= ~(Pipes);
		
		writeRegisterOneByte(Device, EN_RXADDR, pipeValue);
	}
}

/**
 * @brief	Get the pipe which there's data on
 * @param	Device: The device to use
 * @param	None
 * @retval	The pipe number
 */
uint8_t NRF24L01_GetPipeNumber(NRF24L01_Device* Device)
{
	return GetPipeNumber(Device);
}

/**
 * @brief	Check if a pipe has available data in it's buffer
 * @param	Device: The device to use
 * @param	Pipe: The pipe to check for data
 * @retval	The available data for the specified pipe
 */
uint8_t NRF24L01_GetAvailableDataForPipe(NRF24L01_Device* Device, uint8_t Pipe)
{
	if (IsValidPipe(Pipe))
	{
		return circularBuffer_GetCount(&Device->RxPipeBuffer[Pipe]);
	}
	return 0;
}

/**
 * @brief	Get a certain amount of data from a specified pipe
 * @param	Device: The device to use
 * @param	Pipe: The pipe to check for data
 * @param	Storage: Pointer to where the data should be stored
 * @param	DataCount: The amount of data to get
 * @retval	None
 * @note	It's good to check that the requested amount of data is available
			first by calling NRF24L01_GetAvailableDataForPipe()
 */
void NRF24L01_GetDataFromPipe(NRF24L01_Device* Device, uint8_t Pipe, uint8_t* Storage, uint8_t DataCount)
{
	if (IsValidPipe(Pipe))
	{
		uint8_t i;
		for (i = 0; i < DataCount; i++)
		{
			Storage[i] = circularBuffer_Remove(&Device->RxPipeBuffer[Pipe]);
		}
	}
}

/**
 * @brief	Get the checksum for a package of data
 * @param	Device: The device to use
 * @param	Data: The data for which to calculate the checksum on
 * @param	DataCount: The amount of data
 * @retval	The calculated checksum
 * @note	Checksum = ~(DataCount + Data1 + Data2 + ... + Data N)
 */
uint8_t NRF24L01_GetChecksum(NRF24L01_Device* Device, uint8_t* Data, uint8_t DataCount)
{
	volatile uint8_t checksum = DataCount;
	uint8_t i;
	for (i = 0; i < DataCount; i++)
	{
		checksum += Data[i];
	}
	return ~checksum;
}

/**
 * @brief	Get the amount of checksum errors that has occurred
 * @param	Device: The device to use
 * @param	None
 * @retval	The amount of checksum errors
 */
uint16_t NRF24L01_GetChecksumErrors(NRF24L01_Device* Device)
{
	return Device->ChecksumErrors;
}


/**
 * @brief	Write some debug info to the UART
 * @param	Device: The device to use
 * @param	None
 * @retval	None
 */
/*
void NRF24L01_WriteDebugToUart(NRF24L01_Device* Device)
{
	UART_WriteString("------------\r");
	uint8_t status = NRF24L01_GetStatus(Device);
	UART_WriteString("Status: ");
	UART_WriteHexByte(status, 1);
	UART_WriteString("\r");
	
	uint8_t pipe = NRF24L01_GetPipeNumber(Device);
	UART_WriteString("Pipe: ");
	UART_WriteUintAsString(pipe);
	UART_WriteString("\r");
	
	uint8_t en_rxaddr;
	readRegister(Device, EN_RXADDR, &en_rxaddr, 1);
	UART_WriteString("EN_RXADDR: ");
	UART_WriteHexByte(en_rxaddr, 1);
	UART_WriteString("\r");

	uint8_t addrTx[5];
	readRegister(Device, TX_ADDR, addrTx, 5);
	UART_WriteString("TX_ADDR: ");
	UART_WriteHexByte(addrTx[0], 1);
	UART_WriteHexByte(addrTx[1], 0);
	UART_WriteHexByte(addrTx[2], 0);
	UART_WriteHexByte(addrTx[3], 0);
	UART_WriteHexByte(addrTx[4], 0);
	UART_WriteString("\r");

	uint8_t addr0[5];
	readRegister(Device, RX_ADDR_P0, addr0, 5);
	UART_WriteString("RX_ADDR_P0: ");
	UART_WriteHexByte(addr0[0], 1);
	UART_WriteHexByte(addr0[1], 0);
	UART_WriteHexByte(addr0[2], 0);
	UART_WriteHexByte(addr0[3], 0);
	UART_WriteHexByte(addr0[4], 0);
	UART_WriteString("\r");

	uint8_t addr1[5];
	readRegister(Device, RX_ADDR_P1, addr1, 5);
	UART_WriteString("RX_ADDR_P1: ");
	UART_WriteHexByte(addr1[0], 1);
	UART_WriteHexByte(addr1[1], 0);
	UART_WriteHexByte(addr1[2], 0);
	UART_WriteHexByte(addr1[3], 0);
	UART_WriteHexByte(addr1[4], 0);
	UART_WriteString("\r");

	uint8_t addr2;
	readRegister(Device, RX_ADDR_P2, &addr2, 1);
	UART_WriteString("RX_ADDR_P2: ");
	UART_WriteHexByte(addr2, 1);
	UART_WriteString("\r");

	uint8_t addr3;
	readRegister(Device, RX_ADDR_P3, &addr3, 1);
	UART_WriteString("RX_ADDR_P3: ");
	UART_WriteHexByte(addr3, 1);
	UART_WriteString("\r");

	uint8_t addr4;
	readRegister(Device, RX_ADDR_P4, &addr4, 1);
	UART_WriteString("RX_ADDR_P4: ");
	UART_WriteHexByte(addr4, 1);
	UART_WriteString("\r");

	uint8_t addr5;
	readRegister(Device, RX_ADDR_P5, &addr5, 1);
	UART_WriteString("RX_ADDR_P5: ");
	UART_WriteHexByte(addr5, 1);
	UART_WriteString("\r");
}
*/

/* Private Functions ---------------------------------------------------------*/
/**
 * @brief	Write one byte to a register in the nRF24L01
 * @param	Device: The device to use
 * @param	Register: The register to read from
 * @param	Data: The byte to write
 * @retval	None
 */
static void writeRegisterOneByte(NRF24L01_Device* Device, uint8_t Register, uint8_t Data)
{
	if (IS_VALID_REGISTER(Register))
	{
		selectNrf24l01(Device);
		Device->SPIx_WriteRead(W_REGISTER | (REGISTER_MASK & Register));
		Device->SPIx_WriteRead(Data);
		deselectNrf24l01(Device);
	}		
}

/**
 * @brief	Reads data from a register in the nRF24L01
 * @param	Device: The device to use
 * @param	Register: The register to read from
 * @param	Storage: Pointer to where to store the read data
 * @param	ByteCount: How many bytes to read
 * @retval	The status register
 * @note	LSByte is read first
 */
static void readRegister(NRF24L01_Device* Device, uint8_t Register, uint8_t* Storage, uint8_t ByteCount)
{
	if (IS_VALID_REGISTER(Register))
	{
		selectNrf24l01(Device);
		Device->SPIx_WriteRead(R_REGISTER | Register);
		uint8_t i;
		for (i = 0; i < ByteCount; i++)
		{
			Storage[i] = Device->SPIx_WriteRead(0);
		}
		deselectNrf24l01(Device);
	}	
}

/**
 * @brief	Write data to a register in the nRF24L01
 * @param	Device: The device to use
 * @param	Register: The register to read from
 * @param	Storage: Pointer to where to store the read data
 * @param	ByteCount: How many bytes to read
 * @retval	None
 */
static void writeRegister(NRF24L01_Device* Device, uint8_t Register, uint8_t* Data, uint8_t ByteCount)
{
	if (IS_VALID_REGISTER(Register))
	{
		selectNrf24l01(Device);
		Device->SPIx_Write(W_REGISTER | Register);
		uint8_t i;
		for (i = 0; i < PAYLOAD_SIZE; i++)
		{
			Device->SPIx_WriteRead(Data[i]);
		}
		deselectNrf24l01(Device);
	}	
}

/**
 * @brief	Flush the TX buffer
 * @param	Device: The device to use
 * @retval	None
 */
static void flushTX(NRF24L01_Device* Device)
{
	selectNrf24l01(Device);
	Device->SPIx_WriteRead(FLUSH_TX);
	deselectNrf24l01(Device);
}

/**
 * @brief	Flush the RX buffer
 * @param	Device: The device to use
 * @retval	None
 */
static void flushRX(NRF24L01_Device* Device)
{
	selectNrf24l01(Device);
	Device->SPIx_WriteRead(FLUSH_RX);
	deselectNrf24l01(Device);
}

/**
 * @brief	Reset everything and power up as a receiver
 * @param	Device: The device to use
 * @retval	None
 */
static void resetToRx(NRF24L01_Device* Device)
{
	disableRf(Device);
	RxPowerup(Device);
	enableRf(Device);
	Device->InTxMode = False;
		
	ResetStatus(Device);
}

/**
 * @brief	Gets data from the RX buffer
 * @param	Device: The device to use
 * @param	Storage: Pointer to where the data should be stored
 * @retval	The amount of data received
 * @note	The checksum is checked here and if it doesn't match it will return 0 as
 *			the amount of data
 */
static uint8_t getData(NRF24L01_Device* Device, uint8_t* Storage)
{	
	selectNrf24l01(Device);
	Device->SPIx_WriteRead(R_RX_PAYLOAD);
	uint8_t dataCount = Device->SPIx_WriteRead(PAYLOAD_FILLER_DATA);
	uint8_t i;
	for (i = 0; i < dataCount + 1; i++)
	{
		Storage[i] = Device->SPIx_WriteRead(PAYLOAD_FILLER_DATA);
	}	
	deselectNrf24l01(Device);
	flushRX(Device);
	writeRegisterOneByte(Device, STATUS, (1 << RX_DR));
	
	return dataCount;
}

/**
 * @brief	Checks if there is data available
 * @param	Device: The device to use
 * @retval	0: If no data is ready
 * @retval	1: If data is ready
 */
static uint8_t dataReady(NRF24L01_Device* Device)
// Checks if data is available for reading
{
	static uint32_t timeoutTimer = 0;
	if (Device->InTxMode)
	{
		if (millis() - timeoutTimer >= TX_MODE_TIMEOUT) resetToRx(Device);
		return 0;
	}
	
	volatile uint8_t status = NRF24L01_GetStatus(Device);
	return (status & (1 << RX_DR));
}

/**
 * @brief	Power up in RX mode
 * @param	Device: The device to use
 * @retval	None
 */
static void powerUpRx(NRF24L01_Device* Device)
{
	Device->InTxMode = False;
	writeRegisterOneByte(Device, CONFIG, CONFIG_VALUE | ((1 << PWR_UP) | (1 << PRIM_RX)));
	writeRegisterOneByte(Device, STATUS, (1 << TX_DS) | (1 << MAX_RT));
	//_delay_us(50);		// NOT GOOD!!!
}

/**
 * @brief	Power up in TX mode
 * @param	Device: The device to use
 * @retval	None
 */
static void powerUpTx(NRF24L01_Device* Device)
{
	Device->InTxMode = True;
	writeRegisterOneByte(Device, CONFIG, CONFIG_VALUE | ((1 << PWR_UP) | (0 << PRIM_RX)));
}

/* Interrupt Service Routines ------------------------------------------------*/
void NRF24L01_Interrupt(NRF24L01_Device* Device)
{
	uint8_t status = NRF24L01_GetStatus(Device);
	// Data Sent TX FIFO interrupt, asserted when packet transmitted on TX.
	if (status & (1 << TX_DS))
	{
		ResetStatus(Device);
		powerUpRx(Device);
		//GPIO_SetBits(GPIOC, GPIO_Pin_9);
	}
	else if (status & (1 << MAX_RT))
	{
		ResetStatus(Device);
		powerUpRx(Device);
	}
	else if (status & (1 << RX_DR))
	{
		uint8_t pipe = GetPipeFromStatus(status);
		if (IsValidPipe(pipe))
		{
			uint8_t buffer[MAX_DATA_COUNT];
			uint8_t availableData = getData(Device, buffer);

			uint8_t receivedChecksum = buffer[availableData];
			uint8_t calculatedChecksum = NRF24L01_GetChecksum(Device, buffer, availableData);

			if (receivedChecksum == calculatedChecksum)
			{
				uint8_t i;
				for (i = 0; i < availableData; i++)
				{
					if (!circularBuffer_IsFull(&Device->RxPipeBuffer[pipe]))
						circularBuffer_Insert(&Device->RxPipeBuffer[pipe], buffer[i]);
				}
			}
			else
			{
				// Checksum error
				Device->ChecksumErrors++;
			}
		}
		ResetStatusRxDr(Device);
	}
}
