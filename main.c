#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "nrf24l01.h"
#include "nrf24l01_register_map.h"
#include "rf_spi2.h"
#include "alex_uart.h"

#define DEVICE_1_ADDRESS	0X0101010101
#define DEVICE_2_ADDRESS	0X0202020202

//передтчик
NRF24L01_Device NRF24L01_1;

//прерывание от NRF24L01
void EXTI1_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		NRF24L01_Interrupt(&NRF24L01_1);
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}


void BOARD_Init()
{
	/* nRF24L01_1 */
	NRF24L01_1.NRF24L01_DeviceName	= "1";
	NRF24L01_1.CSN_Pin 				= GPIO_Pin_12;
	NRF24L01_1.CSN_GPIO				= GPIOB;

	NRF24L01_1.CE_Pin 				= GPIO_Pin_6;
	NRF24L01_1.CE_GPIO				= GPIOC;

	NRF24L01_1.IRQ_Pin 				= GPIO_Pin_1;
	NRF24L01_1.IRQ_GPIO				= GPIOC;
	NRF24L01_1.IRQ_GPIO_PortSource 	= GPIO_PortSourceGPIOC;
	NRF24L01_1.IRQ_GPIO_PinSource 	= GPIO_PinSource1;
	NRF24L01_1.IRQ_EXTI_Line 		= EXTI_Line1;
	NRF24L01_1.IRQ_NVIC_IRQChannel 	= EXTI1_IRQn;

	NRF24L01_1.SPIx 				= SPI2;
	NRF24L01_1.SPIx_Init 			= RF_SPI2_Init;
	NRF24L01_1.SPIx_WriteRead 		= RF_SPI2_WriteRead;
	NRF24L01_1.SPIx_Write 			= RF_SPI2_Write;
	NRF24L01_Init(&NRF24L01_1);

	NRF24L01_SetRFChannel(&NRF24L01_1, 66);
	NRF24L01_SetTxAddress(&NRF24L01_1, DEVICE_2_ADDRESS);
	NRF24L01_SetRxPipeAddress(&NRF24L01_1, 0, DEVICE_2_ADDRESS);
	NRF24L01_SetRxPipeAddress(&NRF24L01_1, 1, DEVICE_1_ADDRESS);

	//Затактируем порт C
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);

	//Прерывания - это альтернативная функция порта
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN , ENABLE);

	// по умолчанию там ноли, поэтому установим только 1 бит
	AFIO->EXTICR[0]|=AFIO_EXTICR1_EXTI1_PC;

	//Прерывания от первой ноги
	EXTI->IMR|=(EXTI_IMR_MR1);

	//Прерывания по спадаюшему фронту
	EXTI->FTSR|=(EXTI_RTSR_TR1);

	//Разрешаем оба прерывания
	NVIC_EnableIRQ (EXTI1_IRQn);
}

void init_lamps()
{
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef gpio_init_struct;
	GPIO_StructInit(&gpio_init_struct);
	gpio_init_struct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &gpio_init_struct);

    GPIO_ResetBits(GPIOC, GPIO_Pin_8);
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
}


CircularBuffer_TypeDef uart_in_buffer;
CircularBuffer_TypeDef uart_out_buffer;

int main(void)
{
    uint8_t buffer[40];
    uint8_t index = 0;

	//1. включаем nrf24l01
	BOARD_Init();

	//2. включим порт
	init_usart(256000, &uart_in_buffer, &uart_out_buffer);
	init_lamps();

	while(1)
    {
		if (circularBuffer_GetCount(&uart_in_buffer) > 0 )
		{
			while (!circularBuffer_IsEmpty(&uart_in_buffer) )
			{
				buffer[index] = circularBuffer_Remove(&uart_in_buffer);

				//сбока завершена.
				if ( (*buffer) == index)
				{
					GPIO_ResetBits(GPIOC, GPIO_Pin_8);
					GPIO_ResetBits(GPIOC, GPIO_Pin_9);

					NRF24L01_Write(&NRF24L01_1, buffer, buffer[0] + 1);
					index = 0;
				}
				else
					index++;
			}
		}

		if ( NRF24L01_1.send_status )
		{
			if (NRF24L01_1.send_status == 1)
			{
				GPIO_SetBits(GPIOC, GPIO_Pin_8);
				send_data(1);
				send_data(1);
			}
			else
			{
				GPIO_SetBits(GPIOC, GPIO_Pin_9);
				send_data(1);
				send_data(0);
			}
			NRF24L01_1.send_status = 0;
		}
    }
}
