#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"

#include "nrf24l01.h"
#include "nrf24l01_register_map.h"
#include "rf_spi2.h"
#include "rf_spi1.h"

NRF24L01_Device NRF24L01_2;
NRF24L01_Device NRF24L01_1;



#define DEVICE_1_ADDRESS	0X0101010101LL
#define DEVICE_2_ADDRESS	0X0202020202LL

void BOARD_Init()
{
	/* nRF24L01_1 */
	NRF24L01_1.NRF24L01_DeviceName	= "1";
	NRF24L01_1.CSN_Pin 				= GPIO_Pin_12;
	NRF24L01_1.CSN_GPIO				= GPIOB;

	NRF24L01_1.CE_Pin 				= GPIO_Pin_6;
	NRF24L01_1.CE_GPIO				= GPIOC;

	NRF24L01_1.IRQ_Pin 				= GPIO_Pin_1;
	NRF24L01_1.IRQ_GPIO				= GPIOA;
	NRF24L01_1.IRQ_GPIO_PortSource 	= GPIO_PortSourceGPIOA;
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
	NRF24L01_EnableByteMode(&NRF24L01_1, True);
}


/* Interrupt Service Routines ------------------------------------------------*/
void EXTI1_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		NRF24L01_Interrupt(&NRF24L01_1);

		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

int main(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef gpio_init_struct;
	GPIO_StructInit(&gpio_init_struct);
	gpio_init_struct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &gpio_init_struct);

    GPIO_ResetBits(GPIOC, GPIO_Pin_8);
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);


	BOARD_Init();
	uint8_t status = NRF24L01_GetStatus(&NRF24L01_1);


	while(1)
    {
		if (circularBuffer_GetCount(&NRF24L01_1.RxPipeBuffer[1]) > 0 )
		{
			uint8_t data;

			while (!circularBuffer_IsEmpty(&NRF24L01_1.RxPipeBuffer[1]) )
			{
				data = circularBuffer_Remove(&NRF24L01_1.RxPipeBuffer[1]);
				GPIO_SetBits(GPIOC, GPIO_Pin_8);

				uint32_t c = 24000;
				while (c){c--;};

				GPIO_ResetBits(GPIOC, GPIO_Pin_8);

				c = 24000;
				while (c){c--;};
			}
		}
    }
}
