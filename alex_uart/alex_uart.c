#include "alex_uart.h"
#include <stm32f10x_conf.h>
#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>


CircularBuffer_TypeDef * in_buffer = 0;
CircularBuffer_TypeDef * out_buffer = 0;

//Инициализируем USART2
void init_usart(uint32_t speed,
				CircularBuffer_TypeDef * __in_buffer,
				CircularBuffer_TypeDef * __out_buffer)
{
	in_buffer = __in_buffer;
	out_buffer = __out_buffer;

	if (in_buffer)
		circularBuffer_Init(in_buffer);

	if (out_buffer)
		circularBuffer_Init(out_buffer);


    GPIO_InitTypeDef GPIO_InitStructure; //Структура содержащая настройки порта
    USART_InitTypeDef USART_InitStructure; //Структура содержащая настройки USART

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    //Конфигурируем PA2 как альтернативную функцию -> TX UART.
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //Конфигурируем PA3 как альтернативную функцию -> RX UART.
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //Инициализируем UART с дефолтными настройками
    USART_StructInit(&USART_InitStructure); 
    USART_InitStructure.USART_BaudRate            = speed;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART2, &USART_InitStructure);

    //Прерывание по приему
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    //Включаем UART
    USART_Cmd(USART2, ENABLE);

    //Включаем прерывания от UART
    NVIC_EnableIRQ(USART2_IRQn); 
    
    //Прерывание от UART, приоритет 0, самый высокий
    NVIC_SetPriority(USART2_IRQn, 0);
}

void send_data(CIRCULARBUFFER_DATATYPE data)
{
	if (out_buffer)
	{
		//1. отключим прерывания.
		USART_ITConfig(USART2, USART_IT_TC, DISABLE);

		//2. положим данные в буффер.
		circularBuffer_Insert(out_buffer, data);

		//3. Инициализируем передачу.
	    if ( USART_GetFlagStatus(USART2, USART_FLAG_TXE) != RESET )
	    	USART_SendData(USART2, circularBuffer_Remove(out_buffer));

	    //4. включим прерывания.
		USART_ITConfig(USART2, USART_IT_TC, ENABLE);
	}
}


void USART2_IRQHandler (void)
{
    //Приём байта
    if ( USART_GetITStatus(USART2, USART_IT_RXNE) != RESET )
    {
    	//пишем данные в буффер.
    	char uart_data = USART2->DR;

    	if (in_buffer)
    		circularBuffer_Insert(in_buffer, uart_data);

    	//чистим флаг прерывания
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }

    //Предача байта из буффера
    if ( USART_GetITStatus(USART2, USART_IT_TC) != RESET )
    {
    	if ( !circularBuffer_IsEmpty(out_buffer) )
    		USART_SendData(USART2, circularBuffer_Remove(out_buffer));

    	USART_ClearITPendingBit(USART2, USART_IT_TC);
    }
}
