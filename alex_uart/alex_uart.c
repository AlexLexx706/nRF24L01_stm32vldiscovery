#include "alex_uart.h"
#include <stm32f10x_conf.h>
#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>


CircularBuffer_TypeDef * buffer = 0;

//Инициализируем USART2
void init_usart(uint8_t speed, CircularBuffer_TypeDef * __buffer)
{
	buffer = __buffer;
	circularBuffer_Init(buffer);
    GPIO_InitTypeDef GPIO_InitStructure; //Структура содержащая настройки порта
    USART_InitTypeDef USART_InitStructure; //Структура содержащая настройки USART

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

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


void USART2_IRQHandler (void)
{
    //Приём байта
    if ( USART_GetITStatus(USART2, USART_IT_RXNE) != RESET )
    {
    	//пишем данные в буффер.
    	circularBuffer_Insert(buffer, USART2->DR);

    	//чистим флаг прерывания
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}
