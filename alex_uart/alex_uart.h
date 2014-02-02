#ifndef __ALEX_UART_H_
#define __ALEX_UART_H_
#include "stm32f10x.h"
#include "circularBuffer.h"

//инициализация uart
void init_usart(uint32_t speed,					//скорость в бодах
		CircularBuffer_TypeDef * __in_buffer,	//входной кольцевой буффер
		CircularBuffer_TypeDef * __out_buffer); //выходной кольцевой буффер

//Положить данные в выходной кольцевой буффер
void send_data(CIRCULARBUFFER_DATATYPE data);

#endif
