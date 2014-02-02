#ifndef __ALEX_UART_H_
#define __ALEX_UART_H_
#include "stm32f10x.h"
#include "circularBuffer.h"

//������������� uart
void init_usart(uint32_t speed,					//�������� � �����
		CircularBuffer_TypeDef * __in_buffer,	//������� ��������� ������
		CircularBuffer_TypeDef * __out_buffer); //�������� ��������� ������

//�������� ������ � �������� ��������� ������
void send_data(CIRCULARBUFFER_DATATYPE data);

#endif
