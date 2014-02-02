#ifndef __ALEX_UART_H_
#define __ALEX_UART_H_
#include "stm32f10x.h"
#include "circularBuffer.h"

void init_usart(uint8_t speed, CircularBuffer_TypeDef * buffer);
void USART2_IRQHandler (void);
#endif
