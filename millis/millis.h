/**
 ******************************************************************************
 * @file	millis.h
 * @author	Hampus Sandberg
 * @version	0.1
 * @date	2013-05-09
 * @brief	Manage a millis counter
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MILLIS_H_
#define MILLIS_H_

/* Includes ------------------------------------------------------------------*/
#ifdef STM32F10X_MD_VL
#include "stm32f10x.h"
#endif
#ifdef STM32F4XX
#include "stm32f4xx.h"
#endif
/* Defines -------------------------------------------------------------------*/
/* Typedefs ------------------------------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/
void MILLIS_Init();
uint32_t millis();

#endif /* MILLIS_H_ */
