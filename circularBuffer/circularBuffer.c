/**
 ******************************************************************************
 * @file	circularBuffer.c
 * @author	Hampus Sandberg
 * @version	0.1
 * @date	2013-02-10
 * @brief	Contains the function implementations for the circular buffer.
 *			Uses ATOMIC where necessary to avoid problem
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "circularBuffer.h"

/* Private defines -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
/**
 * @brief	Initializes the circular buffer that is specified in the parameter
 * @param	CircularBuffer: the  buffer that should be initialized. If the 
 *			buffer already is initialized it will be reset
 * @retval	None
 */
void circularBuffer_Init(CircularBuffer_TypeDef* CircularBuffer)
{
	CircularBuffer->in = CircularBuffer->data;
	CircularBuffer->out = CircularBuffer->data;
	CircularBuffer->count = 0;
	CircularBuffer->Initialized = True;
	CircularBuffer->locked_count = 0;
}

/**
 * @brief	Insert an item in the front of the buffer
 * @param	CircularBuffer: the buffer to insert into
 * @param	Data: data to insert
 * @retval	None
 */
void circularBuffer_Insert(CircularBuffer_TypeDef* CircularBuffer, CIRCULARBUFFER_DATATYPE Data)
{
	*CircularBuffer->in = Data;
		
	if (++CircularBuffer->in == &CircularBuffer->data[CIRCULARBUFFER_SIZE])
		CircularBuffer->in = CircularBuffer->data;
	CircularBuffer->count++;
}

//Add locked data to buffer
void circularBuffer_InsertLocked(CircularBuffer_TypeDef* CircularBuffer, CIRCULARBUFFER_DATATYPE Data)
{
	*CircularBuffer->in = Data;

	if (++CircularBuffer->in == &CircularBuffer->data[CIRCULARBUFFER_SIZE])
		CircularBuffer->in = CircularBuffer->data;
	CircularBuffer->locked_count++;
}


//unlock locked data in buffer
void circularBuffer_UnlockData(CircularBuffer_TypeDef* CircularBuffer)
{
	CircularBuffer->count = CircularBuffer->count +  CircularBuffer->locked_count;
	CircularBuffer->locked_count = 0;
}

//Remove locked data
void circularBuffer_RemoveLockedData(CircularBuffer_TypeDef* CircularBuffer)
{
	CircularBuffer->in = CircularBuffer->in - CircularBuffer->locked_count;

	if ( CircularBuffer->in < CircularBuffer->data )
		CircularBuffer->in = &CircularBuffer->data[CIRCULARBUFFER_SIZE] - (CircularBuffer->data - CircularBuffer->in);

	CircularBuffer->locked_count = 0;
}

CIRCULARBUFFER_COUNTTYPE circularBuffer_GetFreeSize(CircularBuffer_TypeDef* CircularBuffer)
{
	return CIRCULARBUFFER_SIZE - (CircularBuffer->locked_count + CircularBuffer->count);
}



/**
 * @brief	Removes one item from the end of the buffer
 * @param	CircularBuffer: buffer to remove from
 * @retval	data removed from the end of the buffer
 */
CIRCULARBUFFER_DATATYPE circularBuffer_Remove(CircularBuffer_TypeDef* CircularBuffer)
{
	CIRCULARBUFFER_DATATYPE data = *CircularBuffer->out;
	    
    if (++CircularBuffer->out == &CircularBuffer->data[CIRCULARBUFFER_SIZE])
        CircularBuffer->out = CircularBuffer->data;
    CircularBuffer->count--;
    
    return data;
}

/**
 * @brief	Get the current count for the buffer
 * @param	CircularBuffer: the buffer to get the count for
 * @retval	the count value
 */
CIRCULARBUFFER_COUNTTYPE circularBuffer_GetCount(CircularBuffer_TypeDef* CircularBuffer)
{    
	CIRCULARBUFFER_COUNTTYPE count;
	count = CircularBuffer->count;

    return count;
}

/**
 * @brief	Check if the buffer is empty
 * @param	CircularBuffer: the buffer to check
 * @retval	(1) if it was empty, (0) otherwise
 */
uint8_t circularBuffer_IsEmpty(CircularBuffer_TypeDef* CircularBuffer)
{
	return (circularBuffer_GetCount(CircularBuffer) == 0);
}

/**
 * @brief	Check if the buffer is full
 * @param	CircularBuffer: the buffer to check
 * @retval	(1) if it was full, (0) otherwise
 */
uint8_t circularBuffer_IsFull(CircularBuffer_TypeDef* CircularBuffer)
{
	return (circularBuffer_GetCount(CircularBuffer) == CIRCULARBUFFER_SIZE);
}
