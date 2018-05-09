/*
 * utils.h
 *	Header file for various user created functions
 *  Created on: 3 Mar 2018
 *      Author: Ronan Byrne
 *		Last Updated: 09/05/2018
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "stm32f3xx_hal.h"

/**
  * @brief  This function checks the error status of HAL functions, if an error is received,
  * we enter the error handler
  * @param  status: HAL_StatusTypeDef error returned from HAL function
  * @retval None
  */
void HAL_Status_Check(HAL_StatusTypeDef status);
/**
  * @brief  This function sends the bytes given over USART without prepending or appending anything
  * @param  byte: Pointer to byte array with value to print
  * @param  size: Number of bytes to send
  * @retval None
  */
void HAL_print_raw(uint8_t* byte, uint16_t size);
/**
  * @brief  This function works as a standard printf sending the string over USART,
  * It prepends an 's' to signify it is a string and appends an '\n'
  * @param  fmt: Format of string to send
  * @retval None
  */
void HAL_printf(const char *fmt, ...);

/* Struct for managing circular ring buffer */
typedef struct{
	uint8_t * buffer;
	uint16_t head;
	uint16_t tail;
	size_t size;
	uint8_t wrapped;
}circle_buff_s;

#endif /* UTILS_H_ */
