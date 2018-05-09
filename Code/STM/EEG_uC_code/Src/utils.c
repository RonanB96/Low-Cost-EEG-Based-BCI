/*
 * utils.c
 *	Various user created functions
 *  Created on: 3 Mar 2018
 *      Author: Ronan Byrne
 *		Last Updated: 09/05/2018
 */

#include "utils.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define PRINT_BUFFER_SIZE 		256


extern UART_HandleTypeDef huart1;

/* Handles errors from HAL functions */
void HAL_Status_Check(HAL_StatusTypeDef status)
{
	if(status != HAL_OK)
	{
		Error_Handler();
	}
}

/* Prints raw bytes */
void HAL_print_raw(uint8_t * byte, uint16_t size)
{
	while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
	HAL_Status_Check(HAL_UART_Transmit_DMA(&huart1,byte, size));
}

/* Format string from arg's given and print over USART */
void HAL_printf_valist(const char *fmt, va_list argp) {
  char string[PRINT_BUFFER_SIZE] ={0};
  /* Prepending 's' to differentiate between string print and raw print */
  string[0] = 's';

  /* Wait until USART is ready */
  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
  /* If string isn't empty, print over USART */
  if (vsnprintf(string+1,PRINT_BUFFER_SIZE-2, fmt, argp) > 0)
  {
	  /* Append '\n' */
	  string[strlen(string)]='\n';
	  HAL_Status_Check(HAL_UART_Transmit_DMA(&huart1,(uint8_t*)&string, strlen(string)));
  }
  else
  {
	  HAL_Status_Check(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)"E - Print\n", 14));
  }
}

/* Find args and format, then send over USART*/
void HAL_printf(const char *fmt, ...) {
  va_list argp;

  va_start(argp, fmt);
  HAL_printf_valist(fmt, argp);
  va_end(argp);
}
