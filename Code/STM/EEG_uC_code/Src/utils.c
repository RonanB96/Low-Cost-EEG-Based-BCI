/*
 * utils.c
 *
 *  Created on: 3 Mar 2018
 *      Author: ronan
 */

#include "utils.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define UART_NON_BLOCKING		1
#define PRINT_BUFFER_SIZE 		256
#define UART_BLOCKING_TIMEOUT	200


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
void HAL_print_raw(uint8_t* byte, uint16_t size)
{
	while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
#if UART_NON_BLOCKING
	HAL_Status_Check(HAL_UART_Transmit_DMA(&huart1,byte, size));
#else
	HAL_Status_Check(HAL_UART_Transmit(&huart1,(byte, size, UART_BLOCKING_TIMEOUT));
#endif
}

/** Custom printf function in order to use HAL_UART_Transmit()
 * @param *fmt String to print
 * @param argp Parameters list
 */
void HAL_printf_valist(const char *fmt, va_list argp) {
  char string[PRINT_BUFFER_SIZE] ={0};
  /* Adding s to differentiate between string print and raw print */
  string[0] = 's';

  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
  if (vsnprintf(string+1,PRINT_BUFFER_SIZE-2, fmt, argp) > 0)
  {
	  string[strlen(string)]='\n';
#if UART_NON_BLOCKING
	  HAL_Status_Check(HAL_UART_Transmit_DMA(&huart1,(uint8_t*)&string, strlen(string))); // send message via UART
  }
  else
  {
	  HAL_Status_Check(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)"E - Print\n", 14));
  }
#else
	  HAL_Status_Check(HAL_UART_Transmit(&huart1,(uint8_t*)&string, strlen(string), UART_BLOCKING_TIMEOUT)); // send message via UART
  }
  else
  {
	  HAL_Status_Check(HAL_UART_Transmit(&huart1, (uint8_t*)"E - Print\n", 14, UART_BLOCKING_TIMEOUT));
  }
#endif
}

/** Custom printf function, only translate to va_list arg HAL_UART.
 * @param *fmt String to print
 * @param ... Data
 */
void HAL_printf(const char *fmt, ...) {
  va_list argp;

  va_start(argp, fmt);
  HAL_printf_valist(fmt, argp);
  va_end(argp);
}
