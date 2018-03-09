/*
 * utils.h
 *
 *  Created on: 3 Mar 2018
 *      Author: ronan
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "stm32f3xx_hal.h"

typedef enum
{
	DownFactor_1 = 1,
	DownFactor_2 = 2,
	DownFactor_4 = 4,
	DownFactor_8 = 8,
	DownFactor_16 = 16
}DownFactor_e;

void HAL_Status_Check(HAL_StatusTypeDef status);
void HAL_print_number(uint8_t* num, uint16_t size);
void HAL_printf(const char *fmt, ...);
void split_data(uint16_t* data, uint16_t data_length, uint8_t * split_data, DownFactor_e down_factor);


#endif /* UTILS_H_ */
