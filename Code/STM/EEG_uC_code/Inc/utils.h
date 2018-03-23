/*
 * utils.h
 *
 *  Created on: 3 Mar 2018
 *      Author: ronan
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "stm32f3xx_hal.h"

void HAL_Status_Check(HAL_StatusTypeDef status);
void HAL_print_raw(uint8_t* byte, uint16_t size);
void HAL_printf(const char *fmt, ...);

#endif /* UTILS_H_ */
