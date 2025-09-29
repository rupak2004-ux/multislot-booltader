/*
 * stm32l476xx_rcc.h
 *
 *  Created on: Aug 21, 2025
 *      Author: rupak
 */

#ifndef INC_STM32L476XX_RCC_H_
#define INC_STM32L476XX_RCC_H_

#include "stm32l476xx.h"
#include <stdint.h>

uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);



#endif /* INC_STM32L476XX_RCC_H_ */
