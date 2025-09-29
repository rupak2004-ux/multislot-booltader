/*
 * bootloader.h
 *
 *  Created on: Aug 30, 2025
 *      Author: rupak
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_
#include "stm32l476xx.h"


#define APPLICATION_ADDR     0X08008000UL
#define VECTOR_TAB_BASE_ADDR    FLASH_BASE_ADDR
#define VECTOR_TAB_OFFSET       0xC000

#define MSP_VERIFY_MASK			    0x2FFE0000
#define EMPTY_MEM					0xFFFFFFFF
#define MEM_CHECKK_V2
#define MEM_CHECKK_V1

#define SECTOR0_BASE_ADDRESS		0x08000000 /*Bootloader sector*/
#define SECTOR1_BASE_ADDRESS		0x08004000 /*Default APP sector*/
#define SECTOR2_BASE_ADDRESS		0x08008000 /*APP1 sector*/
#define SECTOR3_BASE_ADDRESS		0x0800C000 /*Factory App sector*/

#define DEFAULT_APP_ADDRESS		SECTOR1_BASE_ADDRESS
#define APP1_ADDRESS			SECTOR2_BASE_ADDRESS
#define FACTORY_APP_ADDRESS		SECTOR3_BASE_ADDRESS

void USART2_GPIOInit(void);
void USART2_Init(void);
void USART2_GPIOInit(void);
typedef void(*func_ptr)(void);
void jump_to_app(uint32_t app_addr);
void jmp_to_default(void);



void SystemInit(void);



#endif /* INC_BOOTLOADER_H_ */
