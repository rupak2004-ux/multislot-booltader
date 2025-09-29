/*
 * flash.h
 *
 *  Created on: Aug 27, 2025
 *      Author: rupak
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "stm32l476xx.h"


//flash reltaed definition

#define FLASH_SIZE_BYTES       0x00100000UL //1MB
#define FLASH_END_ADDR         (FLASH_BASE_ADDR + FLASH_SIZE_BYTES -1)
#define FLASH_PAGE_SIZE        2048UL //2KB
#define FLASH_PAGES_TOTAL      (FLASH_SIZE_BYTES / FLASH_PAGE_SIZE) //512 PAGES
#define FLASH_BANK_SIZE        FLASH_SIZE_BYTES /2
#define FLASH_BANK1_ADDR       FLASH_BASE_ADDR
#define FLASH_BANK2_ADDR       FLASH_BASE_ADDR + FLASH_BANK_SIZE

//STATUS CODES
typedef enum
{
	FLASH_OK =0,
	FLASH_ERR_BUSY_TIMEOUT,
    FLASH_ERR_RANGE,
	FLASH_ERR_ALIGNMENT,
	FLASH_ERR_WRITE_PROTECT,
	FLASH_ERR_PROG,
	FLASH_ERR_SIZE,
	FLASH_ERR_UNKNOWN

}FLASH_STATUS_T;


#define FLASH_TIMEOUT_LOOPS   50000U
#define FLASH_INVALID_PAGE      0xFFFFFFFFUL


//flash api
uint8_t FLASH_UNLOCK(void);
void FLASH_LOCK(void);
void FLASH_CLEAR_ALL_FLAGS(void);

FLASH_STATUS_T FLASH_WaitForLastOperation(uint32_t timeoutLoops);

FLASH_STATUS_T FLASH_ERASE_PAGE(uint32_t pageindex);
FLASH_STATUS_T FLASH_ERASE_PAGES(uint32_t first_page, uint32_t pagecount);

FLASH_STATUS_T FLASH_PROGRAM_DOUBLE_WORD(uint32_t address, uint64_t data);    /* 8-byte aligned */
FLASH_STATUS_T FLASH_PROGRAM_BUFFER_DW(uint32_t address, const void *buf, size_t lenBytes);



uint32_t    FLASH_ADDRESS_TO_PAGE(uint32_t address);


#endif /* INC_FLASH_H_ */
