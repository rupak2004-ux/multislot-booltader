/*
 * flash.c
 *
 *  Created on: Aug 27, 2025
 *      Author: rupak
 */

#include "flash.h"

// Flash unlock keys
#define FLASH_KEY1            0x45670123UL
#define FLASH_KEY2            0xCDEF89ABUL

// Flags
#define SR_EOP_FLAG      FLASH_SR_EOP
#define SR_ERR_FLAGS    ( FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | \
                          FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | \
                          FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_RDERR | FLASH_SR_OPTVERR )

// Helpers for bank/page
static inline uint8_t _bank_from_page(uint32_t page)
{
    return (page < 256U) ? 0U : 1U;   // Bank 0 = first 256 pages, Bank 1 = next 256 pages
}

static inline uint32_t _page_in_bank(uint32_t page)
{
    return (page < 256U) ? page : (page - 256U);
}

// Unlock flash
uint8_t FLASH_UNLOCK(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
    return ((FLASH->CR & FLASH_CR_LOCK) == 0U) ? 1U : 0U;
}

// Lock flash
void FLASH_LOCK(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

// Clear only error flags (not EOP)
static inline void FLASH_CLEAR_ERROR_FLAGS(void)
{
    FLASH->SR = SR_ERR_FLAGS;
}

// Clear EOP
static inline void FLASH_CLEAR_EOP(void)
{
    FLASH->SR = SR_EOP_FLAG;
}

// Wait until flash operation completes
FLASH_STATUS_T FLASH_WaitForLastOperation(uint32_t timeoutLoops)
{
    // Wait for BSY to clear
    while (FLASH->SR & FLASH_SR_BSY) {
        if (timeoutLoops-- == 0U) {
            return FLASH_ERR_BUSY_TIMEOUT;
        }
    }

    uint32_t sr = FLASH->SR;

    // Check errors
    if (sr & SR_ERR_FLAGS) {
        FLASH->SR = SR_ERR_FLAGS;   // Clear them
        return FLASH_ERR_PROG;
    }

    // Clear EOP (if set)
    if (sr & SR_EOP_FLAG) {
        FLASH_CLEAR_EOP();
    }

    return FLASH_OK;
}

// Convert address → page number
uint32_t FLASH_ADDRESS_TO_PAGE(uint32_t address)
{
    if (address < FLASH_BASE_ADDR || address > FLASH_END_ADDR) {
        return FLASH_INVALID_PAGE;
    }
    return (address - FLASH_BASE_ADDR) / FLASH_PAGE_SIZE;
}

// Erase one page
FLASH_STATUS_T FLASH_ERASE_PAGE(uint32_t pageindex)
{
    if (pageindex >= FLASH_PAGES_TOTAL) return FLASH_ERR_RANGE;

    FLASH_STATUS_T st = FLASH_WaitForLastOperation(FLASH_TIMEOUT_LOOPS);
    if (st != FLASH_OK) return st;

    FLASH_CLEAR_ERROR_FLAGS();   // clear errors only

    uint8_t bank = _bank_from_page(pageindex);
    uint32_t pnb = _page_in_bank(pageindex);

    uint32_t cr = FLASH->CR;
    cr &= ~(FLASH_CR_PNB | FLASH_CR_BKER);
    cr |= FLASH_CR_PER | (pnb << FLASH_CR_PNB_Pos);
    if (bank == 1U) cr |= FLASH_CR_BKER;
    FLASH->CR = cr;

    // Start erase
    FLASH->CR |= FLASH_CR_STRT;

    st = FLASH_WaitForLastOperation(FLASH_TIMEOUT_LOOPS);

    FLASH->CR &= ~FLASH_CR_PER; // disable erase mode

    return st;
}

// Erase multiple pages
FLASH_STATUS_T FLASH_ERASE_PAGES(uint32_t first_page, uint32_t pagecount)
{
    if (pagecount == 0U) return FLASH_ERR_SIZE;
    if ((first_page + pagecount) > FLASH_PAGES_TOTAL) return FLASH_ERR_RANGE;

    for (uint32_t p = 0; p < pagecount; ++p) {
        FLASH_STATUS_T st = FLASH_ERASE_PAGE(first_page + p);
        if (st != FLASH_OK) return st;
    }
    return FLASH_OK;
}

// Program a double word (64 bits)
FLASH_STATUS_T FLASH_PROGRAM_DOUBLE_WORD(uint32_t address, uint64_t data)
{
    if (address < FLASH_BASE_ADDR || (address + 7U) > FLASH_END_ADDR)
        return FLASH_ERR_RANGE;
    if ((address & 0x7U) != 0U)
        return FLASH_ERR_ALIGNMENT;

    FLASH_STATUS_T st = FLASH_WaitForLastOperation(FLASH_TIMEOUT_LOOPS);
    if (st != FLASH_OK) return st;

    FLASH_CLEAR_ERROR_FLAGS();

    FLASH->CR |= FLASH_CR_PG;

    // Write lower 32 bits
    *(volatile uint32_t*)address = (uint32_t)(data & 0xFFFFFFFFULL);
    st = FLASH_WaitForLastOperation(FLASH_TIMEOUT_LOOPS);
    if (st != FLASH_OK) {
        FLASH->CR &= ~FLASH_CR_PG;
        return st;
    }

    // Write upper 32 bits
    *(volatile uint32_t*)(address + 4) = (uint32_t)(data >> 32);
    st = FLASH_WaitForLastOperation(FLASH_TIMEOUT_LOOPS);

    FLASH->CR &= ~FLASH_CR_PG;  // end programming

    return st;
}

// Program buffer in 64-bit chunks
FLASH_STATUS_T FLASH_PROGRAM_BUFFER_DW(uint32_t address, const void *buf, size_t lenBytes)
{
    if (address < FLASH_BASE_ADDR || (address + lenBytes - 1U) > FLASH_END_ADDR)
        return FLASH_ERR_RANGE;
    if ((address & 0x7U) != 0U)
        return FLASH_ERR_ALIGNMENT;

    const uint8_t *p = (const uint8_t *)buf;
    size_t remaining = lenBytes;

    while (remaining >= 8U) {
        uint64_t qw =
            ((uint64_t)p[0])       |
            ((uint64_t)p[1] << 8 ) |
            ((uint64_t)p[2] << 16) |
            ((uint64_t)p[3] << 24) |
            ((uint64_t)p[4] << 32) |
            ((uint64_t)p[5] << 40) |
            ((uint64_t)p[6] << 48) |
            ((uint64_t)p[7] << 56);

        FLASH_STATUS_T st = FLASH_PROGRAM_DOUBLE_WORD(address, qw);
        if (st != FLASH_OK) return st;

        address   += 8U;
        p         += 8U;
        remaining -= 8U;
    }

    // Tail bytes → pad with 0xFF
    if (remaining > 0U) {
        uint8_t tail[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
        for (size_t i = 0; i < remaining; ++i) tail[i] = p[i];

        uint64_t qw =
            ((uint64_t)tail[0])       |
            ((uint64_t)tail[1] << 8 ) |
            ((uint64_t)tail[2] << 16) |
            ((uint64_t)tail[3] << 24) |
            ((uint64_t)tail[4] << 32) |
            ((uint64_t)tail[5] << 40) |
            ((uint64_t)tail[6] << 48) |
            ((uint64_t)tail[7] << 56);

        FLASH_STATUS_T st = FLASH_PROGRAM_DOUBLE_WORD(address, qw);
        if (st != FLASH_OK) return st;
    }

    return FLASH_OK;
}
