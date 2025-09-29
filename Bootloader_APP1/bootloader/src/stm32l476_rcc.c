#include "stm32l476xx.h"

// Prescaler lookup tables
uint16_t AHB_PreScaler[8]  = {2,4,8,16,64,128,256,512};
uint8_t  APB_PreScaler[4]  = {2,4,8,16};

#define HSE_VALUE 8000000U


static uint32_t RCC_GetMSIClock(void)
{

    uint32_t msirange = (RCC->CR >> 4) & 0xF;

    switch(msirange)
    {
        case 0:  return 100000;   // 100 kHz
        case 1:  return 200000;   // 200 kHz
        case 2:  return 400000;   // 400 kHz
        case 3:  return 800000;   // 800 kHz
        case 4:  return 1000000;  // 1 MHz
        case 5:  return 2000000;  // 2 MHz
        case 6:  return 4000000;  // 4 MHz
        case 7:  return 8000000;  // 8 MHz
        case 8:  return 16000000; // 16 MHz
        case 9:  return 24000000; // 24 MHz
        case 10: return 32000000; // 32 MHz
        case 11: return 48000000; // 48 MHz
        default: return 4000000;  // default safe fallback
    }
}

 uint32_t RCC_GetPLLOutputClock(void)
{
    uint32_t pll_src, pll_m, pll_n, pll_r;
    uint32_t pll_input, pll_vco;

    pll_src = (RCC->PLLCFGR >> 0) & 0x3;   // PLLSRC bits [1:0]
    pll_m   = ((RCC->PLLCFGR >> 4) & 0x7) + 1;  // PLLM (÷1..÷8)
    pll_n   = (RCC->PLLCFGR >> 8) & 0x7F;       // PLLN (8..86)
    pll_r   = (((RCC->PLLCFGR >> 25) & 0x3) + 1) * 2; // PLLR (÷2,4,6,8)

    // PLL input clock
    switch(pll_src)
    {
        case 1: pll_input = 16000000; break;         // HSI16
        case 2: pll_input = HSE_VALUE; break;        // HSE
        case 3: pll_input = RCC_GetMSIClock(); break; // MSI
        default: return 0; // No source
    }

    pll_vco = (pll_input / pll_m) * pll_n;   // VCO frequency
    return pll_vco / pll_r;                  // PLLR output
}


uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t SystemClock, pclk1;
    uint8_t ahbp, apb1p;

    uint8_t clk_src = (RCC->CFGR >> 2) & 0x3; // SWS bits

    switch(clk_src)
    {
        case 0: SystemClock = RCC_GetMSIClock(); break;
        case 1: SystemClock = 16000000; break; // HSI16
        case 2: SystemClock = HSE_VALUE; break;
        case 3: SystemClock = RCC_GetPLLOutputClock(); break;
        default: SystemClock = 4000000; break;
    }

    // AHB prescaler
    uint8_t tmp = (RCC->CFGR >> 4) & 0xF;
    if(tmp < 8) ahbp = 1;
    else ahbp = AHB_PreScaler[tmp-8];

    // APB1 prescaler
    tmp = (RCC->CFGR >> 10) & 0x7;
    if(tmp < 4) apb1p = 1;
    else apb1p = APB_PreScaler[tmp-4];

    pclk1 = (SystemClock / ahbp) / apb1p;
    return pclk1;
}


uint32_t RCC_GetPCLK2Value(void)
{
    uint32_t SystemClock, pclk2;
    uint8_t ahbp, apb2p;

    uint8_t clk_src = (RCC->CFGR >> 2) & 0x3;

    switch(clk_src)
    {
        case 0: SystemClock = RCC_GetMSIClock(); break;
        case 1: SystemClock = 16000000; break;
        case 2: SystemClock = HSE_VALUE; break;
        case 3: SystemClock = RCC_GetPLLOutputClock(); break;
        default: SystemClock = 4000000; break;
    }

    // AHB prescaler
    uint8_t tmp = (RCC->CFGR >> 4) & 0xF;
    if(tmp < 8) ahbp = 1;
    else ahbp = AHB_PreScaler[tmp-8];

    // APB2 prescaler
    tmp = (RCC->CFGR >> 11) & 0x7;
    if(tmp < 4) apb2p = 1;
    else apb2p = APB_PreScaler[tmp-4];

    pclk2 = (SystemClock / ahbp) / apb2p;
    return pclk2;
}
