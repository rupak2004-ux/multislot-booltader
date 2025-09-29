/*
 * stm32l476xx.h
 *
 *  Created on: Aug 18, 2025
 *      Author: rupak
 */

#ifndef INC_STM32L476XX_H_
#define INC_STM32L476XX_H_
#include <stdint.h>
#include <stddef.h>

#define _vo volatile
#define __weak __attribute__((weak))

// processor specific details

// arm cortex m4 nvic isrex(  interrupt set ) register address

#define NVIC_ISER0      ((_vo uint32_t *) 0xE000E100)
#define NVIC_ISER1      ((_vo uint32_t *) 0xE000E104)
#define NVIC_ISER2      ((_vo uint32_t *) 0xE000E108)
#define NVIC_ISER3      ((_vo uint32_t *) 0xE000E10C)

// arm cortex m4 nvic iCrex(  interrupt set ) register address
#define NVIC_ICER0           ((_vo uint32_t*)0xE000E180)
#define NVIC_ICER1           ((_vo uint32_t*)0xE000E184)
#define NVIC_ICER2           ((_vo uint32_t*)0xE000E188)
#define NVIC_ICER3           ((_vo uint32_t*)0xE000E18C)

// NVIC PRIORITY BASE ADDRESS
#define NVIC_PR_BASE_ADDR     ((_vo uint32_t*)0xE000E400)


#define NO_PR_BITS_IMPLEMENTED   4

//arm cortex m4 specific define
#define  SCB_BASE_ADDR            0xE000E000UL


// SCB Register Map
typedef struct
{
    volatile uint32_t CPUID;    // 0x00: CPUID Base Register (RO)
    volatile uint32_t ICSR;     // 0x04: Interrupt Control and State Register (RW/RO)
    volatile uint32_t VTOR;     // 0x08: Vector Table Offset Register (RW)
    volatile uint32_t AIRCR;    // 0x0C: Application Interrupt and Reset Control Register (RW)
    volatile uint32_t SCR;      // 0x10: System Control Register (RW)
    volatile uint32_t CCR;      // 0x14: Configuration and Control Register (RW)
    volatile uint32_t SHPR1;    // 0x18: System Handler Priority Register 1 (RW)
    volatile uint32_t SHPR2;    // 0x1C: System Handler Priority Register 2 (RW)
    volatile uint32_t SHPR3;    // 0x20: System Handler Priority Register 3 (RW)
    volatile uint32_t SHCSR;    // 0x24: System Handler Control and State Register (RW)
    // More registers exist, but these are the main ones for bootloader
} SCB_RegDef;

#define SCB           ((SCB_RegDef*)SCB_BASE_ADDR)



//base address for the l476rg
#define ROM_BASE_ADDR                 0x1FFF0000U
#define FLASH_BASE_ADDR               0x08000000U
#define SRAM1_BASE_ADDR               0x20000000U
#define SRAM2_BASE_ADDR               0x20040000U
#define PERI_BASE_ADDR                0x40000000U
#define SRAM_BASE_ADDR               SRAM1_BASE_ADDR

//BASE ADDRESS OF THE BUSES THAT ARE AVAILABLE
#define APB1_BASE_ADDR             PERI_BASE_ADDR
#define APB2_BASE_ADDR              0x40010000U
#define AHB1_BASE_ADDR              0x40020000U
#define AHB2_BASE_ADDR              0x48000000U

// PERIPHERALS THAT ARE HANGING ON
/* TIMERS */
#define TIM2_BASE_ADDR     (APB1_BASE_ADDR + 0x0000U)
#define TIM3_BASE_ADDR     (APB1_BASE_ADDR + 0x0400U)
#define TIM4_BASE_ADDR     (APB1_BASE_ADDR + 0x0800U)
#define TIM5_BASE_ADDR     (APB1_BASE_ADDR + 0x0C00U)
#define TIM6_BASE_ADDR     (APB1_BASE_ADDR + 0x1000U)
#define TIM7_BASE_ADDR     (APB1_BASE_ADDR + 0x1400U)

/* LCD & RTC */
#define LCD_BASE_ADDR      (APB1_BASE_ADDR + 0x2400U)
#define RTC_BASE_ADDR      (APB1_BASE_ADDR + 0x2800U)

/* WATCHDOGS */
#define WWDG_BASE_ADDR     (APB1_BASE_ADDR + 0x2C00U)
#define IWDG_BASE_ADDR     (APB1_BASE_ADDR + 0x3000U)

/* SPI */
#define SPI2_BASE_ADDR     (APB1_BASE_ADDR + 0x3800U)
#define SPI3_BASE_ADDR     (APB1_BASE_ADDR + 0x3C00U)

/* USART / UART */
#define USART2_BASE_ADDR   (APB1_BASE_ADDR + 0x4400U)
#define USART3_BASE_ADDR   (APB1_BASE_ADDR + 0x4800U)
#define UART4_BASE_ADDR    (APB1_BASE_ADDR + 0x4C00U)
#define UART5_BASE_ADDR    (APB1_BASE_ADDR + 0x5000U)

/* (Already defined earlier in your map) */
#define I2C1_BASE_ADDR     (APB1_BASE_ADDR + 0x5400U)
#define I2C2_BASE_ADDR     (APB1_BASE_ADDR + 0x5800U)
#define I2C3_BASE_ADDR     (APB1_BASE_ADDR + 0x5C00U)
#define CAN1_BASE_ADDR     (APB1_BASE_ADDR + 0x6400U)
#define PWR_BASE_ADDR      (APB1_BASE_ADDR + 0x7000U)
#define DAC1_BASE_ADDR     (APB1_BASE_ADDR + 0x7400U)
#define OPAMP_BASE_ADDR    (APB1_BASE_ADDR + 0x7800U)
#define LPTIM1_BASE_ADDR   (APB1_BASE_ADDR + 0x7C00U)
#define LPUART1_BASE_ADDR  (APB1_BASE_ADDR + 0x8000U)
#define SWPMI1_BASE_ADDR   (APB1_BASE_ADDR + 0x8800U)
#define LPTIM2_BASE_ADDR   (APB1_BASE_ADDR + 0x9400U)

//PERIPHERAL HANGING ON THE APB2 BUS
/* SYSCFG + VREFBUF + COMP + EXTI */
#define SYSCFG_BASE_ADDR    (APB2_BASE_ADDR + 0x0000U)
#define VREFBUF_BASE_ADDR   (APB2_BASE_ADDR + 0x0030U)
#define COMP_BASE_ADDR      (APB2_BASE_ADDR + 0x0200U)
#define EXTI_BASE_ADDR      (APB2_BASE_ADDR + 0x0400U)

/* FIREWALL */
#define FIREWALL_BASE_ADDR  (APB2_BASE_ADDR + 0x1C00U)

/* SDMMC */
#define SDMMC1_BASE_ADDR    (APB2_BASE_ADDR + 0x2800U)

/* TIMERS */
#define TIM1_BASE_ADDR      (APB2_BASE_ADDR + 0x2C00U)
#define SPI1_BASE_ADDR      (APB2_BASE_ADDR + 0x3000U)
#define TIM8_BASE_ADDR      (APB2_BASE_ADDR + 0x3400U)
#define USART1_BASE_ADDR    (APB2_BASE_ADDR + 0x3800U)
#define TIM15_BASE_ADDR     (APB2_BASE_ADDR + 0x4000U)
#define TIM16_BASE_ADDR     (APB2_BASE_ADDR + 0x4400U)
#define TIM17_BASE_ADDR     (APB2_BASE_ADDR + 0x4800U)

/* SAI */
#define SAI1_BASE_ADDR      (APB2_BASE_ADDR + 0x5400U)
#define SAI2_BASE_ADDR      (APB2_BASE_ADDR + 0x5800U)

/* DFSDM */
#define DFSDM1_BASE_ADDR    (APB2_BASE_ADDR + 0x6000U)

//AHB1 BUS PERIPHERALS
/* DMA */
#define DMA1_BASE_ADDR      (AHB1_BASE_ADDR + 0x0000U)
#define DMA2_BASE_ADDR      (AHB1_BASE_ADDR + 0x0400U)

/* RCC + FLASH */
#define RCC_BASE_ADDR       (AHB1_BASE_ADDR + 0x1000U)
#define FLASH_R_BASE_ADDR   (AHB1_BASE_ADDR + 0x2000U)

/* CRC */
#define CRC_BASE_ADDR       (AHB1_BASE_ADDR + 0x3000U)

/* TSC */
#define TSC_BASE_ADDR       (AHB1_BASE_ADDR + 0x4000U)

//AHB2 BUSES PERIPHERALS
/* GPIO Ports */
#define GPIOA_BASE_ADDR      (AHB2_BASE_ADDR + 0x0000U)
#define GPIOB_BASE_ADDR      (AHB2_BASE_ADDR + 0x0400U)
#define GPIOC_BASE_ADDR      (AHB2_BASE_ADDR + 0x0800U)
#define GPIOD_BASE_ADDR      (AHB2_BASE_ADDR + 0x0C00U)
#define GPIOE_BASE_ADDR      (AHB2_BASE_ADDR + 0x1000U)
#define GPIOF_BASE_ADDR      (AHB2_BASE_ADDR + 0x1400U)
#define GPIOG_BASE_ADDR      (AHB2_BASE_ADDR + 0x1800U)
#define GPIOH_BASE_ADDR      (AHB2_BASE_ADDR + 0x1C00U)
#define GPIOI_BASE_ADDR      (AHB2_BASE_ADDR + 0x2000U)

/* OTG FS */
#define OTG_FS_BASE_ADDR     0x50000000UL  /* full 256 KB block */

/* ADC */
#define ADC_BASE_ADDR        0x50040000UL

/* AES & RNG (Crypto) */
#define AES_BASE_ADDR        0x50060000UL
#define RNG_BASE_ADDR        (0x50060000UL + 0x0800U)


//typedef struct for the gpio

typedef struct
{
    volatile uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00 */
    volatile uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04 */
    volatile uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08 */
    volatile uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C */
    volatile uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10 */
    volatile uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14 */
    volatile uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18 */
    volatile uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C */
    volatile uint32_t AFR[2];     /*!< GPIO alternate function low and highregister,  Address offset: 0x20 */
    volatile uint32_t BRR;      /*!< GPIO port bit reset register,          Address offset: 0x28 */
    volatile uint32_t ASCR;     /*!< GPIO port analog switch control reg.,  Address offset: 0x2C */
} GPIO_RegDef_t;

#define  GPIOA      ((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define  GPIOB      ((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define  GPIOC      ((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define  GPIOD      ((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define  GPIOE      ((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define  GPIOF      ((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define  GPIOG      ((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define  GPIOH      ((GPIO_RegDef_t*)GPIOH_BASE_ADDR)
#define  GPIOI      ((GPIO_RegDef_t*)GPIOI_BASE_ADDR)


// usart type def struct
typedef struct
{
    volatile uint32_t CR1;    /*!< USART Control register 1,      Address offset: 0x00 */
    volatile uint32_t CR2;    /*!< USART Control register 2,      Address offset: 0x04 */
    volatile uint32_t CR3;    /*!< USART Control register 3,      Address offset: 0x08 */
    volatile uint32_t BRR;    /*!< USART Baud rate register,      Address offset: 0x0C */
    volatile uint32_t GTPR;   /*!< USART Guard time & prescaler,  Address offset: 0x10 */
    volatile uint32_t RTOR;   /*!< USART Receiver timeout,        Address offset: 0x14 */
    volatile uint32_t RQR;    /*!< USART Request register,        Address offset: 0x18 */
    volatile uint32_t ISR;    /*!< USART Interrupt & status,      Address offset: 0x1C */
    volatile uint32_t ICR;    /*!< USART Interrupt flag clear,    Address offset: 0x20 */
    volatile uint32_t RDR;    /*!< USART Receive data register,   Address offset: 0x24 */
    volatile uint32_t TDR;    /*!< USART Transmit data register,  Address offset: 0x28 */
} USART_RegDef_t;

#define USART1         ((USART_RegDef_t*)USART1_BASE_ADDR)
#define USART2         ((USART_RegDef_t*)USART2_BASE_ADDR)
#define USART3         ((USART_RegDef_t*)USART3_BASE_ADDR)
#define UART4         ((USART_RegDef_t*)UART4_BASE_ADDR)
#define UART5         ((USART_RegDef_t*)UART5_BASE_ADDR)


//TYPEDEF STRUCT FOR THE RCC
typedef struct
{
    volatile uint32_t CR;          /*!< RCC clock control register,           Address offset: 0x00 */
    volatile uint32_t ICSCR;       /*!< Internal clock sources calibration,   Address offset: 0x04 */
    volatile uint32_t CFGR;        /*!< RCC clock configuration register,     Address offset: 0x08 */
    volatile uint32_t PLLCFGR;     /*!< RCC PLL configuration register,       Address offset: 0x0C */
    volatile uint32_t PLLSAI1CFGR; /*!< RCC PLLSAI1 configuration register,   Address offset: 0x10 */
    volatile uint32_t PLLSAI2CFGR; /*!< RCC PLLSAI2 configuration register,   Address offset: 0x14 */
    volatile uint32_t CIER;        /*!< RCC clock interrupt enable register,  Address offset: 0x18 */
    volatile uint32_t CIFR;        /*!< RCC clock interrupt flag register,    Address offset: 0x1C */
    volatile uint32_t CICR;        /*!< RCC clock interrupt clear register,   Address offset: 0x20 */
    uint32_t RESERVED0;            /*!< Reserved,                             Address offset: 0x24 */
    volatile uint32_t AHB1RSTR;    /*!< RCC AHB1 peripheral reset register,   Address offset: 0x28 */
    volatile uint32_t AHB2RSTR;    /*!< RCC AHB2 peripheral reset register,   Address offset: 0x2C */
    volatile uint32_t AHB3RSTR;    /*!< RCC AHB3 peripheral reset register,   Address offset: 0x30 */
    uint32_t RESERVED1;            /*!< Reserved,                             Address offset: 0x34 */
    volatile uint32_t APB1RSTR1;   /*!< RCC APB1 peripheral reset register 1, Address offset: 0x38 */
    volatile uint32_t APB1RSTR2;   /*!< RCC APB1 peripheral reset register 2, Address offset: 0x3C */
    volatile uint32_t APB2RSTR;    /*!< RCC APB2 peripheral reset register,   Address offset: 0x40 */
    uint32_t RESERVED2;            /*!< Reserved,                             Address offset: 0x44 */
    volatile uint32_t AHB1ENR;     /*!< RCC AHB1 peripheral clock enable reg, Address offset: 0x48 */
    volatile uint32_t AHB2ENR;     /*!< RCC AHB2 peripheral clock enable reg, Address offset: 0x4C */
    volatile uint32_t AHB3ENR;     /*!< RCC AHB3 peripheral clock enable reg, Address offset: 0x50 */
    uint32_t RESERVED3;            /*!< Reserved,                             Address offset: 0x54 */
    volatile uint32_t APB1ENR1;    /*!< RCC APB1 peripheral clock enable reg1,Address offset: 0x58 */
    volatile uint32_t APB1ENR2;    /*!< RCC APB1 peripheral clock enable reg2,Address offset: 0x5C */
    volatile uint32_t APB2ENR;     /*!< RCC APB2 peripheral clock enable reg, Address offset: 0x60 */
    uint32_t RESERVED4;            /*!< Reserved,                             Address offset: 0x64 */
    volatile uint32_t AHB1SMENR;   /*!< RCC AHB1 peripheral sleep/stop enable,Address offset: 0x68 */
    volatile uint32_t AHB2SMENR;   /*!< RCC AHB2 peripheral sleep/stop enable,Address offset: 0x6C */
    volatile uint32_t AHB3SMENR;   /*!< RCC AHB3 peripheral sleep/stop enable,Address offset: 0x70 */
    uint32_t RESERVED5;            /*!< Reserved,                             Address offset: 0x74 */
    volatile uint32_t APB1SMENR1;  /*!< RCC APB1 peripheral sleep/stop enable1,Offset: 0x78 */
    volatile uint32_t APB1SMENR2;  /*!< RCC APB1 peripheral sleep/stop enable2,Offset: 0x7C */
    volatile uint32_t APB2SMENR;   /*!< RCC APB2 peripheral sleep/stop enable,Offset: 0x80 */
    uint32_t RESERVED6;            /*!< Reserved,                             Address offset: 0x84 */
    volatile uint32_t CCIPR;       /*!< RCC peripherals independent clock cfg,Offset: 0x88 */
    uint32_t RESERVED7;            /*!< Reserved,                             Address offset: 0x8C */
    volatile uint32_t BDCR;        /*!< RCC backup domain control register,   Address offset: 0x90 */
    volatile uint32_t CSR;         /*!< RCC control/status register,          Address offset: 0x94 */
    volatile uint32_t CRRCR;       /*!< RCC clock recovery RC register,       Address offset: 0x98 */
    volatile uint32_t CCIPR2;      /*!< RCC peripherals independent clk cfg 2,Offset: 0x9C */
} RCC_RegDef_t;

#define RCC             ((RCC_RegDef_t*)RCC_BASE_ADDR)

//tyedef struct for the exti

typedef struct
{
    volatile uint32_t IMR1;     /*!< Interrupt mask register 1,     Address offset: 0x00 */
    volatile uint32_t EMR1;     /*!< Event mask register 1,         Address offset: 0x04 */
    volatile uint32_t RTSR1;    /*!< Rising trigger selection reg 1,Address offset: 0x08 */
    volatile uint32_t FTSR1;    /*!< Falling trigger selection reg1,Address offset: 0x0C */
    volatile uint32_t SWIER1;   /*!< Software interrupt event reg 1,Address offset: 0x10 */
    volatile uint32_t PR1;      /*!< Pending register 1,            Address offset: 0x14 */
    uint32_t RESERVED0[2];      /*!< Reserved,                      Address offset: 0x18-0x1C */
    volatile uint32_t IMR2;     /*!< Interrupt mask register 2,     Address offset: 0x20 */
    volatile uint32_t EMR2;     /*!< Event mask register 2,         Address offset: 0x24 */
    volatile uint32_t RTSR2;    /*!< Rising trigger selection reg 2,Address offset: 0x28 */
    volatile uint32_t FTSR2;    /*!< Falling trigger selection reg 2,Address offset: 0x2C */
    volatile uint32_t SWIER2;   /*!< Software interrupt event reg 2,Address offset: 0x30 */
    volatile uint32_t PR2;      /*!< Pending register 2,            Address offset: 0x34 */
} EXTI_RegDef_t;

#define EXTI   ((EXTI_RegDef_t *) EXTI_BASE_ADDR)


//FLASH REGDEF STRUCT
typedef struct
{
    volatile uint32_t ACR;        /*!< FLASH access control register,      Address offset: 0x00 */
    volatile uint32_t PDKEYR;     /*!< FLASH power-down key register,      Address offset: 0x04 */
    volatile uint32_t KEYR;       /*!< FLASH key register,                 Address offset: 0x08 */
    volatile uint32_t OPTKEYR;    /*!< FLASH option key register,          Address offset: 0x0C */
    volatile uint32_t SR;         /*!< FLASH status register,              Address offset: 0x10 */
    volatile uint32_t CR;         /*!< FLASH control register,             Address offset: 0x14 */
    volatile uint32_t ECCR;       /*!< FLASH ECC register,                 Address offset: 0x18 */
             uint32_t RESERVED1;  /*!< Reserved,                           Address offset: 0x1C */
    volatile uint32_t OPTR;       /*!< FLASH option register,              Address offset: 0x20 */
    volatile uint32_t PCROP1SR;   /*!< FLASH bank1 PCROP start,            Address offset: 0x24 */
    volatile uint32_t PCROP1ER;   /*!< FLASH bank1 PCROP end,              Address offset: 0x28 */
    volatile uint32_t WRP1AR;     /*!< FLASH bank1 WRP area A,             Address offset: 0x2C */
    volatile uint32_t WRP1BR;     /*!< FLASH bank1 WRP area B,             Address offset: 0x30 */
             uint32_t RESERVED2[4]; /*!< Reserved,                          Address offset: 0x34–0x40 */
    volatile uint32_t PCROP2SR;   /*!< FLASH bank2 PCROP start,            Address offset: 0x44 */
    volatile uint32_t PCROP2ER;   /*!< FLASH bank2 PCROP end,              Address offset: 0x48 */
    volatile uint32_t WRP2AR;     /*!< FLASH bank2 WRP area A,             Address offset: 0x4C */
    volatile uint32_t WRP2BR;     /*!< FLASH bank2 WRP area B,             Address offset: 0x50 */
} FLASH_REGDef;

#define FLASH         ((FLASH_REGDef * ) FLASH_R_BASE_ADDR)


//clock enable for the gpio
#define GPIOA_CLK_EN            (RCC->AHB2ENR|= (1<<0))
#define GPIOB_CLK_EN            (RCC->AHB2ENR|= (1<<1))
#define GPIOC_CLK_EN            (RCC->AHB2ENR|= (1<<2))
#define GPIOD_CLK_EN            (RCC->AHB2ENR|= (1<<3))
#define GPIOE_CLK_EN            (RCC->AHB2ENR|= (1<<4))
#define GPIOF_CLK_EN            (RCC->AHB2ENR|= (1<<5))
#define GPIOG_CLK_EN            (RCC->AHB2ENR|= (1<<6))
#define GPIOH_CLK_EN            (RCC->AHB2ENR|= (1<<7))
#define GPIOI_CLK_EN            (RCC->AHB2ENR|= (1<<8))


//CLK ENABLE FOR THE USART
#define USART1_CLK_EN           ((RCC->APB2ENR) |= (1<<14))
#define USART2_CLK_EN            ((RCC->APB1ENR1) |= (1<<17))
#define USART3_CLK_EN            ((RCC->APB1ENR1) |= (1<<18))
#define UART4_CLK_EN             ((RCC->APB1ENR1) |= (1<<19))
#define UART5_CLK_EN             ((RCC->APB1ENR1) |= (1<<20))

//CLOCK DISABLE FOR THE GPIO AND THE USART
#define GPIOA_CLK_DI            (RCC->AHB2ENR&= ~(1<<0))
#define GPIOB_CLK_DI            (RCC->AHB2ENR &= ~(1<<1))
#define GPIOC_CLK_DI            (RCC->AHB2ENR &= ~(1<<2))
#define GPIOD_CLK_DI            (RCC->AHB2ENR &= ~(1<<3))
#define GPIOE_CLK_DI            (RCC->AHB2ENR &= ~(1<<4))
#define GPIOF_CLK_DI           (RCC->AHB2ENR &= ~(1<<5))
#define GPIOG_CLK_DI            (RCC->AHB2ENR &= ~(1<<6))
#define GPIOH_CLK_DI            (RCC->AHB2ENR &= ~(1<<7))
#define GPIOI_CLK_DI            (RCC->AHB2ENR &= ~(1<<8))


#define USART1_CLK_DI           ((RCC->APB2ENR) &= ~(1<<14))
#define USART2_CLK_DI            ((RCC->APB1ENR1) &= ~(1<<17))
#define USART3_CLK_DI            ((RCC->APB1ENR1) &= ~(1<<18))
#define UART4_CLK_DI             ((RCC->APB1ENR1) &= ~(1<<19))
#define UART5_CLK_DI             ((RCC->APB1ENR1) &= ~(1<<20))

//reset for the gpio and usart

#define GPIOA_RESET()        do{  (RCC->AHB2RSTR |= (1<<0)) ; (RCC->AHB2RSTR &= ~(1<<0));} while(0)
#define GPIOB_RESET()        do{  (RCC->AHB2RSTR |= (1<<1)) ; (RCC->AHB2RSTR &= ~(1<<1));} while(0)
#define GPIOC_RESET()        do{  (RCC->AHB2RSTR |= (1<<2)) ; (RCC->AHB2RSTR &= ~(1<<2));} while(0)
#define GPIOD_RESET()        do{  (RCC->AHB2RSTR |= (1<<3)) ; (RCC->AHB2RSTR &= ~(1<<3));} while(0)
#define GPIOE_RESET()        do{  (RCC->AHB2RSTR |= (1<<4)) ; (RCC->AHB2RSTR &= ~(1<<4));} while(0)
#define GPIOF_RESET()        do{  (RCC->AHB2RSTR |= (1<<5)) ; (RCC->AHB2RSTR &= ~(1<<5));} while(0)
#define GPIOG_RESET()        do{  (RCC->AHB2RSTR |= (1<<6)) ; (RCC->AHB2RSTR &= ~(1<<6));} while(0)
#define GPIOH_RESET()        do{  (RCC->AHB2RSTR |= (1<<7)) ; (RCC->AHB2RSTR &= ~(1<<7));} while(0)
#define GPIOI_RESET()        do{  (RCC->AHB2RSTR |= (1<<8)) ; (RCC->AHB2RSTR &= ~(1<<8));} while(0)


#define USART1_RESET()       do{  (RCC->APB2RSTR |= (1<<14)) ; (RCC->APB2RSTR &= ~(1<<14));} while(0)
#define USART2_RESET()       do{  (RCC->APB1RSTR1 |= (1<<17)) ; (RCC->APB1RSTR1 &= ~(1<<17));} while(0)
#define USART3_RESET()       do{  (RCC->APB1RSTR1 |= (1<<18)) ; (RCC->APB1RSTR1 &= ~(1<<18));} while(0)
#define UART4_RESET()       do{  (RCC->APB1RSTR1 |= (1<<19)) ; (RCC->APB1RSTR1 &= ~(1<<19));} while(0)
#define UART5_RESET()       do{  (RCC->APB1RSTR1 |= (1<<20)) ; (RCC->APB1RSTR1 &= ~(1<<20));} while(0)

//ENABLE,SET
#define SET   1
#define RESET  0
#define ENABLE  SET
#define DISABLE  RESET
#define HIGH  SET
#define LOW   RESET
#define FLAG_SET  SET
#define FLAG_RESET   RESET
#define false      RESET
#define true     SET

/* FLASH_SR bits */
#define FLASH_SR_EOP        (1U << 0)   // End of operation
#define FLASH_SR_OPERR      (1U << 1)   // Operation error
#define FLASH_SR_PROGERR    (1U << 3)   // Programming error
#define FLASH_SR_WRPERR     (1U << 4)   // Write protection error
#define FLASH_SR_PGAERR     (1U << 5)   // Programming alignment error
#define FLASH_SR_SIZERR     (1U << 6)   // Size error
#define FLASH_SR_PGSERR     (1U << 7)   // Programming sequence error
#define FLASH_SR_MISERR     (1U << 8)   // Fast programming data miss error
#define FLASH_SR_FASTERR    (1U << 9)   // Fast programming error
#define FLASH_SR_RDERR      (1U << 14)  // Read protection error (PCROP / RDP)
#define FLASH_SR_OPTVERR    (1U << 15)  // Option validity error
#define FLASH_SR_BSY        (1U << 16)


/* FLASH_CR bit definitions for STM32L476 */
#define FLASH_CR_PG              (1U << 0)       /* Programming */
#define FLASH_CR_PER             (1U << 1)       /* Page Erase */
#define FLASH_CR_MER1            (1U << 2)       /* Bank1 Mass Erase */
#define FLASH_CR_PNB_Pos         3U
#define FLASH_CR_PNB_Msk         (0xFFU << FLASH_CR_PNB_Pos)   /* 8-bit page number field */
#define FLASH_CR_PNB             FLASH_CR_PNB_Msk
#define FLASH_CR_BKER            (1U << 11)      /* Bank Erase */
#define FLASH_CR_MER2            (1U << 15)      /* Bank2 Mass Erase */
#define FLASH_CR_STRT            (1U << 16)      /* Start Erase */
#define FLASH_CR_OPTSTRT         (1U << 17)      /* Options modification start */
#define FLASH_CR_FSTPG           (1U << 18)      /* Fast programming */
#define FLASH_CR_EOPIE           (1U << 24)      /* End of prog interrupt */
#define FLASH_CR_ERRIE           (1U << 25)      /* Error interrupt */
#define FLASH_CR_RDERRIE         (1U << 26)      /* PCROP read error interrupt */
#define FLASH_CR_OBL_LAUNCH      (1U << 27)      /* Option byte load launch */
#define FLASH_CR_OPTLOCK         (1U << 30)      /* Options Lock */
#define FLASH_CR_LOCK            (1U << 31)      /* FLASH control register lock */


#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


#include "stm32l476xx_gpio.h"
#include "stm32l476xx_rcc.h"
#include "stm32l476xx_usart.h"
#include "flash.h"
#include "bootloader.h"

#endif /* INC_STM32L476XX_H_ */
