/*
 * bootloader.c
 *
 *  Created on: Aug 30, 2025
 *      Author: rupak
 */

#include "bootloader.h"
#include <string.h>


char msg1[100] ="application working\r\n";
char msg2[100] = "not working";

static void delay(uint32_t value)
{
	for(uint32_t i=0;i<value;i++);
}

 void USART2_GPIOInit(void);
 void USART2_Init(void);
 USART_Handle_t usart_handle;

 extern void USART2_IRQHandler(void);


void USART2_GPIOInit(void)
{
    GPIO_Handle_t usart_gpios;

    usart_gpios.pGPIOx = GPIOA;
    usart_gpios.GPIO_Config_t.GPIO_PinMode = GPIO_MODE_ALTERNATE;
    usart_gpios.GPIO_Config_t.GPIO_PinOPType = GPIO_TYPE_PUSH_PULL;
    usart_gpios.GPIO_Config_t.GPIO_PinPuPdControl = GPIO_PUPD_PULLUP;
    usart_gpios.GPIO_Config_t.GPIO_PinSpeed = GPIO_SPEED_HIGH_SPEED;
    usart_gpios.GPIO_Config_t.GPIO_PinAltFunMode =7;

    //USART2 TX
    usart_gpios.GPIO_Config_t.GPIO_PinNumber  = GPIO_PIN_NO_2;
    GPIO_INIT(&usart_gpios);

    //USART2 RX
    usart_gpios.GPIO_Config_t.GPIO_PinNumber = GPIO_PIN_NO_3;
    GPIO_INIT(&usart_gpios);
}
void USART2_Init(void)
{
	usart_handle.pUSARTx = USART2;
	usart_handle.USART_Config_t.USART_BAUD_RATE = USART_STD_BAUD_115200;
	usart_handle.USART_Config_t.USART_HW_FLOW_CTRL = USART_HW_FLOW_CTRL_NONE;
	usart_handle.USART_Config_t.USART_MODE = USART_MODE_TXRX;
	usart_handle.USART_Config_t.USART_STOP_BITS = USART_STOP_BIT_1;
	usart_handle.USART_Config_t.USART_WORD_LEN = USART_WORDLEN_8BITS;
	usart_handle.USART_Config_t.USART_PARITY = USART_PARITY_DISABLE;
    USART_INIT(&usart_handle);
}



__attribute__((always_inline)) static inline void __set_MSP(uint32_t topOfMainStack)
{
    __asm volatile ("MSR msp, %0" : : "r" (topOfMainStack) : );
}


void jmp_to_default(void)
{
    uint32_t app_msp;
    uint32_t app_reset_handler;
    func_ptr jump_to_app;

    // small delay
    delay(300);

    // Validate application presence / MSP first word
    app_msp = *((uint32_t *)APPLICATION_ADDR);          // first word: initial MSP
    app_reset_handler = *((uint32_t *)(APPLICATION_ADDR + 4)); // second word: reset handler addr

    // Basic sanity check: MSP must be SRAM region (0x2000 0000 .. 0x2004 FFFF )
    if ( (app_msp & 0x2FFE0000U) == 0x20000000U && (app_reset_handler != 0xFFFFFFFFU) )
    {
        // notify
    	USART2_GPIOInit();
    	USART2_Init();
    	//USART_IRQInterruptConfig(IRQ_NO_USART2,ENABLE);
    	USART_PeripheralControl(USART2,ENABLE);




        USART_SEND_DATA(&usart_handle, (uint8_t*)msg1, strlen(msg1));

        // 1) Set application's vector table (so interrupts use app vectors)
        SCB->VTOR = APPLICATION_ADDR;



        // 3) Set Main Stack Pointer from application's vector[0]
        __set_MSP(app_msp);

        // 4) Jump to application's reset handler (address may be Thumb, ensure LSB = 1)
        jump_to_app = (func_ptr)(app_reset_handler | 1U); // ensure proper address
        // Use branch to function pointer - now executing in app context
        jump_to_app();


    }
    else
    {
        // application not present or invalid
        USART_SEND_DATA(&usart_handle, (uint8_t*)msg2, strlen(msg2));
    }
}


void jump_to_app(uint32_t app_addr)
{
    uint32_t app_msp;
    uint32_t app_reset_handler;
    func_ptr jump_to_app;

    // small delay
    delay(300);

    // Validate application presence / MSP first word
    app_msp = *((uint32_t *)app_addr);          // first word: initial MSP
    app_reset_handler = *((uint32_t *)(app_addr + 4)); // second word: reset handler addr

    // Basic sanity check: MSP must be SRAM region (0x2000 0000 .. 0x2004 FFFF depending on device)
    if ( (app_msp & 0x2FFE0000U) == 0x20000000U && (app_reset_handler != 0xFFFFFFFFU) )
    {
        // notify (optional)
    	USART2_GPIOInit();
    	USART2_Init();
    	//USART_IRQInterruptConfig(IRQ_NO_USART2,ENABLE);
    	USART_PeripheralControl(USART2,ENABLE);




        USART_SEND_DATA(&usart_handle, (uint8_t*)msg1, strlen(msg1));

        // 1) Set application's vector table (so interrupts use app vectors)
        SCB->VTOR = app_addr;



        // 3) Set Main Stack Pointer from application's vector[0]
        __set_MSP(app_msp);

        // 4) Jump to application's reset handler (address may be Thumb, ensure LSB = 1)
        jump_to_app = (func_ptr)(app_reset_handler | 1U); // ensure proper address
        // Use branch to function pointer - now executing in app context
        jump_to_app();


    }
    else
    {
        // application not present or invalid
        USART_SEND_DATA(&usart_handle, (uint8_t*)msg2, strlen(msg2));
    }
}

//void SystemInit(void)
//{
//	SCB->VTOR = VECTOR_TAB_BASE_ADDR|VECTOR_TAB_OFFSET;
//}
