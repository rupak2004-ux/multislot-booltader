/*
 * multislot_boot.c
 *
 *  Created on: Sep 8, 2025
 *      Author: rupak
 */


#include "stm32l476xx.h"
#include<stdio.h>
#include<string.h>


void USART2_IRQHandler(void);






//extern void USART2_GPIOInit(void);
//extern void USART2_Init(void);
//extern USART_Handle_t usart_handle;



USART_Handle_t usart_handle;




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

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn,GpioLed;

	//this is btn gpio configurationp
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_Config_t.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_Config_t.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOBtn.GPIO_Config_t.GPIO_PinSpeed = GPIO_SPEED_HIGH_SPEED;
	GPIOBtn.GPIO_Config_t.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	GPIO_INIT(&GPIOBtn);

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_Config_t.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_Config_t.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_Config_t.GPIO_PinSpeed = GPIO_SPEED_HIGH_SPEED;
	GpioLed.GPIO_Config_t.GPIO_PinOPType = GPIO_TYPE_PUSH_PULL;
	GpioLed.GPIO_Config_t.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	GPIO_INIT(&GpioLed);

}



void delays(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


char msg[] = "FACTORY APP is running \n\r";





int main(void)
{
//   GPIO_ButtonInit(); // optional, for bootloader button
//   USART2_GPIOInit();
   USART2_Init();
 // USART_IRQInterruptConfig(IRQ_NO_USART2,ENABLE);
   USART_PeripheralControl(USART2, ENABLE);



while (1)
{
	 USART_SEND_DATA(&usart_handle, (uint8_t*)msg, strlen(msg));


}

 }



void USART2_IRQHandler(void)
{
    USART_IRQHandling(&usart_handle);
}

void SystemInit(void)
{

    SCB->VTOR = 0x0800C000;
}





