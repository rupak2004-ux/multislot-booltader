/*
 * tx.c
 *
 *  Created on: Aug 31, 2025
 *      Author: rupak
 */


/*
 * uart_tx.c
 *
 *  Created on: Jan 22, 2019
 *      Author: admin
 */

#include<stdio.h>
#include<string.h>
#include "stm32l476xx.h"

char msg[1024] = "HELLO WORLD FROM RUPAK \n \r  uart data is receeving \n\r";

USART_Handle_t usart2_handle;


void USART2_Init(void)
{
    usart2_handle.pUSARTx = USART2;
    usart2_handle.USART_Config_t.USART_BAUD_RATE = USART_STD_BAUD_115200;
    usart2_handle.USART_Config_t.USART_HW_FLOW_CTRL = USART_HW_FLOW_CTRL_NONE;
    usart2_handle.USART_Config_t.USART_MODE = USART_MODE_ONLY_TX;
    usart2_handle.USART_Config_t.USART_STOP_BITS = USART_STOP_BIT_1;
    usart2_handle.USART_Config_t.USART_WORD_LEN = USART_WORDLEN_8BITS;
    usart2_handle.USART_Config_t.USART_PARITY = USART_PARITY_DISABLE;
    USART_INIT(&usart2_handle);
}

//void USART2_GPIOInit(void)
//{
//    GPIO_Handle_t usart_gpios;
//
//    usart_gpios.pGPIOx = GPIOA;
//    usart_gpios.GPIO_Config_t.GPIO_PinMode = GPIO_MODE_ALTERNATE;
//    usart_gpios.GPIO_Config_t.GPIO_PinOPType = GPIO_TYPE_PUSH_PULL;
//    usart_gpios.GPIO_Config_t.GPIO_PinPuPdControl = GPIO_PUPD_PULLUP;
//    usart_gpios.GPIO_Config_t.GPIO_PinSpeed = GPIO_SPEED_HIGH_SPEED;
//    usart_gpios.GPIO_Config_t.GPIO_PinAltFunMode =7;
//
//    //USART2 TX
//    usart_gpios.GPIO_Config_t.GPIO_PinNumber  = GPIO_PIN_NO_2;
//    GPIO_INIT(&usart_gpios);
//
//    //USART2 RX
//    usart_gpios.GPIO_Config_t.GPIO_PinNumber = GPIO_PIN_NO_3;
//    GPIO_INIT(&usart_gpios);
//}

//void GPIO_ButtonInit(void)
//{
//	GPIO_Handle_t GPIOBtn,GpioLed;
//
//	//this is btn gpio configuration
//	GPIOBtn.pGPIOx = GPIOC;
//	GPIOBtn.GPIO_Config_t.GPIO_PinNumber = GPIO_PIN_NO_13;
//	GPIOBtn.GPIO_Config_t.GPIO_PinMode = GPIO_MODE_INPUT;
//	GPIOBtn.GPIO_Config_t.GPIO_PinSpeed = GPIO_SPEED_HIGH_SPEED;
//	GPIOBtn.GPIO_Config_t.GPIO_PinPuPdControl = GPIO_PUPD_NO;
//
//	GPIO_INIT(&GPIOBtn);
//
//	//this is led gpio configuration
//	GpioLed.pGPIOx = GPIOA;
//	GpioLed.GPIO_Config_t.GPIO_PinNumber = GPIO_PIN_NO_5;
//	GpioLed.GPIO_Config_t.GPIO_PinMode = GPIO_MODE_OUTPUT;
//	GpioLed.GPIO_Config_t.GPIO_PinSpeed = GPIO_SPEED_HIGH_SPEED;
//	GpioLed.GPIO_Config_t.GPIO_PinOPType = GPIO_TYPE_PUSH_PULL;
//	GpioLed.GPIO_Config_t.GPIO_PinPuPdControl = GPIO_PUPD_NO;
//
//	GPIO_INIT(&GpioLed);
//
//}

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main(void)
{

//	GPIO_ButtonInit();
//
//	USART2_GPIOInit();

	USART2_Init();

    USART_PeripheralControl(USART2,ENABLE);


    while(1)
    {
		//wait till button is pressed
//		while( GPIO_READ_INPUT_PIN(GPIOC,GPIO_PIN_NO_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//USART_SEND_DATA(&usart2_handle,(uint8_t*)msg,strlen(msg));

    }

	return 0;
}
