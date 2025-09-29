/*
 * tx.c
 *
 *  Created on: Aug 31, 2025
 *      Author: rupak
 */




#include<stdio.h>
#include<string.h>
#include "stm32l476xx.h"

void USART2_IRQHandler(void);


//char msg[1024] = "HELLO WORLD FROM RUPAK \n \r  uart data is receeving \n\r";


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
    usart_gpios.GPIO_Config_t.GPIO_PinAltFunMode = 7;

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

	//this is btn gpio configuration
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



//interrupt test
char rec[5];

int main(void)
{
    GPIO_ButtonInit(); // optional, for bootloader button
   USART2_GPIOInit();
   USART2_Init();
   USART_IRQInterruptConfig(IRQ_NO_USART2,ENABLE);

  USART_PeripheralControl(USART2, ENABLE);

    // Start RX interrupt for 1 byte
    USART_REC_DATA_IT(&usart_handle, (uint8_t*)rec, 1);

    while(1)
    {

        if (rec[0] == 'A')
        {
            const char msg[] = "\nHELLO FROM INTERRUPT UART\n\r";
            USART_SEND_DATA_IT(&usart_handle, (uint8_t*)msg, sizeof(msg)-1);

           rec[0] = 0; // reset to receive next byte
            USART_REC_DATA_IT(&usart_handle, (uint8_t*)rec, 1);
        }
    }
}


void USART2_IRQHandler(void)
{
    USART_IRQHandling(&usart_handle);
}


