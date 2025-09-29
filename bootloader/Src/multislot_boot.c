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

volatile uint8_t g_ch_key;
volatile uint8_t g_un_key;



typedef enum
{
	APP1 =1,
	FACTORY_APP,
}SYS_APPS;

extern void USART2_GPIOInit(void);
extern void USART2_Init(void);
extern USART_Handle_t usart_handle;


static void proces_bt_cmd(SYS_APPS curr_addr);
static void usart_callback(void);
//USART_Handle_t usart_handle;




//void USART2_Init(void)
//{
//	usart_handle.pUSARTx = USART2;
//	usart_handle.USART_Config_t.USART_BAUD_RATE = USART_STD_BAUD_115200;
//	usart_handle.USART_Config_t.USART_HW_FLOW_CTRL = USART_HW_FLOW_CTRL_NONE;
//	usart_handle.USART_Config_t.USART_MODE = USART_MODE_TXRX;
//	usart_handle.USART_Config_t.USART_STOP_BITS = USART_STOP_BIT_1;
//	usart_handle.USART_Config_t.USART_WORD_LEN = USART_WORDLEN_8BITS;
//	usart_handle.USART_Config_t.USART_PARITY = USART_PARITY_DISABLE;
//    USART_INIT(&usart_handle);
//}
//
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

int btn_state(void)
{
	if(GPIO_READ_INPUT_PIN(GPIOC, GPIO_PIN_NO_13) == 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void delays(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}



//char msg[100] = "BTN IS PRESSED\r\n";
char menu[] = "     \r\n  BOOTLOADER MENU          \r\n"
		      "     \rAvialable Commands:\r\n         "
	          "   \r1  ==>   Run APP1\r\n"
			  "  \r F  ==> Factory App2\r \n"
			  "  \rAny key ==> Run Default App\r\n";



const char msg_app1[]    = "APP1 SELECTED \r\n";
const char msg_factory[] = "FACTORY_APP SELECTED \r\n";
const char msg_unknown[] = "UNKNOWN CMD\r\n";



char rec[5];

int main(void)
{
   GPIO_ButtonInit(); // optional, for bootloader button
   USART2_GPIOInit();
   USART2_Init();
  USART_IRQInterruptConfig(IRQ_NO_USART2,ENABLE);
   USART_PeripheralControl(USART2, ENABLE);




   if(btn_state())
   {

  USART_SEND_DATA(&usart_handle, (uint8_t*)menu, strlen(menu));



 USART_REC_DATA_IT(&usart_handle, (uint8_t*)&g_ch_key, 1);

while (1)
{
if (g_un_key != 0)   // process only if new key received
 {
   proces_bt_cmd((SYS_APPS)g_un_key);

 g_un_key = 0;    // reset after processing
 }

}
   }
   else
   {
	  //jmp_to_default();
	 jump_to_app(DEFAULT_APP_ADDRESS);
   }

while(1)
{

}
}


void USART2_IRQHandler(void)
{
    USART_IRQHandling(&usart_handle);
}
static void proces_bt_cmd(SYS_APPS curr_addr)
{
//	 char msg[] = "\nAPP1 SELECTED \r\n";
//	 char msg1[] = "\nFACTORY_APP SELECTED \r\n";
//	 char msg2[] = "UNKNOWN CMD\r\n";

  switch(curr_addr)
  {
  case APP1:
	  USART_SEND_DATA_IT(&usart_handle, (uint8_t*)msg_app1, strlen(msg_app1));
	  jump_to_app(APP1_ADDRESS);
	  break;
  case FACTORY_APP:
	  USART_SEND_DATA_IT(&usart_handle, (uint8_t*)msg_factory, strlen(msg_factory));
	  jump_to_app(FACTORY_APP_ADDRESS);
	  break;
  case 0XFF:
	  jump_to_app(DEFAULT_APP_ADDRESS);
	  break;
  }
}


static void usart_callback(void)
{
    if (g_ch_key == '1') {
        g_un_key = APP1;
    }
    else if ((g_ch_key == 'f') || (g_ch_key == 'F')) {
        g_un_key = FACTORY_APP;
    }
    else {
    g_un_key = 0XFF;
    }

    // Rearm reception
    USART_REC_DATA_IT(&usart_handle, (uint8_t*)&g_ch_key, 1);
}
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t event)
{
    if(event == USART_EVENT_RX_CMPLT) // or whatever macro your driver uses
    {
        usart_callback();
    }
}



