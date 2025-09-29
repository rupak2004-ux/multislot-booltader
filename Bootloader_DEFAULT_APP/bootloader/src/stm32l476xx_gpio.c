/*
 * stm32l476xx_gpio.c
 *
 *  Created on: Aug 21, 2025
 *      Author: rupak
 */


#include "stm32l476xx_gpio.h"



//init and deinit apii
void GPIO_INIT(GPIO_Handle_t *pGPIOhandle)
{   //clk enable for the gpio
	GPIO_PERI_CTRL(pGPIOhandle->pGPIOx, ENABLE);

uint32_t tempreg;

if(pGPIOhandle->GPIO_Config_t.GPIO_PinMode <= GPIO_MODE_ANALOG )
{
	//non interrupt mode
	tempreg = (pGPIOhandle->GPIO_Config_t.GPIO_PinMode <<  (2 * pGPIOhandle->GPIO_Config_t.GPIO_PinNumber));
	//clear the register
	pGPIOhandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOhandle->GPIO_Config_t.GPIO_PinNumber));
	pGPIOhandle->pGPIOx->MODER |= tempreg;
}
else
{
	//interrupt mode
	if(pGPIOhandle->GPIO_Config_t.GPIO_PinMode == GPIO_MODE_IT_FT)
	{
		EXTI->FTSR1 |= (1 << pGPIOhandle->GPIO_Config_t.GPIO_PinNumber)  ;
		EXTI->RTSR1 &= ~(1 << pGPIOhandle->GPIO_Config_t.GPIO_PinNumber)  ;
	}
	if(pGPIOhandle->GPIO_Config_t.GPIO_PinMode == GPIO_MODE_IT_RT)
	{
		EXTI->FTSR1 &= ~(1 << pGPIOhandle->GPIO_Config_t.GPIO_PinNumber)  ;
				EXTI->RTSR1 |= (1 << pGPIOhandle->GPIO_Config_t.GPIO_PinNumber)  ;
	}

	if(pGPIOhandle->GPIO_Config_t.GPIO_PinMode == GPIO_MODE_IT_RFT)
	{

		EXTI->FTSR1 |= (1 << pGPIOhandle->GPIO_Config_t.GPIO_PinNumber)  ;
				EXTI->RTSR1 |= (1 << pGPIOhandle->GPIO_Config_t.GPIO_PinNumber)  ;
	}
}

//configure the output type
tempreg = 0;
tempreg = (pGPIOhandle->GPIO_Config_t.GPIO_PinSpeed << (2 * pGPIOhandle->GPIO_Config_t.GPIO_PinNumber));
pGPIOhandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOhandle->GPIO_Config_t.GPIO_PinNumber));
pGPIOhandle->pGPIOx->OSPEEDR|= tempreg;

//configure the pupd
tempreg =0;
tempreg = (pGPIOhandle->GPIO_Config_t.GPIO_PinPuPdControl << (2 * pGPIOhandle->GPIO_Config_t.GPIO_PinNumber));
pGPIOhandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOhandle->GPIO_Config_t.GPIO_PinNumber));
pGPIOhandle->pGPIOx->PUPDR |= tempreg;


tempreg = 0;
// output type
tempreg = (pGPIOhandle->GPIO_Config_t.GPIO_PinOPType <<  (pGPIOhandle->GPIO_Config_t.GPIO_PinNumber));
pGPIOhandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOhandle->GPIO_Config_t.GPIO_PinNumber) );
pGPIOhandle->pGPIOx->OTYPER |= tempreg;

//configure the alternate functionility

if(pGPIOhandle->GPIO_Config_t.GPIO_PinMode ==GPIO_MODE_ALTERNATE)
{
	uint8_t temp1, temp2;
	temp1 = pGPIOhandle->GPIO_Config_t.GPIO_PinNumber /8;
	temp2 = pGPIOhandle->GPIO_Config_t.GPIO_PinNumber %8;
	pGPIOhandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
	pGPIOhandle->pGPIOx->AFR[temp1] |= (pGPIOhandle->GPIO_Config_t.GPIO_PinAltFunMode << (4 * temp2));
}

}
void GPIO_DEINIT(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_RESET();
	}

	else if(pGPIOx == GPIOB)
		{
			GPIOB_RESET();
		}
	else if(pGPIOx == GPIOC)
		{
			GPIOC_RESET();
		}

	else if(pGPIOx == GPIOD)
		{
			GPIOD_RESET();
		}
	else if(pGPIOx == GPIOE)
		{
			GPIOE_RESET();
		}
	else if(pGPIOx == GPIOF)
		{
			GPIOF_RESET();
		}

	else if(pGPIOx == GPIOG)
		{
			GPIOG_RESET();
		}
	else if(pGPIOx == GPIOH)
		{
			GPIOH_RESET();
		}
	else if(pGPIOx == GPIOI)
		{
			GPIOI_RESET();
		}
}

//peripheral control for the gpio
void GPIO_PERI_CTRL(GPIO_RegDef_t *pGPIOx, uint8_t ENORDI)
{
	if(ENORDI == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_CLK_EN;
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_CLK_EN;
		}
		else if(pGPIOx == GPIOC)
				{
					GPIOC_CLK_EN;
				}
		else if(pGPIOx == GPIOD)
				{
					GPIOD_CLK_EN;
				}
		else if(pGPIOx == GPIOE)
				{
					GPIOE_CLK_EN;
				}
		else if(pGPIOx == GPIOF)
				{
					GPIOF_CLK_EN;
				}
		else if(pGPIOx == GPIOG)
				{
					GPIOG_CLK_EN;
				}
		else if(pGPIOx == GPIOH)
				{
					GPIOH_CLK_EN;
				}
		else if(pGPIOx == GPIOI)
				{
					GPIOI_CLK_EN;
				}
	}
	else
	{
		if(pGPIOx == GPIOA)
				{
					GPIOA_CLK_DI;
				}
				else if(pGPIOx == GPIOB)
				{
					GPIOB_CLK_DI;
				}
				else if(pGPIOx == GPIOC)
						{
							GPIOC_CLK_DI;
						}
				else if(pGPIOx == GPIOD)
						{
							GPIOD_CLK_DI;
						}
				else if(pGPIOx == GPIOE)
						{
							GPIOE_CLK_DI;
						}
				else if(pGPIOx == GPIOF)
						{
							GPIOF_CLK_DI;
						}
				else if(pGPIOx == GPIOG)
						{
							GPIOG_CLK_DI;
						}
				else if(pGPIOx == GPIOH)
						{
							GPIOH_CLK_DI;
						}
				else if(pGPIOx == GPIOI)
						{
							GPIOI_CLK_DI;
						}

	}
}

//READ AND WRITE TO THE GPIO
uint8_t GPIO_READ_INPUT_PIN(GPIO_RegDef_t *pGPIOx, uint8_t pinnumber)
{
	uint8_t read;
	read = (uint8_t)((pGPIOx->IDR >> pinnumber) & 0x00000001);
	return read;
}
uint8_t GPIO_READ_INPUT_PORT(GPIO_RegDef_t *pGPIOx, uint8_t pinnumber)
{
	uint32_t read;
	read = (uint32_t)(pGPIOx->IDR);
	return read;
}

void GPIO_WRITE_TO_INPUT_PIN(GPIO_RegDef_t *pGPIOx,uint8_t pinnumber,uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << pinnumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << pinnumber);
	}


}
void GPIO_WRITE_TO_INPUT_PORT(GPIO_RegDef_t *pGPIOx,uint8_t value)
{
	pGPIOx->ODR = value;
}
void GPIO_TOGGLE_PIN(GPIO_RegDef_t *pGPIOx,uint8_t pinnumber)
{
	pGPIOx->ODR ^= (1<< pinnumber);
}

//gpio irq and isr handling
void IRQ_INTERRUPT_CONFIG(uint8_t IRQNUMBER,uint8_t ENORDI)
{
	if(ENORDI == ENABLE)
	 {
		 if(IRQNUMBER <= 31)
		 {
			 //PROGRAM ISER0
			 *NVIC_ISER0 |= (1<<IRQNUMBER);
		 }
		 else if(IRQNUMBER >31 && IRQNUMBER <=64)
		 {
			 //program iser1
			 *NVIC_ISER1 |= (1<< (IRQNUMBER % 32));

		 }
		 else if(IRQNUMBER >64 && IRQNUMBER <=96)
			 {
				 //program iser1
				 *NVIC_ISER2 |= (1<< (IRQNUMBER % 64));

			 }
	 }
	 else
	 {
		 if(IRQNUMBER <= 31)
		 {
			 //PROGRAM ISER0
			 *NVIC_ICER0 |= (1<<IRQNUMBER);
		 }
		 else if(IRQNUMBER > 31 && IRQNUMBER <64)
		 {
			 //program iser1
			 *NVIC_ICER1 |= (1<< (IRQNUMBER % 32));

		 }
		 else if(IRQNUMBER >= 64 && IRQNUMBER <96)
			 {
				 //program iser1
				 *NVIC_ICER2 |= (1<< (IRQNUMBER % 64));

			 }

	}
}
void IRQ_PRIORITY(uint8_t IRQNUMBER,uint8_t IRQPRIORITY)
{
	uint8_t iprx = IRQNUMBER /4;
		uint8_t iprx_section = IRQNUMBER %4;
		uint8_t shift_amount  = (8 * iprx_section) +(8 - NO_PR_BITS_IMPLEMENTED);

		*(NVIC_PR_BASE_ADDR +iprx) |= (IRQPRIORITY << shift_amount);
}
void IRQ_HANDLING(uint8_t pinnumber )
{
	//clear the exti register  as per the pin number
		if(EXTI->PR1 &(1<< pinnumber))
		{
			//clear
			EXTI->PR1 |= (1<< pinnumber);
		}

}
