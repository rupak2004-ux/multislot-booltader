/*
 * stm32l476xx_usart.c
 *
 *  Created on: Aug 23, 2025
 *      Author: rupak
 */

#include "stm32l476xx_usart.h"
#include "stm32l476xx_rcc.h"


//APIS FOR THE USART

//init and deinit api's
void USART_INIT(USART_Handle_t *pUSARTHandle)
{   //clock for the usart
	USART_PER_CLK_CTRL(pUSARTHandle->pUSARTx, ENABLE);
	uint32_t tempreg=0;
	//configure the mode of the usart
	if(pUSARTHandle->USART_Config_t.USART_MODE ==  USART_MODE_ONLY_TX)
	{
		tempreg |= (1<<3);
	}
	else if(pUSARTHandle->USART_Config_t.USART_MODE == USART_MODE_ONLY_RX)
	{
		tempreg |= (1<<2);
	}
	else if(pUSARTHandle->USART_Config_t.USART_MODE == USART_MODE_TXRX)
	{
		tempreg  |= ((1<<3)|(1<<2));
	}

	//configure the length of the data
	tempreg |= ((pUSARTHandle->USART_Config_t.USART_WORD_LEN & 0x2) << 27) \
	        |  ((pUSARTHandle->USART_Config_t.USART_WORD_LEN & 0x1) << 12);


	//configure the parity for the usart
	if(pUSARTHandle->USART_Config_t.USART_PARITY == USART_PARITY_EN_EVEN)
	{
		tempreg|= (1<<10);
		//dont need to configure the even parity by default it is set to zero and enabled
	}
	else if (pUSARTHandle->USART_Config_t.USART_PARITY == USART_PARITY_EN_ODD)
	{
		tempreg|= (1<<10);
		//enable the odd parity
		tempreg|= (1<<9);

	}

	//storing it in the cr1 reg
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	//CONFIGURE THE CR2 REGISTER FOR NO. OF STOP BITS
	tempreg =0;
	tempreg|= pUSARTHandle->USART_Config_t.USART_STOP_BITS << 12;
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	//configure the cr3 register
	tempreg =0;
	 if(pUSARTHandle->USART_Config_t.USART_HW_FLOW_CTRL == USART_HW_FLOW_CTRL_CTS)
	 {
		 tempreg |= (1<<9);
	 }
	 else if(pUSARTHandle->USART_Config_t.USART_HW_FLOW_CTRL == USART_HW_FLOW_CTRL_RTS)
	 {
		 tempreg |= (1<<8);
	 }
	 else if(pUSARTHandle->USART_Config_t.USART_HW_FLOW_CTRL == USART_HW_FLOW_CTRL_CTS_RTS)
	 {
		 tempreg|= ((1<<9)|(1<<8));
	 }

	 //program the cr3 reg
	 pUSARTHandle->pUSARTx->CR3 = tempreg;

	 //configure the brr reg
	  USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config_t.USART_BAUD_RATE);



}
void USART_DEINIT(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_RESET();
	}
	else if(pUSARTx == USART2)
	{
		USART2_RESET();
	}
	else if(pUSARTx == USART3)
	{
		USART3_RESET();
	}
	else if(pUSARTx == UART4)
		{
			UART4_RESET();
		}
	else if(pUSARTx == UART5)
		{
			UART5_RESET();
		}
}

//peripheral clk control
void USART_PER_CLK_CTRL(USART_RegDef_t *pUSARTx, uint8_t ENORDI)
{
	if(ENORDI == ENABLE)
		{
			if(pUSARTx == USART1)
			{
				USART1_CLK_EN;
			}
			else if(pUSARTx == USART2)
			{
				USART2_CLK_EN;
			}
			else if(pUSARTx == USART3)
					{
				USART3_CLK_EN;
					}
			else if(pUSARTx == UART4)
					{
				UART4_CLK_EN;
					}
			else if (pUSARTx == UART5)
			{
				UART5_CLK_EN;
			}
		}
	else
	{
		if(pUSARTx == USART1)
					{
						USART1_CLK_DI;
					}
					else if(pUSARTx == USART2)
					{
						USART2_CLK_DI;
					}
					else if(pUSARTx == USART3)
							{
						USART3_CLK_DI;
							}
					else if(pUSARTx == UART4)
							{
						UART4_CLK_DI;
							}
					else if (pUSARTx == UART5)
					{
						UART5_CLK_DI;
					}
	}
}

//data transfer and receiver
void USART_SEND_DATA(USART_Handle_t *pUSARTHandle,uint8_t *pTXBuffer, uint8_t len)
{
uint16_t *pdata;

//transfer  the data till the length become zero
for(uint8_t i=0;i<len;i++)
{
	//check transfer buffer is empty or not
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

	if(pUSARTHandle->USART_Config_t.USART_WORD_LEN == USART_WORDLEN_9BITS)
	{
		pdata = (uint16_t*)pTXBuffer;
		pUSARTHandle->pUSARTx->TDR = (*pdata & (uint16_t)0x01FF);

		if(pUSARTHandle->USART_Config_t.USART_PARITY == USART_PARITY_DISABLE)
		{
			pTXBuffer++;
			pTXBuffer++;
		}
		else
		{
			pTXBuffer++;
		}
	}
	else
	{
		//8 bit data transfer
		pUSARTHandle->pUSARTx->TDR = (*pTXBuffer & (uint8_t) 0xFF);
		pTXBuffer++;
	}


}
while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}



void USART_REC_DATA(USART_Handle_t *pUSARTHandle,uint8_t *pRXBuffer,uint8_t len)
{
	for(uint8_t i=0;i<len;i++)
	{
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		if(pUSARTHandle->USART_Config_t.USART_WORD_LEN ==USART_WORDLEN_9BITS)
		{

			if(pUSARTHandle->USART_Config_t.USART_PARITY == USART_PARITY_DISABLE)
			{
				*((uint16_t*)pRXBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint16_t)0x01FF);

				pRXBuffer++;
				pRXBuffer++;
			}
			else
			{
				*((uint16_t*)pRXBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint8_t)0xFF);
				pRXBuffer++;
			}
		}
		else
		{
			if(pUSARTHandle->USART_Config_t.USART_PARITY == USART_PARITY_DISABLE)
						{
							*((uint16_t*)pRXBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint16_t)0xFF);

						}
						else
						{
							*((uint16_t*)pRXBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint8_t)0x7F);

						}
			pRXBuffer++;
		}
	}
}




/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pUSARTx->CR1 |= (1<<0);
	}
	else
	{
		pUSARTx->CR1 &= ~(1<<0);
	}
}
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	if(pUSARTx->ISR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
    uint32_t PCLKx;
    uint32_t usartdiv;
    uint32_t brr;


    if(pUSARTx == USART1 )
    {
        PCLKx = RCC_GetPCLK2Value(); // APB2
    }
    else
    {
        PCLKx = RCC_GetPCLK1Value(); // APB1
    }

    // 2. Check OVER8 (CR1 bit 15)
    if(pUSARTx->CR1 & (1 << 15))  // oversampling by 8
    {
        usartdiv = (2 * PCLKx + BaudRate/2) / BaudRate;  // rounded
        brr = (usartdiv & 0xFFF0) | ((usartdiv & 0x000F) >> 1);
    }
    else // oversampling by 16
    {
        usartdiv = (PCLKx + BaudRate/2) / BaudRate; // rounded
        brr = usartdiv;
    }

    // 3. Write BRR
    pUSARTx->BRR = brr;
}

uint8_t USART_SEND_DATA_IT(USART_Handle_t *pUSARTHandle,uint8_t *pTXBuffer, uint8_t len)
{
	uint8_t txstate = pUSARTHandle->TXBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TXLEN =len;
		pUSARTHandle->pTXBuffer = pTXBuffer;
		pUSARTHandle->TXBusyState = USART_BUSY_IN_TX;

		//enabling the txe interrupt
		pUSARTHandle->pUSARTx->CR1 |= (1<< 7);

		//enabling the transfer complete register
		pUSARTHandle->pUSARTx->CR1 |= (1 << 6);

	}
	return txstate;

}
uint8_t  USART_REC_DATA_IT(USART_Handle_t *pUSARTHandle,uint8_t *pRXBuffer, uint8_t len)
{
uint8_t rxstate = pUSARTHandle->RXBusyState;

if(rxstate != USART_BUSY_IN_RX)
{
	pUSARTHandle->RXLEN = len;
	pUSARTHandle->RXBusyState = USART_BUSY_IN_RX;
	pUSARTHandle->pRXBuffer = pRXBuffer;

 // (void)pUSARTHandle->pUSARTx->RDR;

	//enabling the interupt receive complete
	pUSARTHandle->pUSARTx->CR1 |= (1<< 5);

}
return rxstate;

}

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}

void USART_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}


void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

	uint32_t temp1 , temp2, temp3;

	uint16_t *pdata;

/*************************Check for TC flag ********************************************/

    //Implement the code to check the state of TC bit in the ISR
	temp1 = pUSARTHandle->pUSARTx->ISR & ( 1 << 6);

	 //Implement the code to check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << 6);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TXBusyState== USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TXLEN )
			{
				//Implement the code to clear the TC flag

				 pUSARTHandle->pUSARTx->CR1 &= ~(1 << 7);
				            pUSARTHandle->pUSARTx->CR1 &= ~(1 << 6);
				pUSARTHandle->pUSARTx->ICR |= ( 1 << 6);

				//Implement the code to clear the TCIE control bit

				//Reset the application state
				pUSARTHandle->TXBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTXBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TXLEN = 0;

				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->ISR & ( 1 << 7);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << 7);


	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pUSARTHandle->TXBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TXLEN > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config_t.USART_WORD_LEN == USART_WORDLEN_9BITS)
				{
					//if 9BIT load the TDR with 2bytes masking  the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTXBuffer;
					pUSARTHandle->pUSARTx->TDR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config_t.USART_PARITY == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTXBuffer++;
						pUSARTHandle->pTXBuffer++;
						pUSARTHandle->TXLEN-=2;
					}
					else
					{
						//Parity bit is used in this transfer . so 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTXBuffer++;
						pUSARTHandle->TXLEN-=1;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->TDR = (*pUSARTHandle->pTXBuffer  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTXBuffer++;
					pUSARTHandle->TXLEN-=1;
				}

			}
			if (pUSARTHandle->TXLEN == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << 7);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->ISR & ( 1 << 5);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << 5);


	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne
		if(pUSARTHandle->RXBusyState == USART_BUSY_IN_RX)
		{
			if(pUSARTHandle->RXLEN > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config_t.USART_WORD_LEN == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config_t.USART_PARITY == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 9bits will be of user data

						//read only first 9 bits so mask the RDR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRXBuffer) = (pUSARTHandle->pUSARTx->RDR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRXBuffer++;
						pUSARTHandle->pRXBuffer++;
						pUSARTHandle->RXLEN-=2;
					}
					else
					{
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRXBuffer = (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0xFF);
						 pUSARTHandle->pRXBuffer++;
						 pUSARTHandle->RXLEN-=1;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config_t.USART_PARITY == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *pUSARTHandle->pRXBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0xFF);

					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRXBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRXBuffer++;
					 pUSARTHandle->RXLEN-=1;
				}


			}//if of >0

			if(! pUSARTHandle->RXLEN)
			{
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << 5);
				pUSARTHandle->RXBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}


/*************************Check for CTS flag ********************************************/


	//Implement the code to check the status of CTS bit in the ISR
	temp1 = pUSARTHandle->pUSARTx->ISR & ( 1 << 10);

	//Implement the code to check the state of CTSE bit in CR3
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << 9);

	//Implement the code to check the state of CTSIE bit in CR3
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << 10);


	if(temp1  && temp2 )
	{
		//Implement the code to clear the CTS flag in ISR
		pUSARTHandle->pUSARTx->ICR |= ( 1 << 9);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the ISR
	temp1 = pUSARTHandle->pUSARTx->ISR & ( 1 << 4);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << 4);


	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag.
		temp1 = pUSARTHandle->pUSARTx->ICR |=( 1 <<4);

		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the ISR
	temp1 = pUSARTHandle->pUSARTx->ISR & (1 << 3);

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << 5);


	if(temp1  && temp2 )
	{
		pUSARTHandle->pUSARTx->ICR |= (1<< 3);

		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}



/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << 0) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->ISR;
		if(temp1 & ( 1 << 1))
		{
			//CLEARING THIS REGISTER
			pUSARTHandle->pUSARTx->ICR |= (1 <<1);
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}

		if(temp1 & ( 1 << 2) )
		{
			pUSARTHandle->pUSARTx->ICR |= (1 <<2);
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
		}

		if(temp1 & ( 1 << 3) )
		{
			pUSARTHandle->pUSARTx->ICR |= (1 <<3);
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}


}

__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event)
{

}




