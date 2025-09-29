/*
 * stm32l476xx_usart.h
 *
 *  Created on: Aug 23, 2025
 *      Author: rupak
 */

#ifndef INC_STM32L476XX_USART_H_
#define INC_STM32L476XX_USART_H_

#include "stm32l476xx.h"

typedef struct
{
	uint8_t USART_MODE;
	uint32_t USART_BAUD_RATE;
	uint8_t USART_STOP_BITS;
	uint8_t USART_WORD_LEN;
	uint8_t USART_PARITY;
	uint8_t USART_HW_FLOW_CTRL;

}USART_Config_t;

typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t  USART_Config_t;
	uint8_t *pTXBuffer;
	uint8_t *pRXBuffer;
	uint8_t TXLEN;
	uint8_t RXLEN;
	uint8_t TXBusyState;
	uint8_t RXBusyState;
}USART_Handle_t;

//mode for the usart
#define USART_MODE_ONLY_TX   0
#define USART_MODE_ONLY_RX   1
#define USART_MODE_TXRX      2

//baud rate for the usart
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

//usart stop bits
#define USART_STOP_BIT_0_5    1
#define USART_STOP_BIT_1      0
#define USART_STOP_BIT_1_5    3
#define USART_STOP_BIT_2      2

//WORD LEN FOR THE USART

#define USART_WORDLEN_8BITS   0
#define USART_WORDLEN_9BITS   1

//PARITY FOR THE USART
#define USART_PARITY_EN_ODD   0
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   2

// HW FLOW CTRL FOR USART
/*Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

//FLAG FOR THE USART
#define USART_FLAG_TXE    (1<<7)
#define USART_FLAG_TC     (1<<6)
#define USART_FLAG_RXNE    (1<<5)

#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2
#define USART_READY 0

#define 	USART_EVENT_TX_CMPLT   0
#define		USART_EVENT_RX_CMPLT   1
#define		USART_EVENT_IDLE      2
#define		USART_EVENT_CTS       3
#define		USART_EVENT_PE        4
#define		USART_ERR_FE     	5
#define		USART_ERR_NE    	 6
#define		USART_ERR_ORE    	7
//APIS FOR THE USART

//init and deinit api's
void USART_INIT(USART_Handle_t *pUSARTHandle);
void USART_DEINIT(USART_RegDef_t *pUSARTx);

//peripheral clk control
void USART_PER_CLK_CTRL(USART_RegDef_t *pUSARTx, uint8_t ENORDI);

//data transfer and receiver
void USART_SEND_DATA(USART_Handle_t *pUSARTHandle,uint8_t *pTXBuffer, uint8_t len);
void USART_REC_DATA(USART_Handle_t *pUSARTHandle,uint8_t *pRXBuffer,uint8_t len);

// interrupt base uart api
uint8_t  USART_SEND_DATA_IT(USART_Handle_t *pUSARTHandle,uint8_t *pTXBuffer, uint8_t len);
uint8_t  USART_REC_DATA_IT(USART_Handle_t *pUSARTHandle,uint8_t *pRXBuffer, uint8_t len);
/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);
//void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);


#endif /* INC_STM32L476XX_USART_H_ */
