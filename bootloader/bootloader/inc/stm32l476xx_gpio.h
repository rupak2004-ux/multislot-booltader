/*
 * stm32l476xx_gpio.h
 *
 *  Created on: Aug 21, 2025
 *      Author: rupak
 */

#ifndef INC_STM32L476XX_GPIO_H_
#define INC_STM32L476XX_GPIO_H_
#include "stm32l476xx.h"

//configuration structure for the gpio
typedef struct
{
	uint8_t GPIO_PinNumber;        // possible values from 0 to 15
	    uint8_t GPIO_PinMode;          // Input, Output, Alt Function, Analog
	    uint8_t GPIO_PinSpeed;         // Low, Medium, Fast, High
	    uint8_t GPIO_PinPuPdControl;   // No pull, Pull-up, Pull-down
	    uint8_t GPIO_PinOPType;        // Push-pull or Open-drain
	    uint8_t GPIO_PinAltFunMode;    // Alternate function mode
}GPIO_Config_t;

//handle structer for the gpio
typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_Config_t GPIO_Config_t;
}GPIO_Handle_t;

//gpio pin numbers
#define GPIO_PIN_NO_0    0
#define GPIO_PIN_NO_1    1
#define GPIO_PIN_NO_2    2
#define GPIO_PIN_NO_3    3
#define GPIO_PIN_NO_4    4
#define GPIO_PIN_NO_5    5
#define GPIO_PIN_NO_6    6
#define GPIO_PIN_NO_7    7
#define GPIO_PIN_NO_8    8
#define GPIO_PIN_NO_9    9
#define GPIO_PIN_NO_10   10
#define GPIO_PIN_NO_11   11
#define GPIO_PIN_NO_12   12
#define GPIO_PIN_NO_13   13
#define GPIO_PIN_NO_14   14
#define GPIO_PIN_NO_15   15

//DIFFERENT MODES FOR THE GPIO

#define GPIO_MODE_INPUT      0
#define GPIO_MODE_OUTPUT     1
#define GPIO_MODE_ALTERNATE  2
#define GPIO_MODE_ANALOG     3

// GPIO OUTPUT TYPE
#define GPIO_TYPE_PUSH_PULL  0
#define GPIO_TYPE_OPEN_DRAIN 1

//GPIO SPEED
#define GPIO_SPEED_LOW_SPEED          0
#define GPIO_SPEED_MEDIUM_SPEED       1
#define GPIO_SPEED_HIGH_SPEED         2
#define GPIO_SPEED_VERY_HIGH_SPEED    3

//GPIO PULL UP AND PULL DOWN
#define GPIO_PUPD_NO         0
#define GPIO_PUPD_PULLUP     1
#define GPIO_PUPD_PULLDOWN   2


#define GPIO_PIN_SET   SET
#define GPIO_PIN_RESET  RESET

//interrupt based definition for the gpio
#define GPIO_MODE_IT_RT         4
#define GPIO_MODE_IT_FT         5
#define GPIO_MODE_IT_RFT        6

//API FOR THE GPIO

//init and deinit apii
void GPIO_INIT(GPIO_Handle_t *pGPIOhandle);
void GPIO_DEINIT(GPIO_RegDef_t *pGPIOx);

//peripheral control for the gpio
void GPIO_PERI_CTRL(GPIO_RegDef_t *pGPIOx, uint8_t ENORDI);

//READ AND WRITE TO THE GPIO
uint8_t GPIO_READ_INPUT_PIN(GPIO_RegDef_t *pGPIOx, uint8_t pinnumber);
uint8_t GPIO_READ_INPUT_PORT(GPIO_RegDef_t *pGPIOx, uint8_t pinnumber);

void GPIO_WRITE_TO_INPUT_PIN(GPIO_RegDef_t *pGPIOx,uint8_t pinnumber,uint8_t value);
void GPIO_WRITE_TO_INPUT_PORT(GPIO_RegDef_t * pGPIOx,uint8_t value);
void GPIO_TOGGLE_PIN(GPIO_RegDef_t *pGPIOx,uint8_t pinnumber);

//gpio irq and isr handling
void IRQ_INTERRUPT_CONFIG(uint8_t IRQNUMBER,uint8_t ENORDI);
void IRQ_PRIORITY(uint8_t IRQNUMBER,uint8_t IRQPRIORITY);
void IRQ_HANDLING(uint8_t pinnumber );

#endif /* INC_STM32L476XX_GPIO_H_ */
