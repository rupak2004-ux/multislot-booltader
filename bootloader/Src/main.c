#include <stdint.h>
#include "stm32l476xx.h"
#include <string.h>

#define TEST_ADDR   0x08020000UL   // Last page for 512KB device (adjust if smaller)
#define TEST_PAGE   ((TEST_ADDR - FLASH_BASE_ADDR) / FLASH_PAGE_SIZE)
//#define CUSTOM_FUNC  __attribute__((section(".custom_memory")))   //designed in the lonker script own memory block


char msg[1024] = "UART Tx testing...\n\r";
//unsigned char __attribute__((section(".custom_block")))  buffer[10];
//unsigned char __attribute__((section(".custom_flash")))  buff[10];
void delay(void)
{
    for(uint32_t i =0; i<500000;i++);
}

//void  CUSTOM_FUNC led_toggle(void)
//{
//	GPIO_TOGGLE_PIN(GPIOA, GPIO_PIN_NO_5);
//}

USART_Handle_t usart2_handle;

void SystemClock_Init(void) {
    // Enable MSI clock (4 MHz default)
    RCC->CR |= (1<<0);
    while (!(RCC->CR & (1U << 1)));  // Wait for MSI ready

    // Set flash latency: 2 wait states for 4 MHz at 1.2V (RM0351 Table 13)
    FLASH->ACR &= ~(0x7<<0);
    FLASH->ACR |= (0x2<<0);

    // Select MSI as system clock
    RCC->CFGR &= ~(0x3<<0);
    RCC->CFGR |= (0x0 <<0);
    while ((RCC->CFGR & (0x3<<2)) != 0x0);  // Wait for switch
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

int main(void)
{
    SystemClock_Init();
    //jmp_to_default();

    // Initialize LED (GPIOA Pin 5)
    GPIO_Handle_t GpioLed;
    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_Config_t.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioLed.GPIO_Config_t.GPIO_PinMode = GPIO_MODE_OUTPUT;
    GpioLed.GPIO_Config_t.GPIO_PinSpeed = GPIO_SPEED_HIGH_SPEED;
    GpioLed.GPIO_Config_t.GPIO_PinOPType = GPIO_TYPE_PUSH_PULL;
    GpioLed.GPIO_Config_t.GPIO_PinPuPdControl = GPIO_PUPD_NO;
    GPIO_INIT(&GpioLed);

    // Flash test result flag
    uint8_t flash_ok = 0;

    // Test flash operations (done only once)
    uint64_t pattern = 0x1122334455667788ULL;
    uint64_t verify = 0;
    FLASH_STATUS_T st = FLASH_ERR_PROG;
    uint8_t unlocked = FLASH_UNLOCK();


    if (unlocked) {
        // Check write protection for test page (bank 2)
        if (TEST_PAGE >= 256) {
            uint32_t page_in_bank = TEST_PAGE - 256;
            if ((FLASH->WRP2AR & (1 << page_in_bank)) || (FLASH->WRP2BR & (1 << page_in_bank))) {
                st = FLASH_ERR_WRITE_PROTECT;
            }
        }

        // Erase test page if not protected
        if (st == FLASH_ERR_PROG) {
            st = FLASH_ERASE_PAGE(TEST_PAGE);
        }

        // Program test data if erase OK
        if (st == FLASH_OK) {
            st = FLASH_PROGRAM_DOUBLE_WORD(TEST_ADDR, pattern);
        }

        // Verify written data if program OK
        if (st == FLASH_OK) {
            verify = *(volatile uint64_t*)TEST_ADDR;
            if (verify == pattern) {
                flash_ok = 1;  //
            }
        }

        // Lock flash after operations
        FLASH_LOCK();
    }


    while (1)
    {
        if (flash_ok) {
            // Success: Slow blink (500 ms)
         GPIO_TOGGLE_PIN(GPIOA, GPIO_PIN_NO_5);

            delay();

            USART_SEND_DATA(&usart2_handle, (uint8_t*)&msg, 1);
        } else {
            // Failure: Fast blink (200 ms)
//            GPIO_TOGGLE_PIN(GPIOA, GPIO_PIN_NO_5);
        	GPIO_WRITE_TO_INPUT_PIN(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_SET);
            for (volatile uint32_t i=0; i<200000; i++);
        }
    }

    return 0;
}
