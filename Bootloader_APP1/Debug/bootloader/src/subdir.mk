################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bootloader/src/flash.c \
../bootloader/src/stm32l476_rcc.c \
../bootloader/src/stm32l476xx_gpio.c \
../bootloader/src/stm32l476xx_usart.c 

OBJS += \
./bootloader/src/flash.o \
./bootloader/src/stm32l476_rcc.o \
./bootloader/src/stm32l476xx_gpio.o \
./bootloader/src/stm32l476xx_usart.o 

C_DEPS += \
./bootloader/src/flash.d \
./bootloader/src/stm32l476_rcc.d \
./bootloader/src/stm32l476xx_gpio.d \
./bootloader/src/stm32l476xx_usart.d 


# Each subdirectory must supply rules for building sources it contributes
bootloader/src/%.o bootloader/src/%.su bootloader/src/%.cyclo: ../bootloader/src/%.c bootloader/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4 -DSTM32 -DNUCLEO_L476RG -DSTM32L476RGTx -c -I../Inc -I"/home/rupak/Documents/bootloader/Bootloader_APP1" -I"/home/rupak/Documents/bootloader/Bootloader_APP1/bootloader/inc" -I"/home/rupak/Documents/bootloader/Bootloader_APP1" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bootloader-2f-src

clean-bootloader-2f-src:
	-$(RM) ./bootloader/src/flash.cyclo ./bootloader/src/flash.d ./bootloader/src/flash.o ./bootloader/src/flash.su ./bootloader/src/stm32l476_rcc.cyclo ./bootloader/src/stm32l476_rcc.d ./bootloader/src/stm32l476_rcc.o ./bootloader/src/stm32l476_rcc.su ./bootloader/src/stm32l476xx_gpio.cyclo ./bootloader/src/stm32l476xx_gpio.d ./bootloader/src/stm32l476xx_gpio.o ./bootloader/src/stm32l476xx_gpio.su ./bootloader/src/stm32l476xx_usart.cyclo ./bootloader/src/stm32l476xx_usart.d ./bootloader/src/stm32l476xx_usart.o ./bootloader/src/stm32l476xx_usart.su

.PHONY: clean-bootloader-2f-src

