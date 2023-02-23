################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/_0.c \
../Core/Src/_1.c \
../Core/Src/_2.c \
../Core/Src/_3.c \
../Core/Src/_4.c \
../Core/Src/_5.c \
../Core/Src/_6.c \
../Core/Src/_7.c \
../Core/Src/_8.c \
../Core/Src/_9.c \
../Core/Src/alvo.c \
../Core/Src/fonts.c \
../Core/Src/ili9341.c \
../Core/Src/lcd_io_fsmc8.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/_0.o \
./Core/Src/_1.o \
./Core/Src/_2.o \
./Core/Src/_3.o \
./Core/Src/_4.o \
./Core/Src/_5.o \
./Core/Src/_6.o \
./Core/Src/_7.o \
./Core/Src/_8.o \
./Core/Src/_9.o \
./Core/Src/alvo.o \
./Core/Src/fonts.o \
./Core/Src/ili9341.o \
./Core/Src/lcd_io_fsmc8.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/_0.d \
./Core/Src/_1.d \
./Core/Src/_2.d \
./Core/Src/_3.d \
./Core/Src/_4.d \
./Core/Src/_5.d \
./Core/Src/_6.d \
./Core/Src/_7.d \
./Core/Src/_8.d \
./Core/Src/_9.d \
./Core/Src/alvo.d \
./Core/Src/fonts.d \
./Core/Src/ili9341.d \
./Core/Src/lcd_io_fsmc8.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/_0.d ./Core/Src/_0.o ./Core/Src/_0.su ./Core/Src/_1.d ./Core/Src/_1.o ./Core/Src/_1.su ./Core/Src/_2.d ./Core/Src/_2.o ./Core/Src/_2.su ./Core/Src/_3.d ./Core/Src/_3.o ./Core/Src/_3.su ./Core/Src/_4.d ./Core/Src/_4.o ./Core/Src/_4.su ./Core/Src/_5.d ./Core/Src/_5.o ./Core/Src/_5.su ./Core/Src/_6.d ./Core/Src/_6.o ./Core/Src/_6.su ./Core/Src/_7.d ./Core/Src/_7.o ./Core/Src/_7.su ./Core/Src/_8.d ./Core/Src/_8.o ./Core/Src/_8.su ./Core/Src/_9.d ./Core/Src/_9.o ./Core/Src/_9.su ./Core/Src/alvo.d ./Core/Src/alvo.o ./Core/Src/alvo.su ./Core/Src/fonts.d ./Core/Src/fonts.o ./Core/Src/fonts.su ./Core/Src/ili9341.d ./Core/Src/ili9341.o ./Core/Src/ili9341.su ./Core/Src/lcd_io_fsmc8.d ./Core/Src/lcd_io_fsmc8.o ./Core/Src/lcd_io_fsmc8.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

