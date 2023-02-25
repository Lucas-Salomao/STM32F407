################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/fonts.c \
../Core/Src/game.c \
../Core/Src/gameOverImage.c \
../Core/Src/ili9341.c \
../Core/Src/imageStrengthAim.c \
../Core/Src/lcd_io_fsmc8.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/youWinImage.c 

OBJS += \
./Core/Src/fonts.o \
./Core/Src/game.o \
./Core/Src/gameOverImage.o \
./Core/Src/ili9341.o \
./Core/Src/imageStrengthAim.o \
./Core/Src/lcd_io_fsmc8.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/youWinImage.o 

C_DEPS += \
./Core/Src/fonts.d \
./Core/Src/game.d \
./Core/Src/gameOverImage.d \
./Core/Src/ili9341.d \
./Core/Src/imageStrengthAim.d \
./Core/Src/lcd_io_fsmc8.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/youWinImage.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/fonts.d ./Core/Src/fonts.o ./Core/Src/fonts.su ./Core/Src/game.d ./Core/Src/game.o ./Core/Src/game.su ./Core/Src/gameOverImage.d ./Core/Src/gameOverImage.o ./Core/Src/gameOverImage.su ./Core/Src/ili9341.d ./Core/Src/ili9341.o ./Core/Src/ili9341.su ./Core/Src/imageStrengthAim.d ./Core/Src/imageStrengthAim.o ./Core/Src/imageStrengthAim.su ./Core/Src/lcd_io_fsmc8.d ./Core/Src/lcd_io_fsmc8.o ./Core/Src/lcd_io_fsmc8.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/youWinImage.d ./Core/Src/youWinImage.o ./Core/Src/youWinImage.su

.PHONY: clean-Core-2f-Src

