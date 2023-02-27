################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := MOC32-Aula6-FSM-Menu Interativo
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
MOC32-Aula6-FSM-Menu\ Interativo.elf \

MAP_FILES += \
MOC32-Aula6-FSM-Menu\ Interativo.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
MOC32-Aula6-FSM-Menu\ Interativo.list \


# All Target
all: main-build

# Main-build Target
main-build: MOC32-Aula6-FSM-Menu\ Interativo.elf secondary-outputs

# Tool invocations
MOC32-Aula6-FSM-Menu\ Interativo.elf MOC32-Aula6-FSM-Menu\ Interativo.map: $(OBJS) $(USER_OBJS) C:\Users\salom\Documents\STM32F407\MOC32\MOC32-Aula6-FSM-Menu\ Interativo\STM32F407VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "MOC32-Aula6-FSM-Menu Interativo.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\salom\Documents\STM32F407\MOC32\MOC32-Aula6-FSM-Menu Interativo\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="MOC32-Aula6-FSM-Menu Interativo.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

MOC32-Aula6-FSM-Menu\ Interativo.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "MOC32-Aula6-FSM-Menu Interativo.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) MOC32-Aula6-FSM-Menu\ Interativo.elf MOC32-Aula6-FSM-Menu\ Interativo.list MOC32-Aula6-FSM-Menu\ Interativo.map MOC32-Aula6-FSM-Menu\\ Interativo.elf default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
