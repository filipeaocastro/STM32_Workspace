################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USB_DEVICE/Target/subdir.mk
-include USB_DEVICE/App/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
nrf-ShieldHost-STM-LED.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
nrf-ShieldHost-STM-LED.list \

OBJCOPY_HEX += \
nrf-ShieldHost-STM-LED.hex \


# All Target
all: nrf-ShieldHost-STM-LED.elf secondary-outputs

# Tool invocations
nrf-ShieldHost-STM-LED.elf: $(OBJS) $(USER_OBJS) C:\Users\Filipe\ Augusto\STM32CubeIDE\workspace_1.1.0\nrf-ShieldHost-STM-LED\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "nrf-ShieldHost-STM-LED.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\Filipe Augusto\STM32CubeIDE\workspace_1.1.0\nrf-ShieldHost-STM-LED\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="nrf-ShieldHost-STM-LED.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

nrf-ShieldHost-STM-LED.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "nrf-ShieldHost-STM-LED.list"
	@echo 'Finished building: $@'
	@echo ' '

nrf-ShieldHost-STM-LED.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "nrf-ShieldHost-STM-LED.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
