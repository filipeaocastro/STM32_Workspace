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
nrf-ShieldHost-STM_LED_v02.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
nrf-ShieldHost-STM_LED_v02.list \

OBJCOPY_HEX += \
nrf-ShieldHost-STM_LED_v02.hex \


# All Target
all: nrf-ShieldHost-STM_LED_v02.elf secondary-outputs

# Tool invocations
nrf-ShieldHost-STM_LED_v02.elf: $(OBJS) $(USER_OBJS) C:\Users\Filipe\ Augusto\STM32CubeIDE\workspace_1.1.0\nrf-ShieldHost-STM_LED_v02\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "nrf-ShieldHost-STM_LED_v02.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\Filipe Augusto\STM32CubeIDE\workspace_1.1.0\nrf-ShieldHost-STM_LED_v02\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="nrf-ShieldHost-STM_LED_v02.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

nrf-ShieldHost-STM_LED_v02.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "nrf-ShieldHost-STM_LED_v02.list"
	@echo 'Finished building: $@'
	@echo ' '

nrf-ShieldHost-STM_LED_v02.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "nrf-ShieldHost-STM_LED_v02.hex"
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
