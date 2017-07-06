################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include unit_test/subdir.mk
-include startup/subdir.mk
-include src/subdir.mk
-include api_fc/src/subdir.mk
-include Utilities/subdir.mk
-include Unity/subdir.mk
-include StdPeriph_Driver/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: f0_CMSIS_dsp.elf

# Tool invocations
f0_CMSIS_dsp.elf: $(OBJS) $(USER_OBJS) C:\Users\LOMAS\workspace\f0_CMSIS_dsp\LinkerScript.ld
	@echo 'Building target: $@'
	@echo 'Invoking: MCU GCC Linker'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -L"C:\Users\LOMAS\workspace\CMSIS-SP-00300-r3p1-00rel0\CMSIS\Lib\GCC" -mfloat-abi=soft -T"C:\Users\LOMAS\workspace\f0_CMSIS_dsp\LinkerScript.ld" -Wl,-Map=output.map -Wl,--gc-sections -o "f0_CMSIS_dsp.elf" @"objects.list" $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	-@echo 'Generating binary and Printing size information:'
	arm-none-eabi-objcopy -O binary "f0_CMSIS_dsp.elf" "f0_CMSIS_dsp.bin"
	arm-none-eabi-size "f0_CMSIS_dsp.elf"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
