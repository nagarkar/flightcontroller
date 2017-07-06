################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main.c \
../src/stm32f0xx_it.c \
../src/syscalls.c \
../src/system_stm32f0xx.c 

OBJS += \
./src/main.o \
./src/stm32f0xx_it.o \
./src/syscalls.o \
./src/system_stm32f0xx.o 

C_DEPS += \
./src/main.d \
./src/stm32f0xx_it.d \
./src/syscalls.d \
./src/system_stm32f0xx.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DSTM32F051R8Tx -DARM_CM0 -DARM_MATH_CM0 -DSTM32F0 -DSTM32 -DSTM32F0DISCOVERY -DDEBUG -DUSE_STDPERIPH_DRIVER -DSTM32F051 -I"C:/Users/LOMAS/workspace/f0_CMSIS_dsp/api_fc/inc" -I"C:/Users/LOMAS/workspace/f0_CMSIS_dsp/Unity" -I"C:/Users/LOMAS/workspace/f0_CMSIS_dsp/inc" -I"C:/Users/LOMAS/workspace/f0_CMSIS_dsp/CMSIS/core" -I"C:/Users/LOMAS/workspace/f0_CMSIS_dsp/CMSIS/device" -I"C:/Users/LOMAS/workspace/f0_CMSIS_dsp/StdPeriph_Driver/inc" -I"C:/Users/LOMAS/workspace/f0_CMSIS_dsp/Utilities" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


