################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../api_fc/src/coefnts.c \
../api_fc/src/controller.c \
../api_fc/src/rotation.c \
../api_fc/src/snptrjgen.c \
../api_fc/src/trajgen.c 

OBJS += \
./api_fc/src/coefnts.o \
./api_fc/src/controller.o \
./api_fc/src/rotation.o \
./api_fc/src/snptrjgen.o \
./api_fc/src/trajgen.o 

C_DEPS += \
./api_fc/src/coefnts.d \
./api_fc/src/controller.d \
./api_fc/src/rotation.d \
./api_fc/src/snptrjgen.d \
./api_fc/src/trajgen.d 


# Each subdirectory must supply rules for building sources it contributes
api_fc/src/%.o: ../api_fc/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DSTM32F051R8Tx -DARM_CM0 -DARM_MATH_CM0 -DSTM32F0 -DSTM32 -DSTM32F0DISCOVERY -DDEBUG -DUSE_STDPERIPH_DRIVER -DSTM32F051 -I"C:/Users/LOMAS/workspace/f0_CMSIS_dsp/api_fc/inc" -I"C:/Users/LOMAS/workspace/f0_CMSIS_dsp/Unity" -I"C:/Users/LOMAS/workspace/f0_CMSIS_dsp/inc" -I"C:/Users/LOMAS/workspace/f0_CMSIS_dsp/CMSIS/core" -I"C:/Users/LOMAS/workspace/f0_CMSIS_dsp/CMSIS/device" -I"C:/Users/LOMAS/workspace/f0_CMSIS_dsp/StdPeriph_Driver/inc" -I"C:/Users/LOMAS/workspace/f0_CMSIS_dsp/Utilities" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


