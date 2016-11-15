################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Board/stm32l_discovery_lcd.c 

OBJS += \
./Board/stm32l_discovery_lcd.o 

C_DEPS += \
./Board/stm32l_discovery_lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Board/%.o: ../Board/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DSTM32L1XX_MD -I"/home/jshankar/Projects/Eclipse/STM32L1xx/STM32L1xx_StdPeriph_Template" -I"/home/jshankar/Projects/Eclipse/STM32L1xx/STM32L1xx_StdPeriph_Template/Board" -I"/home/jshankar/Projects/Eclipse/STM32L1xx/STM32L1xx_StdPeriph_Template/User" -I"/home/jshankar/Projects/Eclipse/STM32L1xx/STM32L1xx_StdPeriph_Template/STM32L1xx_StdPeriph_Driver/inc" -I"/home/jshankar/Projects/Eclipse/STM32L1xx/STM32L1xx_StdPeriph_Template/CMSIS/ST/STM32L1xx/Include" -I"/home/jshankar/Projects/Eclipse/STM32L1xx/STM32L1xx_StdPeriph_Template/CMSIS/Include" -O0 -g3 -DWARF2 -Wall -c -fmessage-length=0 -mthumb -mcpu=cortex-m3 -O0 -ffunction-sections -fdata-sections -g -Wall -w -mlong-calls -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


