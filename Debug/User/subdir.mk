################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/algorithm.c \
../User/resources.c \
../User/stm32l1xx_it.c 

OBJS += \
./User/algorithm.o \
./User/resources.o \
./User/stm32l1xx_it.o 

C_DEPS += \
./User/algorithm.d \
./User/resources.d \
./User/stm32l1xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DSTM32L1XX_MD -I"C:\developing\stm32\workspace\STM32L100DACTest" -I"C:\developing\stm32\workspace\STM32L100DACTest\Board" -I"C:\developing\stm32\workspace\STM32L100DACTest\User" -I"C:\developing\stm32\workspace\STM32L100DACTest\STM32L1xx_StdPeriph_Driver\inc" -I"C:\developing\stm32\workspace\STM32L100DACTest\CMSIS\ST\STM32L1xx\Include" -I"C:\developing\stm32\workspace\STM32L100DACTest\CMSIS\Include" -O0 -g3 -DWARF2 -Wall -c -fmessage-length=0 -mthumb -mcpu=cortex-m3 -O0 -ffunction-sections -fdata-sections -g -Wall -w -mlong-calls -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


