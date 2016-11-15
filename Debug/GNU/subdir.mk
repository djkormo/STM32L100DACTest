################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../GNU/startup_stm32l1xx_md.S 

OBJS += \
./GNU/startup_stm32l1xx_md.o 


# Each subdirectory must supply rules for building sources it contributes
GNU/%.o: ../GNU/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	arm-none-eabi-gcc -c -mthumb -mcpu=cortex-m3 -g -Wa,--warn -x assembler-with-cpp -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


