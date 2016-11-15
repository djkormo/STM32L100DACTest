################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/subdir.mk
-include STM32L1xx_StdPeriph_Driver/src/subdir.mk
-include GNU/subdir.mk
-include CMSIS/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: STM32L100DACTest.elf

# Tool invocations
STM32L100DACTest.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross GCC Linker'
	arm-none-eabi-gcc -static -mthumb -mcpu=cortex-m3 -T"C:\developing\stm32\workspace\STM32L100DACTest\GNU\STM32_flash.ld" -static -Wl,-cref,-u,Reset_Handler "-Wl,-Map=STM32L100DACTest.map" -Wl,--gc-sections -Wl,--defsym=malloc_getpagesize_P=0x1000 -o "STM32L100DACTest.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) STM32L100DACTest.elf
	-@echo ' '

post-build:
	-@echo 'converting elf to bin'
	-arm-none-eabi-objcopy -O binary STM32L100DACTest.elf STM32L100DACTest.bin
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
